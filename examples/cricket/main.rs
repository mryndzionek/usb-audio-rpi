#![no_std]
#![no_main]
#![feature(array_chunks)]

#[rtic::app(device = bsp::hal::pac, peripherals = true, dispatchers = [PIO0_IRQ_0])]
mod app {

    use bsp::hal::{
        clocks::init_clocks_and_plls, gpio::PushPull, sio::Sio, timer::Alarm, usb::UsbBus,
        watchdog::Watchdog,
    };
    use bsp::XOSC_CRYSTAL_FREQ;
    use core::iter::Cycle;
    use core::slice::ArrayChunks;
    use cortex_m::prelude::_embedded_hal_watchdog_Watchdog;
    use cortex_m::prelude::_embedded_hal_watchdog_WatchdogEnable;
    use defmt_rtt as _;
    use embedded_hal::digital::v2::OutputPin;
    use fugit::MicrosDurationU32;
    use panic_probe as _;
    use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
    use usbd_audio::{AudioClass, AudioClassBuilder, Format, StreamConfig, TerminalType};
    use vcc_gnd_yd_rp2040 as bsp;

    use usb_device::class_prelude::*;

    const SCAN_TIME_US: MicrosDurationU32 = MicrosDurationU32::millis(1);

    static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<bsp::hal::usb::UsbBus>> = None;
    const VID: u16 = 0x16c0;
    const PID: u16 = 0x27dd;

    const CHIRP: [i16; 768] = [
        0, 64, 179, 323, 460, 561, 599, 536, 385, 140, -184, -586, -969, -1316, -1588, -1717,
        -1689, -1488, -1070, -534, 57, 762, 1516, 2135, 2609, 2879, 2882, 2615, 2085, 1246, 298,
        -743, -1785, -2795, -3529, -3986, -4108, -3818, -3182, -2224, -1014, 349, 1749, 3059, 4228,
        4971, 5302, 5172, 4496, 3431, 2015, 362, -1533, -3216, -4676, -5775, -6419, -6450, -5909,
        -4704, -3121, -1207, 703, 2953, 4834, 6345, 7342, 7711, 7360, 6347, 4742, 2481, 102, -2352,
        -4842, -6768, -8149, -8847, -8732, -7829, -6212, -4006, -1161, 1409, 4155, 6600, 8645,
        9796, 10110, 9447, 7960, 5740, 2965, -396, -3538, -6436, -8830, -10590, -11290, -11013,
        -9762, -7409, -4516, -1168, 2348, 5726, 8666, 10901, 12276, 12458, 11551, 9613, 6523, 3021,
        -809, -4636, -8382, -11149, -12986, -13714, -13154, -11433, -8680, -4794, -704, 3169, 7184,
        10878, 13365, 14720, 14807, 13432, 10923, 7418, 2838, -1722, -6199, -10200, -13575, -15513,
        -16122, -15328, -12939, -9523, -5226, -412, 4505, 9096, 12954, 15897, 17215, 17046, 15384,
        12040, 7814, 2849, -2429, -7966, -12434, -15871, -17965, -18478, -17284, -14564, -10148,
        -5092, 0, 5609, 11188, 15399, 18314, 19664, 19196, 17018, 13328, 8431, 2245, -3749, -9483,
        -14812, -18457, -20530, -20834, -19107, -15761, -11001, -5229, 1062, 7328, 13021, 17637,
        20937, 22126, 21396, 18486, 14091, 8420, 1956, -5293, -11591, -16855, -20657, -22745,
        -22620, -20532, -16662, -10851, -4492, 1694, 8810, 14599, 19120, 21982, 22926, 21725,
        18639, 13935, 7490, 848, -5868, -12075, -17601, -21123, -22811, -22406, -20004, -15865,
        -10350, -3379, 2819, 9329, 15030, 19721, 22278, 22902, 21537, 17958, 13021, 6953, -283,
        -6953, -13021, -17958, -21537, -22902, -22278, -19721, -15030, -9329, -2819, 3379, 10350,
        15865, 20004, 22520, 22811, 21123, 17601, 12075, 5868, -848, -7490, -13935, -18639, -21725,
        -22926, -21982, -19120, -14599, -8286, -1694, 4492, 10851, 16662, 20532, 22620, 22745,
        20657, 16855, 11591, 5320, -1976, -8549, -14380, -19275, -22061, -22933, -21814, -18473,
        -13710, -7756, -1130, 5595, 11834, 17045, 20778, 22780, 22572, 20405, 16068, 10602, 4215,
        -2538, -9587, -15242, -19575, -22209, -22884, -21439, -18132, -13252, -6684, 0, 6141,
        12787, 17781, 21231, 22839, 22344, 19864, 15660, 10097, 3099, -3658, -10097, -15660,
        -20140, -22465, -22839, -21011, -17418, -12314, -6141, 565, 7222, 13252, 18132, 21633,
        22916, 22209, 19575, 14816, 9070, 2538, -4769, -11099, -16467, -20405, -22665, -22707,
        -20778, -17045, -11346, -5045, 1130, 7756, 14158, 18802, 21814, 22936, 21900, 18963, 14380,
        8022, 1412, -5320, -11591, -17233, -20896, -22745, -22620, -20274, -16269, -10851, -3937,
        2257, 8810, 14599, 19427, 22137, 22926, 21725, 18304, 13482, 7490, 848, -6413, -12552,
        -17601, -21337, -22863, -22406, -20004, -15452, -9843, -3379, 2819, 9843, 15452, 19721,
        22278, 22863, 21337, 17958, 12552, 6413, -283, -6953, -13482, -18304, -21537, -22902,
        -22137, -19427, -15030, -9329, -2257, 3937, 10350, 16269, 20274, 22520, 22811, 20896,
        17233, 12075, 5868, -1412, -8022, -13935, -18639, -21900, -22936, -21982, -18802, -14158,
        -8286, -1694, 5045, 11346, 16662, 20532, 22707, 22665, 20657, 16855, 11099, 4769, -1976,
        -9070, -14816, -19275, -22061, -22916, -21633, -18473, -13710, -7222, -565, 5595, 11834,
        17418, 21011, 22780, 22465, 20140, 16068, 10602, 3658, -3099, -9587, -15242, -19864,
        -22344, -22884, -21439, -17781, -12787, -6684, 0, 6684, 12723, 17602, 21115, 22424, 21782,
        19265, 14706, 9201, 2959, -3474, -10016, -15099, -18825, -20984, -21063, -19322, -15930,
        -10763, -5060, 508, 6460, 12194, 16338, 19024, 20037, 19179, 16660, 12731, 7303, 1678,
        -4026, -9315, -14060, -17127, -18679, -18599, -16715, -13480, -9122, -4031, 1345, 6537,
        11099, 14893, 17011, 17635, 16728, 14143, 10504, 6006, 1050, -4335, -8859, -12557, -15121,
        -16392, -16070, -14365, -11162, -7229, -2730, 1554, 6376, 10196, 13081, 14795, 15191,
        14178, 11957, 8740, 4473, 180, -4060, -8179, -11190, -13188, -14017, -13546, -11892, -9240,
        -5836, -1657, 1969, 5688, 8852, 11359, 12610, 12753, 11676, 9641, 6812, 3448, -451, -3952,
        -7046, -9474, -11136, -11636, -11124, -9664, -7189, -4294, -1089, 2144, 5126, 7603, 9373,
        10344, 10287, 9345, 7621, 5067, 2299, -603, -3386, -5998, -7814, -8914, -9219, -8660,
        -7370, -5478, -2962, -426, 1876, 4162, 6167, 7413, 7987, 7859, 6972, 5544, 3681, 1377,
        -816, -2872, -4616, -6000, -6696, -6794, -6305, -5194, -3729, -1996, -153, 1636, 3218,
        4464, 5334, 5624, 5419, 4757, 3620, 2283, 809, -670, -2131, -3226, -3993, -4379, -4361,
        -3948, -3217, -2166, -1049, 0, 1075, 2064, 2734, 3124, 3220, 3013, 2558, 1915, 1156, 293,
        -466, -1119, -1655, -1948, -2042, -1946, -1671, -1285, -833, -365, 68, 429, 689, 835, 877,
        807, 665, 476, 289, 129, 20, -27, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    ];

    const N_SYL: usize = 8;
    const SYL_END: usize = N_SYL * 2 * CHIRP.len() / 96;

    #[shared]
    struct Shared {
        usb_dev: usb_device::device::UsbDevice<'static, bsp::hal::usb::UsbBus>,
        usb_audio: AudioClass<'static, UsbBus>,
    }

    #[local]
    struct Local {
        watchdog: bsp::hal::watchdog::Watchdog,
        alarm: bsp::hal::timer::Alarm0,
        led: bsp::hal::gpio::Pin<
            bsp::hal::gpio::pin::bank0::Gpio25,
            bsp::hal::gpio::Output<PushPull>,
        >,
        chirp: Cycle<ArrayChunks<'static, u8, 96>>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        defmt::info!("Starting Keyberon");

        unsafe {
            bsp::hal::sio::spinlock_reset();
        }

        let mut resets = c.device.RESETS;
        let mut watchdog = Watchdog::new(c.device.WATCHDOG);
        let sio = Sio::new(c.device.SIO);

        let clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let pins = bsp::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let led = pins.led.into_push_pull_output();

        let usb_bus = UsbBusAllocator::new(UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));

        unsafe {
            USB_BUS = Some(usb_bus);
        }

        let usb_audio = AudioClassBuilder::new()
            .input(
                StreamConfig::new_discrete(Format::S16le, 1, &[48000], TerminalType::InMicrophone)
                    .unwrap(),
            )
            .build(unsafe { USB_BUS.as_ref().unwrap() })
            .unwrap();

        let usb_dev =
            UsbDeviceBuilder::new(unsafe { USB_BUS.as_ref().unwrap() }, UsbVidPid(VID, PID))
                .max_packet_size_0(64)
                .manufacturer("XXX")
                .product("Audio port")
                .serial_number("42")
                .build();

        watchdog.start(MicrosDurationU32::millis(10));

        let mut timer = bsp::hal::Timer::new(c.device.TIMER, &mut resets);
        let mut alarm = timer.alarm_0().unwrap();
        let _ = alarm.schedule(SCAN_TIME_US);
        alarm.enable_interrupt();

        let chirp = unsafe { &*(&CHIRP as *const _ as *const [u8; 2 * CHIRP.len()]) }
            .array_chunks::<96>()
            .cycle();

        defmt::info!("Starting RTIC");

        (
            Shared { usb_dev, usb_audio },
            Local {
                watchdog,
                alarm,
                led,
                chirp,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = USBCTRL_IRQ,
           priority = 3,
           shared = [usb_dev, usb_audio],
           local = [input_alt_setting: u8 = 0])]
    fn usb_rx(c: usb_rx::Context) {
        let input_alt_setting = c.local.input_alt_setting;

        let usb = c.shared.usb_dev;
        let audio = c.shared.usb_audio;
        (usb, audio).lock(|usb, audio| {
            if usb.poll(&mut [audio]) {}
            if *input_alt_setting != audio.input_alt_setting().unwrap() {
                *input_alt_setting = audio.input_alt_setting().unwrap();
                defmt::info!("Alt. set. {}", input_alt_setting);
            }
        });
    }

    #[task(
        binds = TIMER_IRQ_0,
        priority = 1,
        shared = [usb_audio],
        local = [watchdog, alarm, led, chirp, n: usize = 0],
    )]
    fn scan_timer_irq(cx: scan_timer_irq::Context) {
        let alarm = cx.local.alarm;
        alarm.clear_interrupt();
        let _ = alarm.schedule(SCAN_TIME_US);

        cx.local.watchdog.feed();

        let chirp = cx.local.chirp;
        let mut audio = cx.shared.usb_audio;
        *cx.local.n += 1;

        audio.lock(|audio| {
            if *cx.local.n > SYL_END {
                cx.local.led.set_low().unwrap();
                let data: &[u8] = &[0; 96];
                audio.write(&data).ok();

                if *cx.local.n > SYL_END + 1000 {
                    *cx.local.n = 0;
                }
            } else {
                cx.local.led.set_high().unwrap();
                let data = chirp.next().unwrap();
                audio.write(data).ok();
            }
        });
    }

    #[idle(local = [])]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }
}
