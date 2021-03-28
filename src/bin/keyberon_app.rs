#![no_main]
#![no_std]
#![allow(unused_imports)]

use generic_array::GenericArray;
use hal::clocks::ExternalOscillator;
use hal::clocks::Internal;
use hal::clocks::LfOscStopped;
use keyberon::action::HoldTapConfig;
use keyberon::layout::CustomEvent;
use keyberon::layout::Event;
use keyboard as _;

//
// use stm32f4xx_hal::gpio::Alternate;
// use stm32f4xx_hal::spi::NoMiso;
// use stm32f4xx_hal::spi::NoSck;
// use stm32f4xx_hal::spi::Spi;
//

use core::convert::Infallible;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::timer::CountDown;
use generic_array::typenum::U8;
use keyberon::action::Action::{self, *};
use keyberon::action::{k, l, m};
use keyberon::debounce::Debouncer;
use keyberon::impl_heterogenous_array;
use keyberon::key_code::KeyCode::*;
use keyberon::key_code::{KbHidReport, KeyCode};
use keyberon::layout::Layout;
use keyberon::matrix::{Matrix, PressedKeys};
use nrf52840_hal::time::U32Ext;

use core::sync::atomic::{AtomicU32, Ordering};

use rtic::app;

use nrf52840_hal::{
    self as hal,
    clocks::{Clocks, LfOscConfiguration},
    gpio::{
        p0::Parts as P0Parts,
        p1::{Parts as P1Parts, P1_04},
        Input, Level, Output, Pin, PullUp, PushPull,
    },
    pac::{Interrupt, Peripherals, TIMER0, TIMER2, TWIM0, UARTE0, USBD},
    ppi::{Parts as PpiParts, Ppi0},
    spim::{Frequency, Pins as SpimPins, Spim, MODE_0},
    spis::{Mode, Pins as SpisPins, Spis, Transfer},
    timer::{Instance as TimerInstance, OneShot, Periodic, Timer},
    twim::{Frequency as TwimFrequency, Pins as TwimPins},
    uarte::{Baudrate, Parity, Pins},
    usbd::Usbd,
    Twim,
};

//
// use stm32f4xx_hal::gpio::{gpioa::*, gpiob::*, Input, Output, PullUp, PushPull};
// use stm32f4xx_hal::prelude::*;
// use stm32f4xx_hal::otg_fs::{USB, UsbBus, UsbBusType};
// use stm32f4xx_hal::{gpio, stm32 as pac, timer};
//

use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass as _;
use usb_device::device::UsbDeviceState;

use core::iter::Cloned;
use core::iter::Cycle;

//
// use stm32f4xx_hal::stm32::{self, DWT, SPI5};
//

use cortex_m::asm::delay;
use smart_leds::RGB;
use smart_leds::{colors, gamma, SmartLedsWrite, RGB8};
use ws2812_spi::{Ws2812, MODE};

type UsbClass<'a> = keyberon::Class<'static, Usbd<'a>, Leds>;
type UsbDevice<'a> = usb_device::device::UsbDevice<'static, Usbd<'a>>;

pub struct Leds {
    //     caps_lock: gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>,
}

impl keyberon::keyboard::Leds for Leds {
    /// Sets the num lock state.
    fn num_lock(&mut self, status: bool) {
        defmt::info!("num_lock: {:?}", status)
    }
    /// Sets the caps lock state.
    fn caps_lock(&mut self, status: bool) {
        defmt::info!("caps_lock: {:?}", status)
    }
    /// Sets the scroll lock state.
    fn scroll_lock(&mut self, status: bool) {
        defmt::info!("scroll_lock: {:?}", status)
    }
    /// Sets the compose state.
    fn compose(&mut self, status: bool) {
        defmt::info!("compose: {:?}", status)
    }
    /// Sets the kana state.
    fn kana(&mut self, status: bool) {
        defmt::info!("kana: {:?}", status)
    }
}

pub struct Cols(
    pub Pin<Input<PullUp>>,
    pub Pin<Input<PullUp>>,
    pub Pin<Input<PullUp>>,
    pub Pin<Input<PullUp>>,
    pub Pin<Input<PullUp>>,
    pub Pin<Input<PullUp>>,
    pub Pin<Input<PullUp>>,
    pub Pin<Input<PullUp>>,
);
impl_heterogenous_array! {
    Cols,
    dyn InputPin<Error = void::Void>,
    U8,
    [0, 1, 2, 3, 4, 5, 6, 7]
}

pub struct Rows(
    pub Pin<Output<PushPull>>,
    pub Pin<Output<PushPull>>,
    pub Pin<Output<PushPull>>,
    pub Pin<Output<PushPull>>,
    pub Pin<Output<PushPull>>,
    pub Pin<Output<PushPull>>,
    pub Pin<Output<PushPull>>,
    pub Pin<Output<PushPull>>,
);
impl_heterogenous_array! {
    Rows,
    dyn OutputPin<Error = void::Void>,
    U8,
    [0, 1, 2, 3, 4, 5, 6, 7]
}

const CUT: Action = m(&[LShift, Delete]);
const COPY: Action = m(&[LCtrl, Insert]);
const PASTE: Action = m(&[LShift, Insert]);
const L2_ENTER: Action = HoldTap {
    timeout: 160,
    hold: &l(2),
    tap: &k(Enter),
    config: HoldTapConfig::Default, // ??
    tap_hold_interval: 1000,        // ??
};
const L1_SP: Action = HoldTap {
    timeout: 200,
    hold: &l(1),
    tap: &k(Space),
    config: HoldTapConfig::Default, // ??
    tap_hold_interval: 1000,        // ??
};
const CSPACE: Action = m(&[LCtrl, Space]);
macro_rules! s {
    ($k:ident) => {
        m(&[LShift, $k])
    };
}
macro_rules! a {
    ($k:ident) => {
        m(&[RAlt, $k])
    };
}

static CTR_TCK: AtomicU32 = AtomicU32::new(0);
static CTR_RPT: AtomicU32 = AtomicU32::new(0);

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers = &[
    &[
        &[k(Grave),  k(Kb1),k(Kb2),k(Kb3),  k(Kb4),k(Kb5), k(Kb6),   k(Kb7),  k(Kb8), k(Kb9),  k(Kb0),   k(Minus)   ],
        &[k(Tab),     k(Q), k(W),  k(E),    k(R), k(T),    k(Y),     k(U),    k(I),   k(O),    k(P),     k(LBracket)],
        &[k(RBracket),k(A), k(S),  k(D),    k(F), k(G),    k(H),     k(J),    k(K),   k(L),    k(SColon),k(Quote)   ],
        &[k(Equal),   k(Z), k(X),  k(C),    k(V), k(B),    k(N),     k(M),    k(Comma),k(Dot), k(Slash), k(Bslash)  ],
        &[Trans,      Trans,k(LGui),k(LAlt),L1_SP,k(LCtrl),k(RShift),L2_ENTER,k(RAlt),k(BSpace),Trans,   Trans      ],
    ], &[
        &[k(F1),         k(F2),   k(F3),     k(F4),     k(F5),    k(F6),k(F7),      k(F8),  k(F9),    k(F10), k(F11),  k(F12)],
        &[Trans,         k(Pause),Trans,     k(PScreen),Trans,    Trans,Trans,      Trans,  k(Delete),Trans,  Trans,   Trans ],
        &[Trans,         Trans,   k(NumLock),k(Insert), k(Escape),Trans,k(CapsLock),k(Left),k(Down),  k(Up),  k(Right),Trans ],
        &[k(NonUsBslash),k(Undo), CUT,       COPY,      PASTE,    Trans,Trans,      k(Home),k(PgDown),k(PgUp),k(End),  Trans ],
        &[Trans,         Trans,   Trans,     Trans,     Trans,    Trans,Trans,      Trans,  Trans,    Trans,  Trans,   Trans ],
    ], &[
        &[Trans,    Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans    ],
        &[s!(Grave),s!(Kb1),s!(Kb2),s!(Kb3),s!(Kb4),s!(Kb5),s!(Kb6),s!(Kb7),s!(Kb8),s!(Kb9),s!(Kb0),s!(Minus)],
        &[ k(Grave), k(Kb1), k(Kb2), k(Kb3), k(Kb4), k(Kb5), k(Kb6), k(Kb7), k(Kb8), k(Kb9), k(Kb0), k(Minus)],
        &[a!(Grave),a!(Kb1),a!(Kb2),a!(Kb3),a!(Kb4),a!(Kb5),a!(Kb6),a!(Kb7),a!(Kb8),a!(Kb9),a!(Kb0),a!(Minus)],
        &[Trans,    Trans,  Trans,  Trans,  CSPACE, Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans    ],
    ], &[
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
        &[k(F1),k(F2),k(F3),k(F4),k(F5),k(F6),k(F7),k(F8),k(F9),k(F10),k(F11),k(F12)],
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
        &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
    ],
];

#[app(device = nrf52840_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static>,
        usb_class: UsbClass<'static>,
        matrix: Matrix<Cols, Rows>,
        debouncer: Debouncer<PressedKeys<U8, U8>>,
        layout: Layout,
        timer: Timer<TIMER0, OneShot>,
        // led: Ws2812<Spi<SPI5, (NoSck, NoMiso, PB8<Alternate<gpio::AF6>>)>>,
        data: [RGB8; 43],
        last: [[bool; 8]; 8],
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut CLOCKS: Option<Clocks<ExternalOscillator, Internal, LfOscStopped>> = None;
        static mut USB_BUS: Option<UsbBusAllocator<Usbd<'static>>> = None;

        defmt::info!("Hello, world!");

        let board = ctx.device;

        while !board
            .POWER
            .usbregstatus
            .read()
            .vbusdetect()
            .is_vbus_present()
        {}

        // wait until USB 3.3V supply is stable
        while !board
            .POWER
            .events_usbpwrrdy
            .read()
            .events_usbpwrrdy()
            .bit_is_clear()
        {}

        let clocks = Clocks::new(board.CLOCK);
        let clocks = clocks.enable_ext_hfosc();

        let mut timer = Timer::new(board.TIMER0);
        let usbd = board.USBD;
        let gpios_p0 = P0Parts::new(board.P0);
        let gpios_p1 = P1Parts::new(board.P1);

        // let spi = Spi::spi5(
        //     board.SPI5,
        //     (NoSck, NoMiso, gpiob.pb8.into_alternate_af6()),
        //     MODE,
        //     3_000_000.hz(),
        //     clocks
        // );
        // let led = Ws2812::new(spi);

        timer.enable_interrupt();
        timer.start(Timer::<TIMER0, OneShot>::TICKS_PER_SECOND / 1000);

        let leds = Leds {
            // caps_lock: led
        };

        //         let usb_dm = gpioa.pa11;
        //         let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        // let usb = USB {
        //     usb_global: board.OTG_FS_GLOBAL,
        //     usb_device: board.OTG_FS_DEVICE,
        //     usb_pwrclk: board.OTG_FS_PWRCLK,
        //     pin_dm: gpioa.pa11.into_alternate_af10(),
        //     pin_dp: gpioa.pa12.into_alternate_af10(),
        //     // hclk: clocks.hclk(),
        // };

        // *USB_BUS = Some(Usbd::new(usbd, &clocks));
        // let usb_bus = USB_BUS.as_ref().unwrap();
        usbd.intenset.write(|w| {
            w.endepin0()
                .set_bit()
                .endepout0()
                .set_bit()
                .endepin1()
                .set_bit()
                .endepout1()
                .set_bit()
                .endepin2()
                .set_bit()
                .endepout2()
                .set_bit()
                .endepin3()
                .set_bit()
                .endepout3()
                .set_bit()
                .endepin4()
                .set_bit()
                .endepout4()
                .set_bit()
                .endepin5()
                .set_bit()
                .endepout5()
                .set_bit()
                .endepin6()
                .set_bit()
                .endepout6()
                .set_bit()
                .endepin7()
                .set_bit()
                .endepout7()
                .set_bit()
                .endisoin()
                .set_bit()
                .endisoout()
                .set_bit()
                .ep0datadone()
                .set_bit()
                .ep0setup()
                .set_bit()
                .epdata()
                .set_bit()
                .sof()
                .set_bit()
                .started()
                .set_bit()
                .usbevent()
                .set_bit()
                .usbreset()
                .set_bit()
        });

        *CLOCKS = Some(clocks);
        let clocks = CLOCKS.as_ref().unwrap();
        *USB_BUS = Some(Usbd::new(usbd, &clocks));
        let usb_bus = USB_BUS.as_ref().unwrap();

        let usb_class = keyberon::new_class(usb_bus, leds);
        let usb_dev = keyberon::new_device(usb_bus);

        // let mut timer =
        //     timer::Timer::tim3(board.TIM3, 1u32.khz(), clocks);
        // timer.listen(timer::Event::TimeOut);

        let matrix = Matrix::new(
            Cols(
                gpios_p0.p0_19.into_pullup_input().degrade(),
                gpios_p0.p0_20.into_pullup_input().degrade(),
                gpios_p0.p0_21.into_pullup_input().degrade(),
                gpios_p0.p0_22.into_pullup_input().degrade(),
                gpios_p0.p0_23.into_pullup_input().degrade(),
                gpios_p0.p0_24.into_pullup_input().degrade(),
                gpios_p0.p0_25.into_pullup_input().degrade(),
                gpios_p0.p0_26.into_pullup_input().degrade(),
            ),
            Rows(
                gpios_p0.p0_05.into_push_pull_output(Level::Low).degrade(), // Start with first row low,
                gpios_p0.p0_06.into_push_pull_output(Level::High).degrade(), // and all other rows high
                gpios_p0.p0_07.into_push_pull_output(Level::High).degrade(),
                gpios_p0.p0_08.into_push_pull_output(Level::High).degrade(),
                gpios_p1.p1_09.into_push_pull_output(Level::High).degrade(),
                gpios_p1.p1_08.into_push_pull_output(Level::High).degrade(),
                gpios_p0.p0_12.into_push_pull_output(Level::High).degrade(),
                gpios_p0.p0_11.into_push_pull_output(Level::High).degrade(),
            ),
        );

        let c = 5;

        let de = Debouncer::new(PressedKeys::default(), PressedKeys::default(), c);

        init::LateResources {
            usb_dev,
            usb_class,
            timer,
            debouncer: de,
            matrix: matrix.unwrap(),
            layout: Layout::new(LAYERS),
            // led,
            data: [colors::BLACK; 43],
            last: [[false; 8]; 8],
        }
    }

    #[task(binds = USBD, priority = 2, resources = [usb_dev, usb_class])]
    fn usb_tx(mut c: usb_tx::Context) {
        static mut ctr: u32 = 0;
        static mut STATE: UsbDeviceState = UsbDeviceState::Default;

        let mut cleared = false;

        'once: loop {
            unsafe {
                let usbd = &*USBD::ptr();

                for i in 0..8 {
                    if usbd.events_endepin[i].read().bits() != 0 {
                        // defmt::info!("events_endepin[{:?}]", i);
                        cleared = true;
                        usbd.events_endepin[i].reset();
                        break 'once;
                    }

                    if usbd.events_endepout[i].read().bits() != 0 {
                        // defmt::info!("events_endepout[{:?}]", i);
                        cleared = true;
                        usbd.events_endepout[i].reset();
                        break 'once;
                    }
                }

                if usbd.events_endisoin.read().bits() != 0 {
                    // defmt::info!("events_endisoin");
                    cleared = true;
                    usbd.events_endisoin.reset();
                    break 'once;
                }

                if usbd.events_endisoout.read().bits() != 0 {
                    // defmt::info!("events_endisoout");
                    cleared = true;
                    usbd.events_endisoout.reset();
                    break 'once;
                }

                if usbd.events_ep0datadone.read().bits() != 0 {
                    // defmt::info!("events_ep0datadone");
                    cleared = true;
                    usbd.events_ep0datadone.reset();
                    break 'once;
                }

                if usbd.events_ep0setup.read().bits() != 0 {
                    // defmt::info!("events_ep0setup");
                    cleared = true;
                    usbd.events_ep0setup.reset();
                    break 'once;
                }

                if usbd.events_epdata.read().bits() != 0 {
                    // defmt::info!("events_epdata");
                    cleared = true;
                    usbd.events_epdata.reset();
                    break 'once;
                }

                if usbd.events_sof.read().bits() != 0 {
                    // defmt::info!("events_sof");
                    cleared = true;
                    usbd.events_sof.reset();
                    break 'once;
                }

                if usbd.events_started.read().bits() != 0 {
                    // defmt::info!("events_started");
                    cleared = true;
                    usbd.events_started.reset();
                    break 'once;
                }

                if usbd.events_usbevent.read().bits() != 0 {
                    // defmt::info!("events_usbevent");
                    cleared = true;
                    usbd.events_usbevent.reset();
                    break 'once;
                }

                if usbd.events_usbreset.read().bits() != 0 {
                    // defmt::info!("events_usbreset");
                    cleared = true;
                    usbd.events_usbreset.reset();
                    break 'once;
                }
            }
        }

        if !cleared {
            defmt::error!("BAD");
            keyboard::exit();
        }

        let new_state = c.resources.usb_dev.state();
        if new_state != *STATE {
            defmt::info!("State change!");
            *STATE = new_state;

            if new_state == UsbDeviceState::Configured {
                defmt::info!("Configured!");
            }
        }

        *ctr += 1;

        if (*ctr % 1_000) == 0 {
            defmt::info!("tick1k - usb");
        }

        usb_poll(&mut c.resources.usb_dev, &mut c.resources.usb_class);
    }

    #[task(binds = TIMER0, priority = 3, resources = [usb_class, matrix, debouncer, layout, timer, /* led, */ data, last])]
    fn tick(mut c: tick::Context) {
        static mut COLOOP: Option<Cycle<Cloned<core::slice::Iter<'static, RGB<u8>>>>> = None;
        static mut COLOOP2: Option<Cycle<Cloned<core::slice::Iter<'static, RGB<u8>>>>> = None;
        static mut roller: usize = 0;

        static mut ctr: u32 = 0;
        static mut ct_down: bool = false;

        static all_colors: &[RGB8; 7] = &[
            colors::RED,
            colors::ORANGE,
            colors::YELLOW,
            colors::GREEN,
            colors::BLUE,
            colors::INDIGO,
            colors::VIOLET,
        ];

        // cortex_m::peripheral::NVIC::pend(Interrupt::USBD);

        let count = CTR_TCK.fetch_add(1, Ordering::SeqCst);

        if (count % 1000) == 0 {
            defmt::info!("tick1k - timer");
        }

        c.resources.timer.event_compare_cc0().write(|w| w);
        c.resources
            .timer
            .start(Timer::<TIMER0, OneShot>::TICKS_PER_SECOND / 1000);

        let coloop = COLOOP.get_or_insert_with(|| all_colors.iter().cloned().cycle());
        let coloop2 = COLOOP2.get_or_insert_with(|| all_colors.iter().cloned().cycle());

        // c.resources.timer.clear_interrupt(timer::Event::TimeOut);

        for event in c
            .resources
            .debouncer
            .events(c.resources.matrix.get().unwrap())
        {
            let (is_low, x, y) = match event {
                Event::Press(x, y) => {
                    defmt::info!("Press: {:?}, {:?}", x, y);
                    (true, x as usize, y as usize)
                }
                Event::Release(x, y) => {
                    defmt::info!("Release: {:?}, {:?}", x, y);
                    (false, x as usize, y as usize)
                }
            };

            if is_low && !c.resources.last[x][y] {
                *c.resources.data.get_mut(row_col_to_pos(x, y)).unwrap() = coloop.next().unwrap();
                // defmt::info!("Pressed Row: {:?} Col: {:?}", rx, cx);
            }

            *c.resources.last.get_mut(x).unwrap().get_mut(y).unwrap() = is_low;

            c.resources.layout.event(event);
        }

        match c.resources.layout.tick() {
            CustomEvent::Press(p) => defmt::info!("press {:?}", p),
            CustomEvent::Release(r) => defmt::info!("release {:?}", r),
            _ => {}
        }
        let rct = CTR_RPT.fetch_add(1, Ordering::SeqCst);
        send_report(
            c.resources.layout.keycodes(),
            &mut c.resources.usb_class,
            &rct,
        );

        *ctr += 1;

        if *ctr >= 10 {
            *ctr = 0;

            for (rx, row) in c.resources.last.iter().enumerate() {
                for (cx, pix) in row.iter().enumerate() {
                    if !*pix {
                        let pix = c.resources.data.get_mut(row_col_to_pos(rx, cx)).unwrap();
                        pix.r = pix.r.saturating_sub(1);
                        pix.g = pix.g.saturating_sub(1);
                        pix.b = pix.b.saturating_sub(1);
                    }
                }
            }

            scanner(c.resources.data, roller, coloop2, ct_down);
            // fix_spi_errors();

            // c.resources.led.write(gamma(c.resources.data.iter().cloned())).ok();
        }
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
};


use keyberon::hid::HidClass;
fn send_report(
    iter: impl Iterator<Item = KeyCode>,
    usb_class: &mut UsbClass<'static>,
    ctr: &u32,
) {
    if (*ctr % 1000) == 0 {
        defmt::info!("tick1k - report!");
    }

    use rtic::Mutex;
    let report: KbHidReport = iter.collect();
    if usb_class.device_mut().set_keyboard_report(report.clone()) {
        let mut rpt = report.as_bytes();
        while !rpt.is_empty() {
            match usb_class.write(rpt) {
                Ok(0) => {
                    defmt::error!("zero!");
                    // continue;
                    break;
                }
                Ok(n) => {
                    defmt::info!("wrote {:?}", n);
                    rpt = &rpt[n..];
                }
                Err(_e) => {
                    panic!();
                }
            }
        }
    }
}

fn usb_poll(usb_dev: &mut UsbDevice, keyboard: &mut resources::usb_class<'_>) {
    use rtic::Mutex;
    keyboard.lock(|keyboard| {
        if usb_dev.poll(&mut [keyboard]) {
            keyboard.poll();
        }
    });

}

fn scanner(
    data: &mut [RGB8],
    idx: &mut usize,
    color: &mut Cycle<Cloned<core::slice::Iter<'_, RGB<u8>>>>,
    go_down: &mut bool,
) {
    let (keys, bar) = data.split_at_mut(24);
    *idx = idx.wrapping_add(1);
    let ctr = *idx % 8;
    let idxn = *idx / 8;

    let dlen = bar.len();

    let idxm = if *go_down {
        (dlen - 1) - (idxn % dlen)
    } else {
        idxn % dlen
    };

    for (i, led) in bar.iter_mut().enumerate() {
        if (idxm == i) && (ctr == 0) {
            *led = color.next().unwrap();

            if *go_down && (idxm == 0) {
                *go_down = false;
                // for key in keys.iter_mut() {
                //     *key = *led;
                // }
            } else if !*go_down && idxm == (dlen - 1) {
                *go_down = true;
                // for key in keys.iter_mut() {
                //     *key = *led;
                // }
            }

            led.r = led.r / 2;
            led.g = led.g / 2;
            led.b = led.b / 2;
        } else {
            led.r = led.r.saturating_sub(4);
            led.g = led.g.saturating_sub(4);
            led.b = led.b.saturating_sub(4);
        }
    }
}

fn row_col_to_pos(row: usize, col: usize) -> usize {
    match (row, col) {
        (0, 0) => 0,
        (0, 1) => 1,
        (0, 2) => 2,
        (0, 3) => 3,
        (1, 0) => 7,
        (1, 1) => 6,
        (1, 2) => 5,
        (1, 3) => 4,
        (2, 0) => 8,
        (2, 1) => 9,
        (2, 2) => 10,
        (2, 3) => 11,
        (3, 0) => 15,
        (3, 1) => 14,
        (3, 2) => 13,
        (3, 3) => 12,
        (4, 0) => 16,
        (4, 1) => 17,
        (4, 2) => 18,
        (4, 3) => 19,
        (5, 0) => 23,
        (5, 1) => 22,
        (5, 2) => 21,
        (5, 3) => 20,
        _ => 24,
    }
}

// Fix Errors
// fn fix_spi_errors() {
//     unsafe {
//         let spi = &*SPI5::ptr();

//         // Read from DR register to clear OVR
//         let _ = spi.dr.read();

//         // Read from SR to clear CRCERR and OVR
//         spi.sr.modify(|r, w| {
//             let _ = r.txe().bit_is_set();

//             if r.crcerr().bit_is_set() {
//                 w.crcerr().clear_bit();
//             }
//             w
//         });

//         // Write to CR1 to clear MODF
//         spi.cr1.modify(|_r, w| w);
//     }
// }