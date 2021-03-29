#![no_main]
#![no_std]

#![allow(unused_imports)]

use hal::clocks::ExternalOscillator;
use hal::clocks::Internal;
use hal::clocks::LfOscStopped;
use keyberon::action::HoldTapConfig;
use keyboard as _;
use generic_array::GenericArray;
use keyberon::layout::CustomEvent;
use keyberon::layout::Event;

use core::convert::Infallible;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::timer::CountDown;
use embedded_hal::blocking::i2c::{WriteRead, Write};
use generic_array::typenum::U8;
use keyberon::action::Action::{self, *};
use keyberon::action::{k, l, m};
use keyberon::debounce::Debouncer;
use keyberon::impl_heterogenous_array;
use keyberon::key_code::KeyCode::*;
use keyberon::key_code::{KbHidReport, KeyCode};
use keyberon::layout::Layout;
use keyberon::matrix::{Matrix, PressedKeys};
use nrf52840_hal::{
    time::U32Ext,
};

use core::sync::atomic::{Ordering, AtomicU32, AtomicBool};

use rtic::app;

use nrf52840_hal::{
    self as hal,
    clocks::{Clocks, LfOscConfiguration},
    gpio::{
        p0::Parts as P0Parts,
        p1::{Parts as P1Parts, P1_04},
        Input, Level, Output, Pin, PullUp, PushPull,
    },
    pac::{Peripherals, TIMER0, TIMER1, TIMER2, TWIM0, UARTE0, USBD},
    ppi::{Parts as PpiParts, Ppi0},
    spim::{Frequency, Pins as SpimPins, Spim, MODE_0},
    spis::{Mode, Pins as SpisPins, Spis, Transfer},
    timer::{Instance as TimerInstance, Periodic, Timer, OneShot},
    twim::{Frequency as TwimFrequency, Pins as TwimPins},
    uarte::{Baudrate, Parity, Pins},
    usbd::Usbd,
    Twim,
};

use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass as _;
use usb_device::device::UsbDeviceState;

use core::iter::Cloned;
use core::iter::Cycle;

use smart_leds::RGB;
use ws2812_spi::{Ws2812, MODE};
use cortex_m::asm::delay;
use smart_leds::{RGB8, SmartLedsWrite, colors, gamma};

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

/*
const CUT: Action = m(&[LShift, Delete]);
const COPY: Action = m(&[LCtrl, Insert]);
const PASTE: Action = m(&[LShift, Insert]);
const L2_ENTER: Action = HoldTap {
    timeout: 160,
    hold: &l(2),
    tap: &k(Enter),
    config: HoldTapConfig::Default, // ??
    tap_hold_interval: 1000, // ??
};
const L1_SP: Action = HoldTap {
    timeout: 200,
    hold: &l(1),
    tap: &k(Space),
    config: HoldTapConfig::Default, // ??
    tap_hold_interval: 1000, // ??
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
*/

static CTR_TCK: AtomicU32 = AtomicU32::new(0);
static CTR_RPT: AtomicU32 = AtomicU32::new(0);

// Constant to denote something I should fix (vs Trans, which is just a nothing)
const TODO: Action = Action::Trans;

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers = &[
    &[
        &[k(Escape),   k(Kb1),     k(Kb2),   k(Kb3),    k(Kb4),      k(Kb5),     k(Kb6),     k(Kb7)     ],
        &[k(Kb8),      k(Kb9),     k(Kb0),   k(Minus),  k(Equal),    k(BSpace),  k(Bslash),  k(RBracket)],
        &[k(LBracket), k(P),       k(O),     k(I),      k(U),        k(Y),       k(T),       k(R)       ],
        &[k(E),        k(W),       k(Q),     k(Tab),    k(CapsLock), k(A),       k(S),       k(D)       ],
        &[k(F),        k(G),       k(H),     k(J),      k(K),        k(L),       k(SColon),  k(Quote)   ],
        &[k(Enter),    k(RShift),  k(Slash), k(Dot),    k(Comma),    k(M),       k(N),       k(B)       ],
        &[k(V),        k(C),       k(X),     k(Z),      k(LShift),   k(LCtrl),   k(LGui),    k(LAlt)    ],
        &[k(Space),    k(RAlt),    k(Menu),  TODO,      k(RCtrl),    Trans,      Trans,      Trans      ],
    ]
    // &[
    //     &[k(Grave),  k(Kb1),k(Kb2),k(Kb3),  k(Kb4),k(Kb5), k(Kb6),   k(Kb7),  k(Kb8), k(Kb9),  k(Kb0),   k(Minus)   ],
    //     &[k(Tab),     k(Q), k(W),  k(E),    k(R), k(T),    k(Y),     k(U),    k(I),   k(O),    k(P),     k(LBracket)],
    //     &[k(RBracket),k(A), k(S),  k(D),    k(F), k(G),    k(H),     k(J),    k(K),   k(L),    k(SColon),k(Quote)   ],
    //     &[k(Equal),   k(Z), k(X),  k(C),    k(V), k(B),    k(N),     k(M),    k(Comma),k(Dot), k(Slash), k(Bslash)  ],
    //     &[Trans,      Trans,k(LGui),k(LAlt),L1_SP,k(LCtrl),k(RShift),L2_ENTER,k(RAlt),k(BSpace),Trans,   Trans      ],
    // ], &[
    //     &[k(F1),         k(F2),   k(F3),     k(F4),     k(F5),    k(F6),k(F7),      k(F8),  k(F9),    k(F10), k(F11),  k(F12)],
    //     &[Trans,         k(Pause),Trans,     k(PScreen),Trans,    Trans,Trans,      Trans,  k(Delete),Trans,  Trans,   Trans ],
    //     &[Trans,         Trans,   k(NumLock),k(Insert), k(Escape),Trans,k(CapsLock),k(Left),k(Down),  k(Up),  k(Right),Trans ],
    //     &[k(NonUsBslash),k(Undo), CUT,       COPY,      PASTE,    Trans,Trans,      k(Home),k(PgDown),k(PgUp),k(End),  Trans ],
    //     &[Trans,         Trans,   Trans,     Trans,     Trans,    Trans,Trans,      Trans,  Trans,    Trans,  Trans,   Trans ],
    // ], &[
    //     &[Trans,    Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans    ],
    //     &[s!(Grave),s!(Kb1),s!(Kb2),s!(Kb3),s!(Kb4),s!(Kb5),s!(Kb6),s!(Kb7),s!(Kb8),s!(Kb9),s!(Kb0),s!(Minus)],
    //     &[ k(Grave), k(Kb1), k(Kb2), k(Kb3), k(Kb4), k(Kb5), k(Kb6), k(Kb7), k(Kb8), k(Kb9), k(Kb0), k(Minus)],
    //     &[a!(Grave),a!(Kb1),a!(Kb2),a!(Kb3),a!(Kb4),a!(Kb5),a!(Kb6),a!(Kb7),a!(Kb8),a!(Kb9),a!(Kb0),a!(Minus)],
    //     &[Trans,    Trans,  Trans,  Trans,  CSPACE, Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans    ],
    // ], &[
    //     &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
    //     &[k(F1),k(F2),k(F3),k(F4),k(F5),k(F6),k(F7),k(F8),k(F9),k(F10),k(F11),k(F12)],
    //     &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
    //     &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
    //     &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
    // ],
];


use bbqueue::{
    BBBuffer, ConstBBBuffer,
    consts as bbconsts,
    framed::{
        FrameProducer,
        FrameConsumer,
    },
};

static REPORT_QUEUE: BBBuffer<bbconsts::U2048> = BBBuffer(ConstBBBuffer::new());
static LAST: [[AtomicBool; 8]; 8] = [
    [AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), ],
    [AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), ],
    [AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), ],
    [AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), ],
    [AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), ],
    [AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), ],
    [AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), ],
    [AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), AtomicBool::new(false), ],
];

#[app(device = nrf52840_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice<'static>,
        usb_class: UsbClass<'static>,
        matrix: Matrix<Cols, Rows>,
        debouncer: Debouncer<PressedKeys<U8, U8>>,
        layout: Layout,
        timer: Timer<TIMER0, Periodic>,
        timer1: Timer<TIMER1, Periodic>,
        // led: Ws2812<Spi<SPI5, (NoSck, NoMiso, PB8<Alternate<gpio::AF6>>)>>,
        key_leds: IS31FL3733,

        data: [RGB8; 8 * 8],

        rpt_prod: FrameProducer<'static, bbconsts::U2048>,
        rpt_cons: FrameConsumer<'static, bbconsts::U2048>,
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

        let mut timer = Timer::periodic(board.TIMER0);
        let mut timer1 = Timer::periodic(board.TIMER1);
        let usbd = board.USBD;
        let gpios_p0 = P0Parts::new(board.P0);
        let gpios_p1 = P1Parts::new(board.P1);

        let twim = Twim::new(
            board.TWIM0,
            TwimPins {
                scl: gpios_p1.p1_06.into_floating_input().degrade(),
                sda: gpios_p1.p1_05.into_floating_input().degrade(),
            },
            TwimFrequency::K400, // ?
        );
        let mut key_leds = IS31FL3733::new(twim, gpios_p1.p1_04.into_push_pull_output(Level::Low));

        key_leds.reset().unwrap();
        cortex_m::asm::delay(64_000_000 / 10);
        key_leds.setup().unwrap();
        cortex_m::asm::delay(64_000_000 / 5);

        timer.enable_interrupt();
        timer.start(Timer::<TIMER0, Periodic>::TICKS_PER_SECOND / 1000);
        // timer1.enable_interrupt();
        // timer1.start(Timer::<TIMER1, Periodic>::TICKS_PER_SECOND / 10000);

        let leds = Leds {
            // caps_lock: led
        };

        *CLOCKS = Some(clocks);
        let clocks = CLOCKS.as_ref().unwrap();
        *USB_BUS = Some(Usbd::new(usbd, &clocks));
        let usb_bus = USB_BUS.as_ref().unwrap();

        let usb_class = keyberon::new_class(usb_bus, leds);
        let usb_dev = keyberon::new_device(usb_bus);

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
                gpios_p0.p0_05.into_push_pull_output(Level::High).degrade(),
                gpios_p0.p0_06.into_push_pull_output(Level::High).degrade(),
                gpios_p0.p0_07.into_push_pull_output(Level::High).degrade(),
                gpios_p0.p0_08.into_push_pull_output(Level::High).degrade(),
                gpios_p1.p1_09.into_push_pull_output(Level::High).degrade(),
                gpios_p1.p1_08.into_push_pull_output(Level::High).degrade(),
                gpios_p0.p0_12.into_push_pull_output(Level::High).degrade(),
                gpios_p0.p0_11.into_push_pull_output(Level::High).degrade(),
            ),
        );

        let c = 5;

        let de = Debouncer::new(
            PressedKeys::default(),
            PressedKeys::default(),
            c,
        );

        let (rpt_prod, rpt_cons) = REPORT_QUEUE.try_split_framed().unwrap();

        init::LateResources {
            usb_dev,
            usb_class,
            timer,
            timer1,
            debouncer: de,
            matrix: matrix.unwrap(),
            layout: Layout::new(LAYERS),
            // led,
            data: [colors::BLACK; 8 * 8],
            rpt_prod,
            rpt_cons,
            key_leds,
        }
    }

    // #[task(binds = TIMER1, priority = 1, resources = [timer1])]
    // fn usb_tick(mut c: usb_tick::Context) {
    //     c.resources.timer1.event_compare_cc0().write(|w| w);
    //     cortex_m::peripheral::NVIC::pend(hal::pac::Interrupt::USBD);
    // }


    #[task(binds = TIMER0, priority = 1, resources = [usb_class, matrix, debouncer, layout, timer, key_leds, data, rpt_prod])]
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

        let count = CTR_TCK.fetch_add(1, Ordering::SeqCst);

        if (count % 1000) == 0 {
            defmt::info!("tick1k - timer");
        }

        c.resources.timer.event_compare_cc0().write(|w| w);

        let coloop = COLOOP.get_or_insert_with(|| all_colors.iter().cloned().cycle());
        let coloop2 = COLOOP2.get_or_insert_with(|| all_colors.iter().cloned().cycle());

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

            if is_low && !LAST[x][y].load(Ordering::Acquire) {
                defmt::info!("Coloring: {:?}, {:?}", x, y);
                let pix_col = coloop.next().unwrap();
                let idx = (x * 8) + y;
                *c.resources.data.get_mut(idx).unwrap() = pix_col;
                c.resources
                    .key_leds
                    .update_pixel(idx as u8, pix_col)
                    .unwrap();
            }

            LAST[x][y].store(is_low, Ordering::Release);

            c.resources.layout.event(event);
        }

        match c.resources.layout.tick() {
            CustomEvent::Press(p) => defmt::info!("press {:?}", p),
            CustomEvent::Release(r) => defmt::info!("release {:?}", r),
            _ => {},
        }
        let rct = CTR_RPT.fetch_add(1, Ordering::SeqCst);
        use rtic::Mutex;

        send_report(
            c.resources.layout.keycodes(),
            &mut c.resources.usb_class,
            &rct,
            &mut c.resources.rpt_prod,
        );


        *ctr += 1;

        if *ctr >= 10 {
            *ctr = 0;

            for (rx, row) in LAST.iter().enumerate() {
                for (cx, pix_bool) in row.iter().enumerate() {
                    if !pix_bool.load(Ordering::Acquire) {
                        let idx = (rx * 8) + cx;
                        let pix = c.resources.data.get_mut(idx).unwrap();
                        pix.r = pix.r.saturating_sub(10);
                        pix.g = pix.g.saturating_sub(10);
                        pix.b = pix.b.saturating_sub(10);

                        c.resources
                            .key_leds
                            .update_pixel(idx as u8, *pix)
                            .unwrap();
                    }
                }
            }
        }
    }

    #[idle(resources = [usb_dev, usb_class, rpt_cons])]
    fn idle(mut c: idle::Context) -> ! {
        let mut state: UsbDeviceState = UsbDeviceState::Default;
        let mut ctr: u32 = 0;

        loop {
            let new_state = c.resources.usb_dev.state();
            if new_state != state {
                defmt::info!("State change!");
                state = new_state;

                if new_state == UsbDeviceState::Configured {
                    defmt::info!("Configured!");
                }
            }

            ctr = ctr.wrapping_add(1);

            if (ctr % 1_000_000) == 0 {
                defmt::info!("tick1m - usb");
            }

            use rtic::Mutex;
            let usb_d = &mut c.resources.usb_dev;
            let rpt_c = &mut c.resources.rpt_cons;
            c.resources.usb_class.lock(|usb_cl| {
                usb_poll(usb_d, usb_cl);

                if let Some(rgr) = rpt_c.read() {
                    match usb_cl.write(&rgr) {
                        Ok(0) => {
                            // We do nothing, letting the grant drop so we can try again later
                        }
                        Ok(n) => {
                            if n == rgr.len() {
                                defmt::info!("wrote {:?}", n);
                            } else {
                                defmt::error!("wrote {:?} of {:?}", n, rgr.len());
                                keyboard::exit();
                            }
                        }
                        Err(_e) => {
                            panic!();
                        }
                    }
                    rgr.release();
                }
            });
        }
    }
};

use keyberon::hid::HidClass;

fn send_report<'a>(
    iter: impl Iterator<Item = KeyCode>,
    usb_class: &mut HidClass<'static, Usbd<'static>, keyberon::keyboard::Keyboard<Leds>>,
    ctr: &u32,
    queue: &'a mut FrameProducer<'static, bbconsts::U2048>,
) -> Option<&'a mut [u8]> {

    if (*ctr % 1000) == 0 {
        defmt::info!("tick1k - report!");
    }

    use rtic::Mutex;
    let report: KbHidReport = iter.collect();

    if usb_class.device_mut().set_keyboard_report(report.clone()) {
        let rpt = report.as_bytes();
        if let Ok(mut wgr) = queue.grant(rpt.len()) {
            wgr.copy_from_slice(rpt);
            wgr.commit(rpt.len());
        }
    }

    None
}

fn usb_poll(usb_dev: &mut UsbDevice, keyboard: &mut UsbClass) {
    if usb_dev.poll(&mut [keyboard]) {
        keyboard.poll();
    }
}

pub struct IS31FL3733 {
    i2c: Twim<TWIM0>,
    power: P1_04<Output<PushPull>>,
}

impl IS31FL3733 {
    fn new(i2c: Twim<TWIM0>, mut power: P1_04<Output<PushPull>>) -> Self {
        power.set_high().ok();
        Self { i2c, power }
    }

    fn write(&mut self, buf: &[u8]) -> Result<(), ()> {
        <Twim<TWIM0> as Write>::write(&mut self.i2c, 0x50, buf).map_err(drop)
    }

    fn page(&mut self, page: u8) -> Result<(), ()> {
        self.write(&[0xFE, 0xC5])?;
        self.write(&[0xFD, page])
    }

    fn reset(&mut self) -> Result<(), ()> {
        self.page(3)?;
        self.read(0x11)
    }

    fn read(&mut self, reg_id: u8) -> Result<(), ()> {
        let inbuf = [reg_id];
        let mut outbuf = [0u8; 1];
        self.i2c.write_read(0x50, &inbuf, &mut outbuf).map_err(drop)
    }

    fn set_brightness(&mut self, brightness: u8) -> Result<(), ()> {
        self.page(3)?;
        self.write(&[1, brightness])
    }

    fn setup(&mut self) -> Result<(), ()> {
        self.page(3)?;
        self.write(&[2, (2 << 5) | (0 << 1)])?;
        self.write(&[3, (2 << 5) | (3 << 1)])?;
        self.write(&[4, (0 << 4)])?;

        self.write(&[6, (2 << 5) | (0 << 1)])?;
        self.write(&[7, (2 << 5) | (2 << 1)])?;
        self.write(&[8, (0 << 4)])?;

        self.write(&[0xA, (1 << 5) | (0 << 1)])?;
        self.write(&[0xB, (1 << 5) | (1 << 1)])?;
        self.write(&[0xC, (0 << 4)])?;

        self.write(&[0, 1])?;
        self.write(&[0, 3])?;
        self.write(&[0xE, 0])?;

        self.page(0)?;
        let mut buf = [0xFF; 0x18 + 1];
        buf[0] = 0x00;
        self.write(&buf)?;

        self.set_brightness(255)
    }

    fn update_pixel(&mut self, i: u8, pix: RGB8) -> Result<(), ()> {
        let row = i >> 4; // # i // 16
        let col = i & 15; // # i % 16
        self.page(1)?;
        self.write(&[row * 48 + col, pix.g])?;
        self.write(&[row * 48 + 16 + col, pix.r])?;
        self.write(&[row * 48 + 32 + col, pix.b])?;
        Ok(())
    }
}
