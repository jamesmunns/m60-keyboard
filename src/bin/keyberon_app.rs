#![no_main]
#![no_std]

use core::{
    iter::{Cloned, Cycle},
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

use embedded_hal::{
    blocking::i2c::{Write, WriteRead},
    digital::v2::{InputPin, OutputPin},
    timer::CountDown,
};
use generic_array::typenum::U8;
use keyberon::{
    action::{
        k,
        Action::{self, *},
    },
    debounce::Debouncer,
    hid::HidClass,
    impl_heterogenous_array,
    key_code::{KbHidReport, KeyCode, KeyCode::*},
    layout::{CustomEvent, Event, Layout},
    matrix::{Matrix, PressedKeys},
};
use keyboard as _;
use nrf52840_hal::{
    clocks::{Clocks, ExternalOscillator, Internal, LfOscStopped},
    gpio::{
        p0::Parts as P0Parts,
        p1::{Parts as P1Parts, P1_04},
        Input, Level, Output, Pin, PullUp, PushPull,
    },
    pac::{TIMER0, TIMER1, TWIM0},
    timer::{Instance as TimerInstance, Periodic, Timer},
    twim::{Frequency as TwimFrequency, Pins as TwimPins},
    usbd::Usbd,
    Twim,
};
use rtic::app;
use smart_leds::{colors, gamma, RGB, RGB8};
use usb_device::{bus::UsbBusAllocator, class::UsbClass as _, device::UsbDeviceState};

static ALL_COLORS: &[RGB8; 7] = &[
    colors::RED,
    colors::ORANGE,
    colors::YELLOW,
    colors::GREEN,
    colors::BLUE,
    colors::INDIGO,
    colors::VIOLET,
];

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
];

use bbqueue::{
    consts as bbconsts,
    framed::{FrameConsumer, FrameProducer},
    BBBuffer, ConstBBBuffer,
};

static REPORT_QUEUE: BBBuffer<bbconsts::U2048> = BBBuffer(ConstBBBuffer::new());

#[rustfmt::skip]
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

#[rustfmt::skip]
static NEEDS_COLOR: [[AtomicBool; 8]; 8] = [
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
        timer1.enable_interrupt();
        timer1.start(Timer::<TIMER1, Periodic>::TICKS_PER_SECOND / 30);

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

        let de = Debouncer::new(PressedKeys::default(), PressedKeys::default(), c);

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

    #[task(binds = TIMER1, priority = 1, resources = [data, key_leds, timer1])]
    fn led_tick(c: led_tick::Context) {
        static mut COLOOP: Option<Cycle<Cloned<core::slice::Iter<'static, RGB<u8>>>>> = None;

        c.resources.timer1.event_compare_cc0().write(|w| w);

        let coloop = COLOOP.get_or_insert_with(|| ALL_COLORS.iter().cloned().cycle());

        for (rx, row) in NEEDS_COLOR.iter().enumerate() {
            for (cx, pix_bool) in row.iter().enumerate() {
                if pix_bool.swap(false, Ordering::AcqRel) {
                    defmt::info!("Coloring: {:?}, {:?}", rx, cx);
                    let pix_col = coloop.next().unwrap();
                    let idx = (rx * 8) + cx;
                    *c.resources.data.get_mut(idx).unwrap() = pix_col;
                    c.resources
                        .key_leds
                        .update_pixel(idx as u8, pix_col)
                        .unwrap();
                }
            }
        }

        for (rx, row) in LAST.iter().enumerate() {
            for (cx, pix_bool) in row.iter().enumerate() {
                if !pix_bool.load(Ordering::Acquire) {
                    let idx = (rx * 8) + cx;
                    let pix = c.resources.data.get_mut(idx).unwrap();
                    let mut pix_clone = pix.clone();
                    pix_clone.r = pix.r.saturating_sub(10);
                    pix_clone.g = pix.g.saturating_sub(10);
                    pix_clone.b = pix.b.saturating_sub(10);

                    if pix_clone != *pix {
                        c.resources.key_leds.update_pixel(idx as u8, *pix).unwrap();
                        *pix = pix_clone;
                    }

                }
            }
        }
    }

    #[task(binds = TIMER0, priority = 1, resources = [usb_class, matrix, debouncer, layout, timer, data, rpt_prod])]
    fn tick(mut c: tick::Context) {
        let count = CTR_TCK.fetch_add(1, Ordering::SeqCst);

        if (count % 1000) == 0 {
            defmt::info!("tick1k - timer");
        }

        c.resources.timer.event_compare_cc0().write(|w| w);

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
                NEEDS_COLOR[x][y].store(true, Ordering::Release);
            }

            LAST[x][y].store(is_low, Ordering::Release);

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
            &mut c.resources.rpt_prod,
        );
    }

    #[idle(resources = [usb_dev, usb_class, rpt_cons])]
    fn idle(mut c: idle::Context) -> ! {
        let mut state: UsbDeviceState = UsbDeviceState::Default;
        let mut ctr: u32 = 0;
        let mut skip_flag = false;

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
                            if !skip_flag {
                                skip_flag = true;
                                defmt::warn!("skipping!");
                            }
                        }
                        Ok(n) => {
                            if skip_flag {
                                defmt::warn!("cleared skip!");
                                skip_flag = false;
                            }
                            if n == rgr.len() {
                                defmt::info!("wrote {:?}", n);
                            } else {
                                defmt::error!("wrote {:?} of {:?}", n, rgr.len());
                                keyboard::exit();
                            }
                            rgr.release();
                        }
                        Err(_e) => {
                            panic!();
                        }
                    }

                }
            });
        }
    }
};

fn send_report<'a>(
    iter: impl Iterator<Item = KeyCode>,
    usb_class: &mut HidClass<'static, Usbd<'static>, keyberon::keyboard::Keyboard<Leds>>,
    ctr: &u32,
    queue: &'a mut FrameProducer<'static, bbconsts::U2048>,
) -> Option<&'a mut [u8]> {
    if (*ctr % 1000) == 0 {
        defmt::info!("tick1k - report!");
    }

    let report: KbHidReport = iter.collect();

    if usb_class.device_mut().set_keyboard_report(report.clone()) {
        let rpt = report.as_bytes();
        if let Ok(mut wgr) = queue.grant(rpt.len()) {
            wgr.copy_from_slice(rpt);
            wgr.commit(rpt.len());
        } else {
            panic!("Overfull queue!");
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
