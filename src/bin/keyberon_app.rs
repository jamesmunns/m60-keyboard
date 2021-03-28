#![no_main]
#![no_std]

use brick_keypad as _;
use generic_array::GenericArray;
use keyberon::layout::CustomEvent;
use keyberon::layout::Event;
use stm32f4xx_hal::gpio::Alternate;
use stm32f4xx_hal::spi::NoMiso;
use stm32f4xx_hal::spi::NoSck;
use stm32f4xx_hal::spi::Spi;

use core::convert::Infallible;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use generic_array::typenum::{U4, U6};
use keyberon::action::Action::{self, *};
use keyberon::action::{k, l, m};
use keyberon::debounce::Debouncer;
use keyberon::impl_heterogenous_array;
use keyberon::key_code::KeyCode::*;
use keyberon::key_code::{KbHidReport, KeyCode};
use keyberon::layout::Layout;
use keyberon::matrix::{Matrix, PressedKeys};

use rtic::app;

use stm32f4xx_hal::gpio::{gpioa::*, gpiob::*, Input, Output, PullUp, PushPull};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::otg_fs::{USB, UsbBus, UsbBusType};
use stm32f4xx_hal::{gpio, stm32 as pac, timer};

use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass as _;

use core::iter::Cloned;
use core::iter::Cycle;

use smart_leds::RGB;
use stm32f4xx_hal::stm32::{self, DWT, SPI5};
use ws2812_spi::{Ws2812, MODE};
use cortex_m::asm::delay;
use smart_leds::{RGB8, SmartLedsWrite, colors, gamma};

type UsbClass = keyberon::Class<'static, UsbBusType, Leds>;
type UsbDevice = usb_device::device::UsbDevice<'static, UsbBusType>;

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
    pub PA6<Input<PullUp>>,
    pub PA7<Input<PullUp>>,
    pub PA8<Input<PullUp>>,
    pub PA9<Input<PullUp>>,
);
impl_heterogenous_array! {
    Cols,
    dyn InputPin<Error = Infallible>,
    U4,
    [0, 1, 2, 3]
}

pub struct Rows(
    pub PA0<Output<PushPull>>,
    pub PA1<Output<PushPull>>,
    pub PA2<Output<PushPull>>,
    pub PA3<Output<PushPull>>,
    pub PA4<Output<PushPull>>,
    pub PA5<Output<PushPull>>,
);
impl_heterogenous_array! {
    Rows,
    dyn OutputPin<Error = Infallible>,
    U6,
    [0, 1, 2, 3, 4, 5]
}

// const CUT: Action = m(&[LShift, Delete]);
// const COPY: Action = m(&[LCtrl, Insert]);
// const PASTE: Action = m(&[LShift, Insert]);
// const L2_ENTER: Action = HoldTap {
//     timeout: 160,
//     hold: &l(2),
//     tap: &k(Enter),
// };
// const L1_SP: Action = HoldTap {
//     timeout: 200,
//     hold: &l(1),
//     tap: &k(Space),
// };
// const CSPACE: Action = m(&[LCtrl, Space]);
// macro_rules! s {
//     ($k:ident) => {
//         m(&[LShift, $k])
//     };
// }
// macro_rules! a {
//     ($k:ident) => {
//         m(&[RAlt, $k])
//     };
// }

#[rustfmt::skip]
pub static LAYERS: keyberon::layout::Layers = &[
    &[
        &[Trans,        Trans,      Trans,          k(NumLock), ],
        &[Trans,        k(KpSlash), k(KpAsterisk),  k(Bslash),  ],
        &[k(Kp7),       k(Kp8),     k(Kp9),         k(KpMinus), ],
        &[k(Kp4),       k(Kp5),     k(Kp6),         k(KpPlus),  ],
        &[k(Kp1),       k(Kp2),     k(Kp3),         k(KpEnter), ],
        // TODO: double zero?
        // ~~~~-----------vvv
        &[k(Kp0),       k(Kp0),     k(KpDot),       k(KpEnter), ],
    ],
    // &[
    //     &[k(Grave),  k(Kb1),k(Kb2),k(Kb3),  k(Kb4),k(Kb5), k(Kb6),   k(Kb7),  k(Kb8), k(Kb9),  k(Kb0),   k(Minus)   ],
    //     &[k(Tab),     k(Q), k(W),  k(E),    k(R), k(T),    k(Y),     k(U),    k(I),   k(O),    k(P),     k(LBracket)],
    //     &[k(RBracket),k(A), k(S),  k(D),    k(F), k(G),    k(H),     k(J),    k(K),   k(L),    k(SColon),k(Quote)   ],
    //     &[k(Equal),   k(Z), k(X),  k(C),    k(V), k(B),    k(N),     k(M),    k(Comma),k(Dot), k(Slash), k(Bslash)  ],
    //     &[Trans,      Trans,k(LGui),k(LAlt),L1_SP,k(LCtrl),k(RShift),L2_ENTER,k(RAlt),k(BSpace),Trans,   Trans      ],
    // ],
    // &[
    //     &[k(F1),         k(F2),   k(F3),     k(F4),     k(F5),    k(F6),k(F7),      k(F8),  k(F9),    k(F10), k(F11),  k(F12)],
    //     &[Trans,         k(Pause),Trans,     k(PScreen),Trans,    Trans,Trans,      Trans,  k(Delete),Trans,  Trans,   Trans ],
    //     &[Trans,         Trans,   k(NumLock),k(Insert), k(Escape),Trans,k(CapsLock),k(Left),k(Down),  k(Up),  k(Right),Trans ],
    //     &[k(NonUsBslash),k(Undo), CUT,       COPY,      PASTE,    Trans,Trans,      k(Home),k(PgDown),k(PgUp),k(End),  Trans ],
    //     &[Trans,         Trans,   Trans,     Trans,     Trans,    Trans,Trans,      Trans,  Trans,    Trans,  Trans,   Trans ],
    // ],
    // &[
    //     &[Trans,    Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans    ],
    //     &[s!(Grave),s!(Kb1),s!(Kb2),s!(Kb3),s!(Kb4),s!(Kb5),s!(Kb6),s!(Kb7),s!(Kb8),s!(Kb9),s!(Kb0),s!(Minus)],
    //     &[ k(Grave), k(Kb1), k(Kb2), k(Kb3), k(Kb4), k(Kb5), k(Kb6), k(Kb7), k(Kb8), k(Kb9), k(Kb0), k(Minus)],
    //     &[a!(Grave),a!(Kb1),a!(Kb2),a!(Kb3),a!(Kb4),a!(Kb5),a!(Kb6),a!(Kb7),a!(Kb8),a!(Kb9),a!(Kb0),a!(Minus)],
    //     &[Trans,    Trans,  Trans,  Trans,  CSPACE, Trans,  Trans,  Trans,  Trans,  Trans,  Trans,  Trans    ],
    // ],
    // &[
    //     &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
    //     &[k(F1),k(F2),k(F3),k(F4),k(F5),k(F6),k(F7),k(F8),k(F9),k(F10),k(F11),k(F12)],
    //     &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
    //     &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
    //     &[Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans,Trans, Trans, Trans ],
    // ],
];

#[app(device = stm32f4xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        usb_dev: UsbDevice,
        usb_class: UsbClass,
        matrix: Matrix<Cols, Rows>,
        debouncer: Debouncer<PressedKeys<U6, U4>>,
        layout: Layout,
        timer: timer::Timer<pac::TIM3>,
        led: Ws2812<Spi<SPI5, (NoSck, NoMiso, PB8<Alternate<gpio::AF6>>)>>,

        data: [RGB8; 43],
        last: [[bool; 4]; 6],
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<USB>>> = None;

        defmt::info!("Hello, world!");

        let board = ctx.device;

        let rcc = board.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(25.mhz())
            .hclk(96.mhz())
            .sysclk(96.mhz())
            .pclk1(48.mhz())
            .pclk2(96.mhz())
            .require_pll48clk()
            .freeze();

        let gpioa = board.GPIOA.split();
        let gpiob = board.GPIOB.split();

        let spi = Spi::spi5(
            board.SPI5,
            (NoSck, NoMiso, gpiob.pb8.into_alternate_af6()),
            MODE,
            3_000_000.hz(),
            clocks
        );
        let led = Ws2812::new(spi);

//         // set 0x424C in DR10 for dfu on reset
//         let bkp = rcc
//             .bkp
//             .constrain(c.device.BKP, &mut rcc.apb1, &mut c.device.PWR);
//         bkp.write_data_register_low(9, 0x424C);

//         // BluePill board has a pull-up resistor on the D+ line.
//         // Pull the D+ pin down to send a RESET condition to the USB bus.
//         let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
//         usb_dp.set_low().unwrap();
//         cortex_m::asm::delay(clocks.sysclk().0 / 100);

//         let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
//         led.set_high().unwrap();
        let leds = Leds {
            // caps_lock: led
        };

//         let usb_dm = gpioa.pa11;
//         let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        let usb = USB {
            usb_global: board.OTG_FS_GLOBAL,
            usb_device: board.OTG_FS_DEVICE,
            usb_pwrclk: board.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate_af10(),
            pin_dp: gpioa.pa12.into_alternate_af10(),
            // hclk: clocks.hclk(),
        };

        *USB_BUS = Some(UsbBus::new(usb, EP_MEMORY));
        let usb_bus = USB_BUS.as_ref().unwrap();

        let usb_class = keyberon::new_class(usb_bus, leds);
        let usb_dev = keyberon::new_device(usb_bus);

        let mut timer =
            timer::Timer::tim3(board.TIM3, 1.khz(), clocks);
        timer.listen(timer::Event::TimeOut);

        let matrix = Matrix::new(
            Cols(
                gpioa.pa6.into_pull_up_input(),
                gpioa.pa7.into_pull_up_input(),
                gpioa.pa8.into_pull_up_input(),
                gpioa.pa9.into_pull_up_input(),
            ),
            Rows(
                gpioa.pa0.into_push_pull_output(),
                gpioa.pa1.into_push_pull_output(),
                gpioa.pa2.into_push_pull_output(),
                gpioa.pa3.into_push_pull_output(),
                gpioa.pa4.into_push_pull_output(),
                gpioa.pa5.into_push_pull_output(),
            ),
        );

        let c = 5;

        let de = Debouncer::new(
            PressedKeys::default(),
            PressedKeys::default(),
            c,
        );

        init::LateResources {
            usb_dev,
            usb_class,
            timer,
            debouncer: de,
            matrix: matrix.unwrap(),
            layout: Layout::new(LAYERS),
            led,
            data: [colors::BLACK; 43],
            last: [[false; 4]; 6]
        }
    }

    #[task(binds = OTG_FS, priority = 2, resources = [usb_dev, usb_class])]
    fn usb_tx(mut c: usb_tx::Context) {
        usb_poll(&mut c.resources.usb_dev, &mut c.resources.usb_class);
    }

    #[task(binds = TIM3, priority = 1, resources = [usb_class, matrix, debouncer, layout, timer, led, data, last])]
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

        let coloop = COLOOP.get_or_insert_with(|| all_colors.iter().cloned().cycle());
        let coloop2 = COLOOP2.get_or_insert_with(|| all_colors.iter().cloned().cycle());

        c.resources.timer.clear_interrupt(timer::Event::TimeOut);

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
            _ => {},
        }
        send_report(c.resources.layout.keycodes(), &mut c.resources.usb_class);

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
            fix_spi_errors();

            c.resources.led.write(gamma(c.resources.data.iter().cloned())).ok();
        }
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }
};

fn send_report(
    iter: impl Iterator<Item = KeyCode>,
    usb_class: &mut resources::usb_class<'_>
) {
    use rtic::Mutex;
    let report: KbHidReport = iter.collect();
    if usb_class.lock(|k| k.device_mut().set_keyboard_report(report.clone())) {
        while let Ok(0) = usb_class.lock(|k| k.write(report.as_bytes())) {}
    }
}

fn usb_poll(usb_dev: &mut UsbDevice, keyboard: &mut UsbClass) {
    if usb_dev.poll(&mut [keyboard]) {
        keyboard.poll();
    }
}



fn scanner(
    data: &mut [RGB8],
    idx: &mut usize,
    color: &mut Cycle<Cloned<core::slice::Iter<'_, RGB<u8>>>>,
    go_down: &mut bool
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
        _ => 24
    }
}

// Fix Errors
fn fix_spi_errors() {
    unsafe {
        let spi = &*SPI5::ptr();

        // Read from DR register to clear OVR
        let _ = spi.dr.read();

        // Read from SR to clear CRCERR and OVR
        spi.sr.modify(|r, w| {
            let _ = r.txe().bit_is_set();

            if r.crcerr().bit_is_set() {
                w.crcerr().clear_bit();
            }
            w
        });

        // Write to CR1 to clear MODF
        spi.cr1.modify(|_r, w| w);
    }
}
