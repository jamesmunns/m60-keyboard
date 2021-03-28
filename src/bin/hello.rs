#![no_main]
#![no_std]

use keyboard as _; // global logger + panicking-behavior + memory layout

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Hello, world!");

    keyboard::exit()
}
