//DMA
//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(const_fn_floating_point_arithmetic)]
#![feature(const_float_bits_conv)]
#![feature(const_for)]
#![feature(generic_const_exprs)]

use bsp::{
    entry,
    hal::{
        dma::DMAExt,
        gpio::{FunctionPio0, Pin},
        pio::PIOExt,
    },
};

use defmt::export::panic;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;

use gb_core::{gameboy::GameBoy, hardware::Screen};
use ili9341::{DisplaySize, DisplaySize240x320};

// use ili9341_pio::{DisplaySize, DisplaySize400x200};
//use ili9341_pio::Orientation;
use panic_probe as _;
// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.

use rp_pico::hal::clocks::Clock;
// use sparkfun_pro_micro_rp2040 as bsp;
use alloc_cortex_m::CortexMHeap;
use core::alloc::Layout;
extern crate alloc;
use alloc::vec::*;
use bsp::hal::{sio::Sio, watchdog::Watchdog};

mod array_scaler;
mod clock;
mod const_math;
mod pio_dma_interface;
mod pio_interface;
mod scaler;
mod stream_display;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

#[entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 131000;
        static mut HEAP: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { ALLOCATOR.init((&mut HEAP).as_ptr() as usize, HEAP_SIZE) }
    }

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    pac.VREG_AND_CHIP_RESET
        .vreg
        .write(|w| unsafe { w.vsel().bits(0b1101) });

    let clocks = clock::configure_overclock(
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    );
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let pins = bsp::hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let dma = pac.DMA.split(&mut pac.RESETS);

    let mut led_pin = pins.gpio25.into_push_pull_output();

    //Configure display

    let reset = pins.gpio2.into_push_pull_output();
    let mut cs = pins.gpio3.into_push_pull_output();
    let rs = pins.gpio4.into_push_pull_output();
    let _: Pin<_, FunctionPio0> = pins.gpio5.into_mode();

    let mut rd = pins.gpio6.into_push_pull_output();

    let _: Pin<_, FunctionPio0> = pins.gpio7.into_mode();
    let _: Pin<_, FunctionPio0> = pins.gpio8.into_mode();
    let _: Pin<_, FunctionPio0> = pins.gpio9.into_mode();
    let _: Pin<_, FunctionPio0> = pins.gpio10.into_mode();
    let _: Pin<_, FunctionPio0> = pins.gpio11.into_mode();
    let _: Pin<_, FunctionPio0> = pins.gpio12.into_mode();
    let _: Pin<_, FunctionPio0> = pins.gpio13.into_mode();
    let _: Pin<_, FunctionPio0> = pins.gpio14.into_mode();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    rd.set_high().unwrap();
    cs.set_low().unwrap();
    let endianess = |be: bool, val: u16| {
        if be {
            val.to_le()
        } else {
            val.to_be()
        }
    };
    let interface =
        pio_interface::PioInterface::new(4f32, rs, &mut pio, sm0, 5, (7, 14), endianess);

    let mut display = ili9341::Ili9341::new(
        interface,
        reset,
        &mut delay,
        ili9341::Orientation::LandscapeFlipped,
        ili9341::DisplaySize320x480,
    )
    .unwrap();

    let gb_rom = load_rom_from_path();
    let cart = gb_rom.into_cartridge();
    let boot_rom = gb_core::hardware::boot_rom::Bootrom::new(Some(
        gb_core::hardware::boot_rom::BootromData::from_bytes(include_bytes!(
            "C:\\Users\\PabloLozano\\Projects\\pico\\dmg\\dmg_boot.bin"
        )),
    ));
    let mut led_state = false;

    let screen = GameboyLineBufferDisplay::new();
    let mut gameboy = GameBoy::create(screen, cart, boot_rom);

    const SCREEN_WIDTH: usize =
        const_math::floorf(<DisplaySize240x320 as DisplaySize>::WIDTH as f32 / 1.0f32) as usize;
    const SCREEN_HEIGHT: usize =
        const_math::floorf(<DisplaySize240x320 as DisplaySize>::HEIGHT as f32 / 1.0f32) as usize;

    let spare: &'static mut [u16] =
        cortex_m::singleton!(: Vec<u16>  = alloc::vec![0; SCREEN_WIDTH ])
            .unwrap()
            .as_mut_slice();

    let dm_spare: &'static mut [u16] =
        cortex_m::singleton!(: Vec<u16>  = alloc::vec![0; SCREEN_WIDTH ])
            .unwrap()
            .as_mut_slice();

    let mut streamer = stream_display::Streamer::new(4f32, 5, (7, 14), dma.ch0, dm_spare, spare);

    loop {
        let display_iter = GameVideoIter::new(&mut gameboy);
        let mut scaler: scaler::ScreenScaler<
            144,
            160,
            { SCREEN_WIDTH },
            { SCREEN_HEIGHT },
            GameVideoIter,
        > = scaler::ScreenScaler::new(display_iter);

        display = display
            .async_transfer_mode(0, 0, SCREEN_HEIGHT as u16, SCREEN_WIDTH as u16, |iface| {
                let (sm, rs) = iface.free(&mut pio);

                let (rs, sm) =
                    streamer.stream::<SCREEN_WIDTH, _, _, _, _>(&mut pio, rs, sm, &mut scaler);

                pio_interface::PioInterface::new(4f32, rs, &mut pio, sm, 5, (7, 14), endianess)
            })
            .unwrap();

        if led_state {
            led_pin.set_low().unwrap();
        } else {
            led_pin.set_high().unwrap();
        }
        led_state = !led_state;
    }
}

pub struct GameVideoIter<'a> {
    gameboy: &'a mut GameBoy<GameboyLineBufferDisplay>,
    current_line_index: usize,
}
impl<'a> GameVideoIter<'a> {
    fn new(gameboy: &'a mut GameBoy<GameboyLineBufferDisplay>) -> Self {
        Self {
            gameboy: gameboy,
            current_line_index: 0,
        }
    }
}

impl<'a> Iterator for GameVideoIter<'a> {
    type Item = u16;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.gameboy.get_screen().turn_off {
                self.gameboy.get_screen().turn_off = false;
                return None;
            }
            if self.gameboy.get_screen().line_complete {
                let pixel = self.gameboy.get_screen().line_buffer[self.current_line_index];
                if self.current_line_index + 1 >= 160 {
                    self.current_line_index = 0;
                    self.gameboy.get_screen().line_complete = false;
                } else {
                    self.current_line_index = self.current_line_index + 1;
                }

                return Some(pixel);
            } else {
                self.gameboy.tick();
            }
        }
    }
}

struct GameboyLineBufferDisplay {
    line_buffer: Vec<u16>,
    line_complete: bool,
    turn_off: bool,
}

impl GameboyLineBufferDisplay {
    fn new() -> Self {
        Self {
            line_buffer: alloc::vec![0; 160],
            line_complete: false,
            turn_off: false,
        }
    }
}
impl Screen for GameboyLineBufferDisplay {
    fn turn_on(&mut self) {
        self.turn_off = true;
    }

    fn turn_off(&mut self) {
        //todo!()
    }

    fn set_pixel(&mut self, x: u8, _y: u8, color: gb_core::hardware::color_palette::Color) {
        let encoded_color = ((color.red as u16 & 0b11111000) << 8)
            + ((color.green as u16 & 0b11111100) << 3)
            + (color.blue as u16 >> 3);

        self.line_buffer[x as usize] = encoded_color;
    }
    fn scanline_complete(&mut self, _y: u8, _skip: bool) {
        self.line_complete = true;
    }

    fn draw(&mut self, _: bool) {}
}

pub fn load_rom_from_path() -> gb_core::hardware::rom::Rom<'static> {
    let rom_f = include_bytes!("C:\\Users\\PabloLozano\\Projects\\pico\\dmg\\sml.gb");
    gb_core::hardware::rom::Rom::from_bytes(rom_f)
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}
