//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::*;
// use defmt_serial as _;
use defmt_rtt as _;
use panic_probe as _;
use static_cell::StaticCell;
use usb_device::{class_prelude::*, prelude::*};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico::{self as bsp, hal::gpio::SioInput};
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    self,
    clocks::init_clocks_and_plls,
    pac::{self},
    // prelude::*,
    watchdog::Watchdog,
    Sio,
};
use embedded_hal::digital::InputPin;

// use usb_device::{class_prelude::*, prelude::*};
use usbd_hid::hid_class::HidClassSettings;
use usbd_hid::hid_class::{HIDClass, HidProtocol, HidSubClass};

// Stolen from the windows xbox controller driver I think?
const REPORT_DESCRIPTOR: [u8; 120] = [
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01, 0xA1, 0x00, 0x09, 0x30, 0x09, 0x31, 0x15, 0x00, 0x26, 0xFF,
    0xFF, 0x35, 0x00, 0x46, 0xFF, 0xFF, 0x95, 0x02, 0x75, 0x10, 0x81, 0x02, 0xC0, 0xA1, 0x00, 0x09,
    0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xFF, 0xFF, 0x35, 0x00, 0x46, 0xFF, 0xFF, 0x95, 0x02, 0x75,
    0x10, 0x81, 0x02, 0xC0, 0xA1, 0x00, 0x09, 0x32, 0x15, 0x00, 0x26, 0xFF, 0xFF, 0x35, 0x00, 0x46,
    0xFF, 0xFF, 0x95, 0x01, 0x75, 0x10, 0x81, 0x02, 0xC0, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0A, 0x95,
    0x0A, 0x75, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x39, 0x15, 0x01, 0x25, 0x08, 0x35, 0x00, 0x46,
    0x3B, 0x10, 0x66, 0x0E, 0x00, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x75, 0x02, 0x95, 0x01, 0x81,
    0x03, 0x75, 0x08, 0x95, 0x02, 0x81, 0x03, 0xC0,
];

#[repr(C, packed)]
struct GamepadReport {
    report_id: u8,
    packet_size: u8,
    misc_buttons: u8,
    buttons: u8,
    left_trigger: u8,
    right_trigger: u8,
    x: i16, // -0x7fff is left
    y: i16, // -0x7fff is up

    right_x: i16,
    right_y: i16,

    unused: [u8; 6],
}

fn create_hid_class<B: UsbBus>(bus_ref: &UsbBusAllocator<B>) -> HIDClass<'_, B> {
    HIDClass::new_with_settings(
        bus_ref,
        &REPORT_DESCRIPTOR,
        60,
        HidClassSettings {
            device_class: 0xff,
            protocol: HidProtocol::Keyboard,
            subclass: HidSubClass::Xbox360,
            ..Default::default()
        },
    )
}

type ControllerMapping = [hal::gpio::Pin<
    hal::gpio::DynPinId,
    bsp::hal::gpio::FunctionSio<SioInput>,
    bsp::hal::gpio::PullUp,
>; 13];

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    static INPUTS: StaticCell<[ControllerMapping; 2]> = StaticCell::new();
    let inputs = INPUTS.init_with(|| {
        [
            [
                // Player one
                pins.gpio10.into_pull_up_input().into_dyn_pin(), // A4
                pins.gpio11.into_pull_up_input().into_dyn_pin(), // A3
                pins.gpio17.into_pull_up_input().into_dyn_pin(), // B1
                pins.gpio16.into_pull_up_input().into_dyn_pin(), // B2
                pins.gpio13.into_pull_up_input().into_dyn_pin(), // A1
                pins.gpio12.into_pull_up_input().into_dyn_pin(), // A2
                pins.gpio9.into_pull_up_input().into_dyn_pin(),  // Start button
                pins.gpio27.into_pull_up_input().into_dyn_pin(), // North
                pins.gpio26.into_pull_up_input().into_dyn_pin(), // West
                pins.gpio22.into_pull_up_input().into_dyn_pin(), // East
                pins.gpio28.into_pull_up_input().into_dyn_pin(), // South
                pins.gpio15.into_pull_up_input().into_dyn_pin(), // B3
                pins.gpio14.into_pull_up_input().into_dyn_pin(), // B4
            ],
            [
                // Player two
                pins.gpio3.into_pull_up_input().into_dyn_pin(),
                pins.gpio2.into_pull_up_input().into_dyn_pin(),
                pins.gpio4.into_pull_up_input().into_dyn_pin(),
                pins.gpio5.into_pull_up_input().into_dyn_pin(),
                pins.gpio6.into_pull_up_input().into_dyn_pin(),
                pins.gpio7.into_pull_up_input().into_dyn_pin(),
                pins.gpio8.into_pull_up_input().into_dyn_pin(),
                pins.gpio20.into_pull_up_input().into_dyn_pin(), // North
                pins.gpio19.into_pull_up_input().into_dyn_pin(), // West
                pins.gpio18.into_pull_up_input().into_dyn_pin(), // East
                pins.gpio21.into_pull_up_input().into_dyn_pin(), // South
                pins.gpio0.into_pull_up_input().into_dyn_pin(),
                pins.gpio1.into_pull_up_input().into_dyn_pin(),
            ],
        ]
    });

    // Create the USB Bus Allocator
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let bus_ref = unsafe {
        static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;
        USB_BUS = Some(usb_bus);
        USB_BUS.as_ref().unwrap()
    };

    // Set up the USB HID Class Device driver, providing Mouse Reports
    let player_one = create_hid_class(bus_ref);
    let player_two = create_hid_class(bus_ref);

    // Create a USB device with a fake VID and PID
    let usb_device = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x24c6, 0xfafe))
        .strings(&[StringDescriptors::default()
            .manufacturer("Rock Candy")
            .product("Rock Candy Gamepad for Xbox 360")
            .serial_number("TEST")])
        .unwrap()
        .device_class(0)
        .build();

    critical_section::with(|token| {
        *GADGET.borrow_ref_mut(token) = Some(Gadget {
            players: [player_one, player_two],
            usb_device,
        });
    });

    // Move the cursor up and down every 200ms
    let mut report = GamepadReport {
        report_id: 0x00, // Button packet
        packet_size: core::mem::size_of::<GamepadReport>() as u8,
        misc_buttons: 0,
        buttons: 0,
        left_trigger: 0,
        right_trigger: 0,
        x: 0,
        y: 0,
        right_x: 0,
        right_y: 0,
        unused: [0, 0, 0, 0, 0, 0],
    };

    loop {
        for (player_index, inputs) in inputs.iter_mut().enumerate() {
            report.misc_buttons = 0;
            report.buttons = 0;
            report.left_trigger = 0;
            report.right_trigger = 0;
            report.x = 0;
            report.y = 0;
            for (index, input) in inputs.iter_mut().enumerate() {
                let state = input.is_high().unwrap();
                if state {
                    continue;
                }
                if index < 2 {
                    // Actual buttons
                    report.buttons = 1 << index;
                } else if index < 2 + 4 {
                    report.buttons = 1 << (index + 2);
                } else if index < 2 + 4 + 1 {
                    report.misc_buttons = 1 << 4;
                } else if index < 2 + 4 + 1 + 4 {
                    // Joysticks
                    let value = if index & 1 == 1 { i16::MAX } else { i16::MIN };
                    if index & 2 == 0 {
                        report.x = value;
                    } else {
                        report.y = value;
                    }
                } else if index < 2 + 4 + 1 + 4 + 2 {
                    // Triggers
                    if index & 1 == 0 {
                        report.left_trigger = u8::MAX;
                    } else {
                        report.right_trigger = u8::MAX;
                    }
                }
            }
            if let Err(err) = push_movement(&report, player_index) {
                info!(
                    "Error pushing movement to controller {:?}",
                    defmt::Debug2Format(&err)
                );
            }
        }
        poll_usb();
    }
}

unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    core::slice::from_raw_parts((p as *const T) as *const u8, core::mem::size_of::<T>())
}

/// Submit a new mouse movement report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_movement(
    report: &GamepadReport,
    player_index: usize,
) -> Result<usize, usb_device::UsbError> {
    info!("Pushing a movement");
    critical_section::with(|token| {
        info!("Pushing a movement acquired lock");
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        let input = unsafe { any_as_u8_slice(report) };
        info!("Input was: {}", input.len());
        GADGET
            .borrow_ref_mut(token)
            .as_ref()
            .map(|gadget| gadget.players[player_index].push_raw_input(input))
    })
    .unwrap()
}

struct Gadget {
    usb_device: UsbDevice<'static, hal::usb::UsbBus>,
    players: [HIDClass<'static, hal::usb::UsbBus>; 2],
}

/// The USB Device Driver (shared with the interrupt).
static GADGET: Mutex<RefCell<Option<Gadget>>> = Mutex::new(RefCell::new(None));

fn poll_usb() {
    // Handle USB request
    critical_section::with(|token| {
        GADGET.borrow_ref_mut(token).as_mut().map(|gadget| {
            let mut players = gadget.players.iter_mut();
            gadget
                .usb_device
                .poll(&mut [players.next().unwrap(), players.next().unwrap()]);
        })
    });
}
