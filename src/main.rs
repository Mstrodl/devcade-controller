//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use bsp::entry;
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;
use usb_device::{class_prelude::*, prelude::*};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use adafruit_kb2040 as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    self,
    clocks::{init_clocks_and_plls, Clock},
    pac::{self, interrupt},
    prelude::*,
    watchdog::Watchdog,
};

use usb_device::{class_prelude::*, prelude::*};
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;

#[gen_hid_descriptor(
    (collection = APPLICATION, usage_page = GENERIC_DESKTOP, usage = 0x05) = {
        // Button packet
        (collection = PHYSICAL, report_id = 0x20) = {
            (usage_page = GENERIC_DESKTOP, usage = 0x00) = {
                #[item_settings constant,variable,absolute] unknown=input;
            };
            (usage_page = GENERIC_DESKTOP, usage = 0x3b) = {
                #[item_settings data,variable,absolute] payload_size=input;
            };
            (usage_page = BUTTON, usage_min = 1, usage_max = 16) = {
                #[packed_bits 16] #[item_settings data,variable,absolute] buttons=input;
            };
            (usage_page = GENERIC_DESKTOP,) = {
                (usage = 0x33,) = {
                    #[item_settings data,variable,absolute] rx=input;
                };
                (usage = 0x34,) = {
                    #[item_settings data,variable,absolute] ry=input;
                };
            };
            (usage_page = GENERIC_DESKTOP, usage = POINTER,) = {
                (collection = PHYSICAL,) = {
                    (usage_page = GENERIC_DESKTOP,) = {
                        (usage = X,) = {
                            #[item_settings data, variable, absolute] x=input;
                        };
                        (usage = Y,) = {
                            #[item_settings data, variable, absolute] y=input;
                        };
                    };
                };
            };
            (usage_page = GENERIC_DESKTOP, usage = POINTER,) = {
                (collection = PHYSICAL,) = {
                    (usage_page = GENERIC_DESKTOP,) = {
                        (usage = Z,) = {
                            #[item_settings data, variable, absolute] z=input;
                        };
                        (usage = 0x35,) = {
                            #[item_settings data, variable, absolute] rz=input;
                        };
                    };
                };
            };
        };
        // Xbox button packet
        (collection = PHYSICAL, report_id = 0x07) = {
            (usage_page = GENERIC_DESKTOP,usage = 0x00) = {
                #[item_settings constant, variable, absolute] unknown2=input;
            };

            (usage_page = GENERIC_DESKTOP, usage = 0x3b) = {
                #[item_settings constant, variable, absolute] payload_size2=input;
            };

            (usage_page = BUTTON, usage = 0x10) = {
                #[packed_bits 1] #[item_settings data, variable, absolute] xbox_button=input;
            }
        };
    }
)]
struct GamepadReport {
    // Normal buttons
    unknown: [u8; 2],
    payload_size: u8,
    buttons: u16,
    rx: u16,
    ry: u16,
    x: i16,
    y: i16,
    z: i16,
    rz: i16,
    // Xbox button
    unknown2: [u8; 2],
    payload_size2: u8,
    xbox_button: u8,
}

#[repr(C, packed)]
struct GamepadReportButtons {
    report_id: u8,
    // Normal buttons
    unknown: [u8; 2],
    payload_size: u8,
    buttons: u16,
    rx: u16,
    ry: u16,
    x: i16,
    y: i16,
    z: i16,
    rz: i16,
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

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
    let usb_hid = HIDClass::new(bus_ref, GamepadReport::desc(), 60);

    // Create a USB device with a fake VID and PID
    let usb_device = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x24c6, 0xfafe))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Twitchy Mousey")
            .serial_number("TEST")])
        .unwrap()
        .device_class(0)
        .build();

    critical_section::with(|token| {
        *GADGET.borrow_ref_mut(token) = Some(Gadget {
            usb_hid,
            usb_device,
        });
    });

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    // Move the cursor up and down every 200ms
    let mut sequence = 0;
    let mut buttons = GamepadReportButtons {
        report_id: 0x20, // Button packet
        unknown: [0, sequence],
        payload_size: 0x0e, //core::mem::size_of::<GamepadReportButtons>() as u8,
        buttons: 0,
        rx: 0,
        ry: 0,
        x: 0,
        y: 0,
        z: 0,
        rz: 0,
    };

    loop {
        delay.delay_ms(100);
        buttons.buttons = 0;
        buttons.unknown[1] = sequence;
        sequence += 1;
        push_movement(&buttons).ok().unwrap_or(0);

        delay.delay_ms(100);

        buttons.buttons = 0xffff;
        buttons.unknown[1] = sequence;
        sequence += 1;
        push_movement(&buttons).ok().unwrap_or(0);
    }
}

unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    core::slice::from_raw_parts((p as *const T) as *const u8, core::mem::size_of::<T>())
}

/// Submit a new mouse movement report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_movement(report: &GamepadReportButtons) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|token| {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        GADGET.borrow_ref_mut(token).as_ref().map(|gadget| {
            gadget
                .usb_hid
                .push_raw_input(unsafe { any_as_u8_slice(&report) })
        })
    })
    .unwrap()
}

struct Gadget {
    usb_device: UsbDevice<'static, hal::usb::UsbBus>,
    usb_hid: HIDClass<'static, hal::usb::UsbBus>,
}

/// The USB Device Driver (shared with the interrupt).
static GADGET: Mutex<RefCell<Option<Gadget>>> = Mutex::new(RefCell::new(None));

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
fn USBCTRL_IRQ() {
    // Handle USB request
    critical_section::with(|token| {
        GADGET
            .borrow_ref_mut(token)
            .as_mut()
            .map(|gadget| gadget.usb_device.poll(&mut [&mut gadget.usb_hid]))
    });
}

// End of file
