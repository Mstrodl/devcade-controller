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
    let usb_hid = HIDClass::new(bus_ref, MouseReport::desc(), 60);

    // Create a USB device with a fake VID and PID
    let usb_device = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27da))
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
    loop {
        delay.delay_ms(100);

        let rep_up = MouseReport {
            x: 0,
            y: 4,
            buttons: 0,
            wheel: 0,
            pan: 0,
        };
        push_mouse_movement(rep_up).ok().unwrap_or(0);

        delay.delay_ms(100);

        let rep_down = MouseReport {
            x: 0,
            y: -4,
            buttons: 0,
            wheel: 0,
            pan: 0,
        };
        push_mouse_movement(rep_down).ok().unwrap_or(0);
    }
}

/// Submit a new mouse movement report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_mouse_movement(report: MouseReport) -> Result<usize, usb_device::UsbError> {
    critical_section::with(|token| {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        GADGET
            .borrow_ref_mut(token)
            .as_ref()
            .map(|gadget| gadget.usb_hid.push_input(&report))
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
