#![no_std]
#![no_main]
#![feature(precise_pointer_size_matching)]
#![feature(type_alias_impl_trait)]

use core::sync::atomic::AtomicBool;

use genawaiter::stack::Co;
use luhack_badge as _;
use noline::history::StaticHistory;
use noline::line_buffer::StaticBuffer;
use rp2040_hal::usb::UsbBus;

static LOCKED: AtomicBool = AtomicBool::new(true);

async fn usb_task(
    mut co: Co<'_, ()>,
    mut serial: noline::sync::embedded::IO<'static, usbd_serial::SerialPort<'static, UsbBus>>,
) {
    let _ = serial.read(&mut co).await;

    let mut editor =
        noline::sync::Editor::<StaticBuffer<128>, StaticHistory<128>, _>::new(&mut co, &mut serial)
            .await
            .unwrap();

    loop {
        if let Ok(line) = editor.readline(&mut co, "> ", &mut serial).await {
            let mut it = line.trim().split_whitespace();

            let unlocked = !LOCKED.load(core::sync::atomic::Ordering::SeqCst);

            match it.next() {
                Some("help") => {
                    serial
                        .write(&mut co, b"Hi!, have some help:\r\n")
                        .await
                        .unwrap();
                    serial
                        .write(&mut co, b"  help: This command\r\n")
                        .await
                        .unwrap();
                    serial
                        .write(&mut co, b"  reboot: Reboot the device\r\n")
                        .await
                        .unwrap();
                    if unlocked {
                        serial
                            .write(&mut co, b"  flag: claim a prize\r\n")
                            .await
                            .unwrap();
                    }
                }
                Some("reboot") => {
                    cortex_m::peripheral::SCB::sys_reset();
                }
                Some("flag") if unlocked => {
                    let flag = luhack_badge::encrypt!(env!("FLAG_UNLOCKED_COMMAND").as_bytes());
                    serial.write(&mut co, b"Here have a flag!: ").await.unwrap();
                    serial.write(&mut co, &flag).await.unwrap();
                    serial.write(&mut co, b"\r\n").await.unwrap();
                }
                Some(x) => {
                    serial.write(&mut co, b"No idea, sorry.\r\n").await.unwrap();
                }
                None => {}
            }
        }
    }
}

pub struct TotallySend<T>(pub T);

unsafe impl<T> Send for TotallySend<T> {}

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ, ADC_IRQ_FIFO, PWM_IRQ_WRAP])]
mod app {
    use core::cell::RefCell;

    use display_interface_spi::SPIInterface;
    use embedded_graphics::pixelcolor::Rgb565;
    use embedded_graphics::prelude::DrawTarget;
    use embedded_hal::digital::blocking::{OutputPin, ToggleableOutputPin};
    use embedded_hal::i2c::blocking::I2c;
    use fugit::RateExtU32;
    use hal::gpio::bank0::{Gpio16, Gpio17, Gpio25, Gpio4, Gpio5};
    use hal::gpio::{FunctionUart, Output, Pin, PushPull};
    use hal::usb::UsbBus;
    use hal::{gpio, uart, Clock};
    use luhack_badge::encrypt;
    use luhack_badge::message::{Command, LuToHack};
    use mipidsi::Orientation;
    use rp2040_hal::gpio::bank0::{Gpio18, Gpio19};
    use rp2040_hal::gpio::FunctionI2C;
    use rp_pico::hal::{self, Sio, Watchdog};

    use luhack_badge::rendering::{FixedST7789, MyDepthBuffer, MyDisplay, MyFrameBuffer};
    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid};

    use crate::{TotallySend, LOCKED};

    #[shared]
    struct Shared {
        delay: cortex_m::delay::Delay,
    }

    #[local]
    struct Local {
        led: Pin<Gpio25, Output<PushPull>>,
        display: MyDisplay,
        i2c0: hal::i2c::I2C<hal::pac::I2C0, I2c0Pins>,
        i2c1: hal::i2c::peripheral::I2CPeripheralEventIterator<hal::pac::I2C1, I2c1Pins>,
        uart1: uart::UartPeripheral<uart::Enabled, hal::pac::UART1, Uart1Pins>,
        usb_gen: TotallySend<
            ::core::pin::Pin<&'static mut dyn genawaiter::Generator<(), Yield = (), Return = ()>>,
        >,
        usb_dev: UsbDevice<'static, UsbBus>,
        serial: TotallySend<&'static RefCell<usbd_serial::SerialPort<'static, UsbBus>>>,
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type RtcMonotonic = rp2040_monotonic::Rp2040Monotonic;

    type Duration = <RtcMonotonic as rtic::Monotonic>::Duration;

    type I2c0Pins = (
        Pin<Gpio16, gpio::Function<gpio::I2C>>,
        Pin<Gpio17, gpio::Function<gpio::I2C>>,
    );

    type I2c1Pins = (
        Pin<Gpio18, gpio::Function<gpio::I2C>>,
        Pin<Gpio19, gpio::Function<gpio::I2C>>,
    );

    type Uart1Pins = (
        Pin<Gpio4, gpio::Function<gpio::Uart>>,
        Pin<Gpio5, gpio::Function<gpio::Uart>>,
    );

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        unsafe {
            hal::sio::spinlock_reset();
        }

        let mut resets = cx.device.RESETS;
        let mut watchdog = Watchdog::new(cx.device.WATCHDOG);
        let clocks = luhack_badge::clock::configure_overclock(
            cx.device.XOSC,
            cx.device.CLOCKS,
            cx.device.PLL_SYS,
            cx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        );

        defmt::info!("Hello world");

        luhack_badge::init_heap();

        let sio = Sio::new(cx.device.SIO);
        let pins = rp_pico::Pins::new(
            cx.device.IO_BANK0,
            cx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut led = pins.led.into_push_pull_output();
        led.set_high().unwrap();

        let timer = rp2040_monotonic::Rp2040Monotonic::new(cx.device.TIMER);

        let mut delay =
            cortex_m::delay::Delay::new(cx.core.SYST, clocks.system_clock.freq().to_Hz());

        let _spi_sclk = pins.gpio10.into_mode::<gpio::FunctionSpi>();
        let _spi_mosi = pins.gpio11.into_mode::<gpio::FunctionSpi>();
        let _spi_miso = pins.gpio12.into_mode::<gpio::FunctionSpi>();
        let spi_cs = pins.gpio13.into_push_pull_output();
        let spi_rst = pins.gpio14.into_push_pull_output();

        let spi_dc = pins.gpio15.into_push_pull_output();

        let spi = hal::spi::Spi::<_, _, 8>::new(cx.device.SPI1);
        let spi = spi.init(
            &mut resets,
            clocks.peripheral_clock.freq(),
            24u32.MHz(),
            &eh_0_2::spi::MODE_3,
        );

        let di = SPIInterface::new(spi, spi_dc, spi_cs);

        let mut display: _  = mipidsi::Builder::with_model(di, FixedST7789(mipidsi::models::ST7789))
            .with_orientation(Orientation::Landscape(true))
            .with_color_order(mipidsi::ColorOrder::Rgb)
            .init(&mut delay, Some(spi_rst))
            .unwrap();

        let mut display = MyDisplay(display);

        display
            .0
            .clear(<Rgb565 as embedded_graphics::prelude::RgbColor>::BLACK)
            .unwrap();

        let i2c_scl = pins.gpio17.into_mode::<FunctionI2C>();
        let i2c_sda = pins.gpio16.into_mode::<FunctionI2C>();

        let i2c0 = hal::i2c::I2C::new_controller(
            cx.device.I2C0,
            i2c_sda,
            i2c_scl,
            400u32.kHz(),
            &mut resets,
            clocks.system_clock.freq(),
        );

        let i2c_scl = pins.gpio19.into_mode::<FunctionI2C>();
        let i2c_sda = pins.gpio18.into_mode::<FunctionI2C>();

        let i2c1 = hal::i2c::I2C::new_peripheral_event_iterator(
            cx.device.I2C1,
            i2c_sda,
            i2c_scl,
            &mut resets,
            0x69,
        );

        let uart1_rx = pins.gpio5.into_mode::<FunctionUart>();
        let uart1_tx = pins.gpio4.into_mode::<FunctionUart>();

        let uart1 = uart::UartPeripheral::new(cx.device.UART1, (uart1_tx, uart1_rx), &mut resets)
            .enable(
                uart::common_configs::_115200_8_N_1,
                clocks.peripheral_clock.freq(),
            )
            .unwrap();

        defmt::info!("Setting up usb bus");

        type U = impl Sized;
        static USB_BUS: static_cell::StaticCell<U> = static_cell::StaticCell::new();
        let usb_bus = &*USB_BUS.init(UsbBusAllocator::new(UsbBus::new(
            cx.device.USBCTRL_REGS,
            cx.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));

        let serial = &*luhack_badge::forever!(RefCell::new(usbd_serial::SerialPort::new(usb_bus)));

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x6969, 0x1234))
            .manufacturer("Wardrobe Emporium")
            .product("Gnot a")
            .serial_number("Gnome")
            .device_class(2)
            .build();

        let shelf = luhack_badge::forever!(genawaiter::stack::Shelf::new());
        type UsbGen = impl genawaiter::Generator<(), Yield = (), Return = ()>;
        static USB_GEN: static_cell::StaticCell<UsbGen> = static_cell::StaticCell::new();
        let usb_gen = unsafe {
            let serial = noline::sync::embedded::IO::new(serial);

            TotallySend(core::pin::Pin::new_unchecked(
                USB_GEN.init(genawaiter::stack::Gen::new(shelf, move |co| {
                    crate::usb_task(co, serial)
                })) as &mut dyn genawaiter::Generator<(), Yield = (), Return = ()>,
            ))
        };

        defmt::info!("Setup done");

        display::spawn().unwrap();
        log_write::spawn().unwrap();
        cmd_write::spawn_after(Duration::secs(5)).unwrap();

        (
            Shared { delay },
            Local {
                led,
                display,
                i2c0,
                i2c1,
                uart1,
                usb_gen,
                usb_dev,
                serial: TotallySend(serial),
            },
            init::Monotonics(timer),
        )
    }

    #[task(priority = 2, local = [led, display,
                                  depth: MyDepthBuffer = MyDepthBuffer::new(),
                                  frame: MyFrameBuffer = MyFrameBuffer::new(),
                                  n: usize = 0usize])]
    fn display(cx: display::Context) {
        let n = cx.local.n;
        luhack_badge::rendering::draw_nth_frame(
            cx.local.display,
            cx.local.frame,
            cx.local.depth,
            *n,
            LOCKED.load(core::sync::atomic::Ordering::Relaxed),
        );
        *n += 1;
        display::spawn_after(Duration::millis(8)).unwrap();
        cx.local.led.toggle().unwrap();
    }

    #[task(priority = 1, local = [i2c0, idx: usize = 0])]
    fn cmd_write(cx: cmd_write::Context) {
        let msg = LuToHack::SendFlag(heapless::String::from(env!("FLAG_MORSE")));
        let cmd = Command::new(msg);

        let mut buf = [0u8; 128];
        let s = postcard::to_slice_cobs(&cmd, &mut buf).unwrap();
        if let Err(e) = cx.local.i2c0.write(0x69, s) {
            defmt::error!("err: {}", e);
        }

        cmd_write::spawn_after(Duration::secs(5)).unwrap();
    }

    #[task(priority = 2, local = [uart1, idx: usize = 0])]
    fn log_write(cx: log_write::Context) {
        use core::fmt::Write;

        *cx.local.idx = match *cx.local.idx {
            n @ 0..=9_999 => {
                let _ = writeln!(cx.local.uart1, "Loading... {}", n);
                n + 1
            }
            n @ 10_000..=10_011 => {
                let i = n - 10_000;
                let msg = [
                    "Exploiting CVE-2022-25375",
                    "Exploiting CVE-2022-25375.",
                    "Exploiting CVE-2022-25375..",
                    "Exploiting CVE-2022-25375...",
                    "Injecting shellcode",
                    "Waiting for response",
                    "Waiting for response..",
                    "Waiting for response...",
                    "Embedding rootkit into EFI",
                    "Embedding rootkit into EFI.",
                    "Embedding rootkit into EFI..",
                    "Dumping passwords...",
                ][i];

                let _ = writeln!(cx.local.uart1, "{}", msg);
                n + 1
            }
            10_012.. => {
                let _ = write!(cx.local.uart1, "Found flag: ");
                cx.local
                    .uart1
                    .write_full_blocking(&encrypt!(env!("FLAG_LOGS").as_bytes()));
                let _ = writeln!(cx.local.uart1, "");
                0
            }
        };

        log_write::spawn_after(Duration::millis(2)).unwrap();
    }

    #[task(binds = I2C1_IRQ,
           local = [i2c1,
                    buf: heapless::Vec<u8, 16> = heapless::Vec::new()],
           priority = 3)]
    fn wake_i2c1(cx: wake_i2c1::Context) {
        let x = cx.local.i2c1.next();
        unsafe {
            let x = (*hal::pac::I2C1::ptr()).ic_intr_stat.read();
            if x.r_tx_empty().is_active() {
                (*hal::pac::I2C1::ptr())
                    .ic_intr_mask
                    .write(|w| w.m_tx_empty().enabled());
            }
        };

        let mut buf = [0u8; 32];
        let len = cx.local.i2c1.read(&mut buf);
        cx.local.buf.extend_from_slice(&buf[..len]);

        let key = luhack_badge::encrypt!(b"open sesame");
        if cx.local.buf.as_slice() == &key {
            defmt::info!("unlocking");
            LOCKED.store(false, core::sync::atomic::Ordering::SeqCst);

            cx.local.buf.clear();
        }
    }

    #[task(binds = USBCTRL_IRQ, local = [usb_dev, usb_gen, serial], priority = 2)]
    fn wake_usb(cx: wake_usb::Context) {
        use genawaiter::Generator;
        defmt::trace!("usb irq");

        let ready = {
            let mut serial = cx
                .local
                .serial
                .0
                .try_borrow_mut()
                .expect("usb task borrowed serial between yields");
            cx.local.usb_dev.poll(&mut [&mut *serial])
        };

        if ready {
            let r = match cx.local.usb_gen.0.as_mut().resume(()) {
                genawaiter::GeneratorState::Yielded(()) => {}
                genawaiter::GeneratorState::Complete(()) => {
                    defmt::error!("usb task generator completed?");
                    return;
                }
            };
        }

        defmt::trace!("usb irq done");
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
