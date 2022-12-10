#![no_std]
#![no_main]
#![feature(precise_pointer_size_matching)]
#![feature(type_alias_impl_trait)]
#![feature(iter_array_chunks)]

use genawaiter::stack::Co;
use hal::i2c;
use hal::usb::UsbBus;
use heapless::spsc::Consumer;
use luhack_badge as _;
use luhack_badge::message::{Command, LuToHack};
use luhack_badge::morse::Morse;
use noline::history::StaticHistory;
use noline::line_buffer::StaticBuffer;
use postcard::accumulator::CobsAccumulator;
use rp2040_hal::gpio;
use rp_pico::hal;
use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812Direct;

#[allow(unused)]
type Duration = <rp2040_monotonic::Rp2040Monotonic as rtic::Monotonic>::Duration;
#[allow(unused)]
type Instant = <rp2040_monotonic::Rp2040Monotonic as rtic::Monotonic>::Instant;

pub enum WaitType {
    Duration(Duration),
    UntilRX,
}

pub type I2c0Pins = (
    gpio::Pin<gpio::bank0::Gpio16, gpio::Function<gpio::I2C>>,
    gpio::Pin<gpio::bank0::Gpio17, gpio::Function<gpio::I2C>>,
);

pub type I2c1Pins = (
    gpio::Pin<gpio::bank0::Gpio14, gpio::Function<gpio::I2C>>,
    gpio::Pin<gpio::bank0::Gpio15, gpio::Function<gpio::I2C>>,
);

async fn rx_msg(
    co: &mut Co<'_, WaitType>,
    mut rx: Consumer<'static, u8, 32>,
    mut neopixel: Ws2812Direct<hal::pac::PIO0, hal::pio::SM0, gpio::bank0::Gpio13>,
) {
    let mut cobs_buf = CobsAccumulator::<128>::new();

    loop {
        match rx.dequeue() {
            Some(x) => {
                let buf = [x];
                let mut window = &buf[..];
                'cobs: while !window.is_empty() {
                    window = match cobs_buf.feed::<Command<LuToHack>>(window) {
                        postcard::accumulator::FeedResult::Consumed => break 'cobs,
                        postcard::accumulator::FeedResult::OverFull(new_wind) => new_wind,
                        postcard::accumulator::FeedResult::DeserError(new_wind) => new_wind,
                        postcard::accumulator::FeedResult::Success { data, remaining } => {
                            if data.validate() {
                                handler(co, data.cmd, &mut neopixel).await;
                            }

                            remaining
                        }
                    }
                }
            }
            None => {
                co.yield_(WaitType::UntilRX).await;
            }
        }
    }
}

fn rgb(r: u8, g: u8, b: u8) -> RGB8 {
    // ???
    RGB8::new(g, r, b)
}

async fn tx_morse(
    co: &mut Co<'_, WaitType>,
    neopixel: &mut Ws2812Direct<hal::pac::PIO0, hal::pio::SM0, gpio::bank0::Gpio13>,
    msg: &str,
) {
    defmt::info!("Got morse message: {}", msg);

    let flag = luhack_badge::encrypt!(env!("FLAG_NEOPIXEL").as_bytes());
    let mut flag_byte_it = flag.into_iter().array_chunks::<3>();

    let it = core::iter::once(rgb(100, 0, 0));
    neopixel.write(it).unwrap();

    co.yield_(WaitType::Duration(Duration::millis(500))).await;

    let it = core::iter::once(rgb(0, 0, 0));
    neopixel.write(it).unwrap();

    co.yield_(WaitType::Duration(Duration::millis(100))).await;

    for signal in luhack_badge::morse::convert(msg) {
        defmt::info!("Sending a {}", signal);
        if signal.lit() {
            let [r, g, b] = flag_byte_it.next().unwrap_or([100, 0, 100]);
            let it = core::iter::once(RGB8::new(r, g, b));
            neopixel.write(it).unwrap();
        }

        co.yield_(WaitType::Duration(Duration::millis(signal.wait_ms())))
            .await;

        let it = core::iter::once(rgb(0, 0, 0));
        neopixel.write(it).unwrap();

        co.yield_(WaitType::Duration(Duration::millis(Morse::INTERVAL_MS)))
            .await;
    }
}

async fn handler(
    co: &mut Co<'_, WaitType>,
    msg: LuToHack,
    neopixel: &mut Ws2812Direct<hal::pac::PIO0, hal::pio::SM0, gpio::bank0::Gpio13>,
) {
    match msg {
        LuToHack::SendFlag(msg) => {
            tx_morse(co, neopixel, msg.as_str()).await;
        }
    }
}

async fn morse_state(
    mut co: Co<'_, WaitType>,
    rx: Consumer<'static, u8, 32>,
    neopixel: Ws2812Direct<hal::pac::PIO0, hal::pio::SM0, gpio::bank0::Gpio13>,
) {
    rx_msg(&mut co, rx, neopixel).await;
}

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

            match it.next() {
                Some("help") => {
                    serial.write(&mut co, b"Hi!, have some help:\r\n").await.unwrap();
                    serial.write(&mut co, b"  help: This command\r\n").await.unwrap();
                    serial.write(&mut co, b"  reboot: Reboot the device\r\n").await.unwrap();
                    serial.write(&mut co, b"  unlock: ???\r\n").await.unwrap();
                }
                Some("reboot") => {
                    cortex_m::peripheral::SCB::sys_reset();
                }
                Some("unlock") => {
                    serial.write(&mut co, b"Somewhere, a heavy door has opened\r\n").await.unwrap();
                    let _ = app::send_unlock::spawn();
                }
                Some("super_secret_hidden_command") => {
                    let flag = luhack_badge::encrypt!(env!("FLAG_FIRMWARE").as_bytes());
                    serial.write(&mut co, b"Here have another flag!: ").await.unwrap();
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

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [XIP_IRQ, ADC_IRQ_FIFO])]
mod app {
    use core::cell::RefCell;

    use cortex_m::peripheral::NVIC;
    use embedded_hal::digital::blocking::OutputPin;
    use embedded_hal::i2c::blocking::I2c;
    use genawaiter::stack::Shelf;
    use hal::gpio::bank0::Gpio25;
    use hal::gpio::{FunctionI2C, Output, Pin, PushPull};
    use hal::i2c::peripheral::I2CPeripheralEventIterator;
    use hal::i2c::I2C;
    use hal::pio::PIOExt;
    use hal::usb::UsbBus;
    use hal::Clock;
    use heapless::spsc::{Consumer, Producer};
    use noline::history::StaticHistory;
    use noline::line_buffer::StaticBuffer;
    use noline::sync::Editor;
    use rp2040_monotonic::fugit::RateExtU32;
    use rp_pico::hal::{self, Sio, Watchdog};
    use rtic::export::Queue;
    use usb_device::class_prelude::UsbBusAllocator;
    use usb_device::prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid};

    use crate::{morse_state, Duration, TotallySend};

    #[shared]
    struct Shared {
        delay: cortex_m::delay::Delay,
    }

    #[local]
    struct Local {
        led: Pin<Gpio25, Output<PushPull>>,
        neopixel_gen: TotallySend<
            ::core::pin::Pin<
                &'static mut dyn genawaiter::Generator<(), Yield = crate::WaitType, Return = ()>,
            >,
        >,
        usb_gen: TotallySend<
            ::core::pin::Pin<&'static mut dyn genawaiter::Generator<(), Yield = (), Return = ()>>,
        >,
        i2c0: I2CPeripheralEventIterator<hal::pac::I2C0, crate::I2c0Pins>,
        i2c1: hal::i2c::I2C<hal::pac::I2C1, crate::I2c1Pins>,
        i2c0_bytes_tx: Producer<'static, u8, 32>,
        usb_dev: UsbDevice<'static, UsbBus>,
        serial: TotallySend<&'static RefCell<usbd_serial::SerialPort<'static, UsbBus>>>,
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type RtcMonotonic = rp2040_monotonic::Rp2040Monotonic;

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

        let delay = cortex_m::delay::Delay::new(cx.core.SYST, clocks.system_clock.freq().to_Hz());

        let i2c_scl = pins.gpio17.into_mode::<FunctionI2C>();
        let i2c_sda = pins.gpio16.into_mode::<FunctionI2C>();

        let i2c0 =
            I2C::new_peripheral_event_iterator(cx.device.I2C0, i2c_sda, i2c_scl, &mut resets, 0x69);

        let i2c_scl = pins.gpio15.into_mode::<FunctionI2C>();
        let i2c_sda = pins.gpio14.into_mode::<FunctionI2C>();

        let i2c1 = I2C::new_controller(
            cx.device.I2C1,
            i2c_sda,
            i2c_scl,
            400u32.kHz(),
            &mut resets,
            clocks.system_clock.freq(),
        );

        static I2C_BYTES_QUEUE: static_cell::StaticCell<Queue<u8, 32>> =
            static_cell::StaticCell::new();
        let i2c_bytes_queue = I2C_BYTES_QUEUE.init(Queue::new());
        let (i2c0_bytes_tx, i2c0_bytes_rx) = i2c_bytes_queue.split();

        let (mut pio, sm0, _, _, _) = cx.device.PIO0.split(&mut resets);
        let neopixel = ws2812_pio::Ws2812Direct::new(
            pins.gpio13.into_mode(),
            &mut pio,
            sm0,
            clocks.peripheral_clock.freq(),
        );

        type A = impl Sized;
        static NEOPIXEL_SHELF: static_cell::StaticCell<A> = static_cell::StaticCell::new();
        let shelf = NEOPIXEL_SHELF.init_with(Shelf::new);
        type B = impl genawaiter::Generator<(), Yield = crate::WaitType, Return = ()>;
        static MORSE_GEN: static_cell::StaticCell<B> = static_cell::StaticCell::new();
        let neopixel_gen = unsafe {
            TotallySend(core::pin::Pin::new_unchecked(
                MORSE_GEN.init(genawaiter::stack::Gen::new(shelf, move |co| {
                    morse_state(co, i2c0_bytes_rx, neopixel)
                }))
                    as &mut dyn genawaiter::Generator<(), Yield = crate::WaitType, Return = ()>,
            ))
        };

        defmt::info!("Setting up usb bus");

        type U = impl Sized;
        static USB_BUS: static_cell::StaticCell<U> = static_cell::StaticCell::new();
        let usb_bus = &*USB_BUS.init(UsbBusAllocator::new(hal::usb::UsbBus::new(
            cx.device.USBCTRL_REGS,
            cx.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        )));

        let serial = &*luhack_badge::forever!(RefCell::new(usbd_serial::SerialPort::new(usb_bus)));

        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x6969, 0x1234))
            .manufacturer("Wardrobe Emporium")
            .product("Gnoblin Deez")
            .serial_number("Nuts")
            .device_class(2)
            .build();

        // type A = impl Sized;
        // static USB_SHELF: static_cell::StaticCell<A> = static_cell::StaticCell::new();
        let shelf = luhack_badge::forever!(Shelf::new());
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

        let _ = morse_task::spawn(WakeReason::Time);

        (
            Shared { delay },
            Local {
                led,
                neopixel_gen,
                usb_gen,
                i2c0,
                i2c1,
                i2c0_bytes_tx,
                usb_dev,
                serial: TotallySend(serial),
            },
            init::Monotonics(timer),
        )
    }

    #[derive(Clone, Copy)]
    pub enum WakeReason {
        Rx,
        Time,
    }

    #[derive(Clone, Copy)]
    pub enum StopReason {
        Rx,
        Time,
    }

    #[task(priority = 2, local = [neopixel_gen, reason: Option<StopReason> = None], capacity = 3)]
    fn morse_task(cx: morse_task::Context, wake_reason: WakeReason) {
        defmt::trace!("morse task tick");
        let r = match cx.local.reason {
            Some(r) => match (r, wake_reason) {
                (StopReason::Rx, WakeReason::Rx) => cx.local.neopixel_gen.0.as_mut().resume(()),
                (StopReason::Rx, WakeReason::Time) => {
                    defmt::error!("morse task was woken for time while it is waitinng for rx");
                    return;
                }
                (StopReason::Time, WakeReason::Rx) => {
                    defmt::trace!("morse task woken for rx while waiting for time");
                    return;
                }
                (StopReason::Time, WakeReason::Time) => cx.local.neopixel_gen.0.as_mut().resume(()),
            },
            None => cx.local.neopixel_gen.0.as_mut().resume(()),
        };

        let r = match r {
            genawaiter::GeneratorState::Yielded(v) => v,
            genawaiter::GeneratorState::Complete(()) => {
                defmt::error!("morse task generator completed?");
                return;
            }
        };

        match r {
            crate::WaitType::UntilRX => {
                defmt::info!("Morse task going to sleep until rx");
                *cx.local.reason = Some(StopReason::Rx);
            }
            crate::WaitType::Duration(d) => {
                defmt::info!("Morse task going to sleep until time");
                *cx.local.reason = Some(StopReason::Time);
                let _ = morse_task::spawn_after(d, WakeReason::Time);
            }
        }
    }

    #[task(priority = 1, local = [i2c1])]
    fn send_unlock(cx: send_unlock::Context) {
        cx.local
            .i2c1
            .write(0x69, &luhack_badge::encrypt!(b"open sesame"));
        defmt::info!("Sent unlock command");
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

    #[task(binds = I2C0_IRQ, local = [i2c0, i2c0_bytes_tx], priority = 3)]
    fn wake_i2c0(cx: wake_i2c0::Context) {
        defmt::trace!("i2c0 tick");

        let x = cx.local.i2c0.next();
        unsafe {
            let x = (*hal::pac::I2C0::ptr()).ic_intr_stat.read();
            if x.r_tx_empty().is_active() {
                (*hal::pac::I2C0::ptr())
                    .ic_intr_mask
                    .write(|w| w.m_tx_empty().enabled());
            }
        };

        let mut should_wake = false;
        loop {
            let mut buf = [0u8; 1];
            if cx.local.i2c0.read(&mut buf) == 1 {
                should_wake = true;
                // discard bytes that don't fit, we'll recover eventually anyway
                let _ = cx.local.i2c0_bytes_tx.enqueue(buf[0]);
            } else {
                break;
            }
        }

        if should_wake {
            defmt::info!("i2c received some data");
            let _ = morse_task::spawn(WakeReason::Rx);
        }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            defmt::trace!("sleep");
            cortex_m::asm::wfi();
            defmt::trace!("wake");
        }
    }
}
