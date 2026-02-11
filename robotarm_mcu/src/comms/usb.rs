use defmt::error;
use embassy_futures::join::join;
use embassy_sync::pipe::Pipe;
use embassy_usb::{
    Builder, Config, UsbDevice,
    class::cdc_acm::{CdcAcmClass, Receiver, Sender, State},
    driver::Driver,
};
use static_cell::StaticCell;

#[derive(defmt::Format)]
pub enum UsbMessage {
    Empty,
    Bytes([u8; 128]),
}

pub struct UsbMonitor {
    class: CdcAcmClass<'static, embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>>,
    buf: [u8; 128],
}

impl UsbMonitor {
    pub fn new(
        spawner: &embassy_executor::Spawner,
        driver: embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>,
    ) -> Self {
        // let driver = embassy_rp::usb::Driver::new(p.USB, Irqs);

        // Create embassy-usb Config
        let config = {
            let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
            config.manufacturer = Some("Embassy");
            config.product = Some("Embassy Serial Logger");
            config.serial_number = Some("12345678");
            config.max_power = 100;
            config.max_packet_size_0 = 64;
            config
        };

        // Create embassy-usb DeviceBuilder using the driver and config.
        // It needs some buffers for building the descriptors.
        let mut builder = {
            static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
            let builder = embassy_usb::Builder::new(
                driver,
                config,
                CONFIG_DESCRIPTOR.init([0; 256]),
                BOS_DESCRIPTOR.init([0; 256]),
                &mut [], // no msos descriptors
                CONTROL_BUF.init([0; 64]),
            );
            builder
        };

        // Create classes on the builder.
        let mut class = {
            static STATE: StaticCell<embassy_usb::class::cdc_acm::State<'static>> =
                StaticCell::new();
            let state = STATE.init(embassy_usb::class::cdc_acm::State::new());
            embassy_usb::class::cdc_acm::CdcAcmClass::new(&mut builder, state, 64)
        };

        let mut out = Self {
            class,
            buf: [0; 128],
        };

        let usb = builder.build();

        spawner.spawn(usb_task(usb)).unwrap();

        out
    }

    pub async fn wait_connection(&mut self) {
        self.class.wait_connection().await;
    }

    // pub async fn wait_read(&mut self, buf: &mut [u8]) -> usize {
    //     self.class.read_packet(buf).await.unwrap_or(0)
    // }

    pub async fn send(&mut self, msg: robotarm_protocol::SerialLogMessage) {
        if let Ok(encoded) = postcard::to_slice_cobs(&msg, &mut self.buf) {
            let _ = self.class.write_packet(encoded).await;
        } else {
            error!("Failed to encode message");
        }
    }

    pub async fn _send(&mut self, data: &[u8]) {
        let _ = self.class.write_packet(data).await;
        // let _ = self.class.write_packet(b"\n").await;
    }
}

#[cfg(feature = "nope")]
impl UsbMonitor {
    pub fn new(
        rx: embassy_sync::channel::Channel<
            embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
            UsbMessage,
            1,
        >,
    ) -> Self {
        Self { rx }
    }

    pub async fn run(
        &mut self,
        spawner: &embassy_executor::Spawner,
        irqs: crate::Irqs,
        usb: embassy_rp::Peri<'static, embassy_rp::peripherals::USB>,
    ) -> ! {
        let driver = embassy_rp::usb::Driver::new(usb, irqs);

        // Create embassy-usb Config
        let config = {
            let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
            config.manufacturer = Some("Embassy");
            config.product = Some("Embassy Serial Logger");
            config.serial_number = Some("12345678");
            config.max_power = 100;
            config.max_packet_size_0 = 64;
            config
        };

        // Create embassy-usb DeviceBuilder using the driver and config.
        // It needs some buffers for building the descriptors.
        let mut builder = {
            static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
            static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
            let builder = embassy_usb::Builder::new(
                driver,
                config,
                CONFIG_DESCRIPTOR.init([0; 256]),
                BOS_DESCRIPTOR.init([0; 256]),
                &mut [], // no msos descriptors
                CONTROL_BUF.init([0; 64]),
            );
            builder
        };

        // Create classes on the builder.
        let mut class = {
            static STATE: StaticCell<embassy_usb::class::cdc_acm::State> = StaticCell::new();
            let state = STATE.init(embassy_usb::class::cdc_acm::State::new());
            embassy_usb::class::cdc_acm::CdcAcmClass::new(&mut builder, state, 64)
        };

        // Build the builder.
        let usb = builder.build();

        // Run the USB device.
        spawner.spawn(usb_task(usb)).unwrap();

        loop {
            let msg = self.rx.receive().await;
            match msg {
                UsbMessage::Empty => (),
                UsbMessage::Bytes(bytes) => {
                    // Process the received bytes
                    defmt::info!("Received USB data: {:?}", bytes);
                    // self.
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn usb_task(
    mut usb: UsbDevice<'static, embassy_rp::usb::Driver<'static, embassy_rp::peripherals::USB>>,
) -> ! {
    usb.run().await
}

#[cfg(feature = "nope")]
mod prev {
    type CS = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

    /// A trait that can be implemented and then passed to the
    pub trait ReceiverHandler {
        /// Data comes in from the serial port with each command and runs this function
        fn handle_data(&self, data: &[u8]) -> impl Future<Output = ()> + Send;

        /// Create a new instance of the Handler
        fn new() -> Self;
    }

    /// The logger state containing buffers that must live as long as the USB peripheral.
    pub struct LoggerState<'d> {
        state: State<'d>,
        config_descriptor: [u8; 128],
        bos_descriptor: [u8; 16],
        msos_descriptor: [u8; 256],
        control_buf: [u8; 64],
    }

    impl<'d> LoggerState<'d> {
        /// Create a new instance of the logger state.
        pub fn new() -> Self {
            Self {
                state: State::new(),
                config_descriptor: [0; 128],
                bos_descriptor: [0; 16],
                msos_descriptor: [0; 256],
                control_buf: [0; 64],
            }
        }
    }

    /// The packet size used in the usb logger, to be used with `create_future_from_class`
    pub const MAX_PACKET_SIZE: u8 = 64;

    /// The logger handle, which contains a pipe with configurable size for buffering log messages.
    pub struct UsbLogger<const N: usize, T: ReceiverHandler + Send + Sync> {
        buffer: Pipe<CS, N>,
        // custom_style: Option<fn(&Record, &mut Writer<'_, N>) -> ()>,
        recieve_handler: Option<T>,
    }

    impl<const N: usize, T: ReceiverHandler + Send + Sync> UsbLogger<N, T> {
        /// Create a new logger instance.
        pub const fn new() -> Self {
            Self {
                buffer: Pipe::new(),
                recieve_handler: None,
            }
        }

        /// Run the USB logger using the state and USB driver. Never returns.
        pub async fn run<'d, D>(&'d self, state: &'d mut LoggerState<'d>, driver: D) -> !
        where
            D: Driver<'d>,
            Self: 'd,
        {
            let mut config = Config::new(0xc0de, 0xcafe);
            config.manufacturer = Some("Embassy");
            config.product = Some("USB-serial logger");
            config.serial_number = None;
            config.max_power = 100;
            config.max_packet_size_0 = MAX_PACKET_SIZE;

            let mut builder = Builder::new(
                driver,
                config,
                &mut state.config_descriptor,
                &mut state.bos_descriptor,
                &mut state.msos_descriptor,
                &mut state.control_buf,
            );

            // Create classes on the builder.
            let class = CdcAcmClass::new(&mut builder, &mut state.state, MAX_PACKET_SIZE as u16);
            let (mut sender, mut receiver) = class.split();

            // Build the builder.
            let mut device = builder.build();
            loop {
                let run_fut = device.run();
                // let class_fut = self.run_logger_class(&mut sender, &mut receiver);
                // join(run_fut, class_fut).await;
            }
        }
    }

    #[cfg(feature = "nope")]
    impl<const N: usize, T: ReceiverHandler + Send + Sync> UsbLogger<N, T> {
        /// Create a new logger instance.
        pub const fn new() -> Self {
            Self {
                buffer: Pipe::new(),
                recieve_handler: None,
            }
        }

        /// Add a command handler to the logger
        pub fn with_handler(&mut self, handler: T) {
            self.recieve_handler = Some(handler);
        }

        /// Run the USB logger using the state and USB driver. Never returns.
        pub async fn run<'d, D>(&'d self, state: &'d mut LoggerState<'d>, driver: D) -> !
        where
            D: Driver<'d>,
            Self: 'd,
        {
            let mut config = Config::new(0xc0de, 0xcafe);
            config.manufacturer = Some("Embassy");
            config.product = Some("USB-serial logger");
            config.serial_number = None;
            config.max_power = 100;
            config.max_packet_size_0 = MAX_PACKET_SIZE;

            let mut builder = Builder::new(
                driver,
                config,
                &mut state.config_descriptor,
                &mut state.bos_descriptor,
                &mut state.msos_descriptor,
                &mut state.control_buf,
            );

            // Create classes on the builder.
            let class = CdcAcmClass::new(&mut builder, &mut state.state, MAX_PACKET_SIZE as u16);
            let (mut sender, mut receiver) = class.split();

            // Build the builder.
            let mut device = builder.build();
            loop {
                let run_fut = device.run();
                let class_fut = self.run_logger_class(&mut sender, &mut receiver);
                join(run_fut, class_fut).await;
            }
        }

        async fn run_logger_class<'d, D>(
            &self,
            sender: &mut Sender<'d, D>,
            receiver: &mut Receiver<'d, D>,
        ) where
            D: Driver<'d>,
        {
            let log_fut = async {
                let mut rx: [u8; MAX_PACKET_SIZE as usize] = [0; MAX_PACKET_SIZE as usize];
                sender.wait_connection().await;
                loop {
                    let len = self.buffer.read(&mut rx[..]).await;
                    let _ = sender.write_packet(&rx[..len]).await;
                    if len as u8 == MAX_PACKET_SIZE {
                        let _ = sender.write_packet(&[]).await;
                    }
                }
            };
            let reciever_fut = async {
                let mut reciever_buf: [u8; MAX_PACKET_SIZE as usize] =
                    [0; MAX_PACKET_SIZE as usize];
                receiver.wait_connection().await;
                loop {
                    let n = receiver.read_packet(&mut reciever_buf).await.unwrap();
                    match &self.recieve_handler {
                        Some(handler) => {
                            let data = &reciever_buf[..n];
                            handler.handle_data(data).await;
                        }
                        None => (),
                    }
                }
            };

            join(log_fut, reciever_fut).await;
        }
    }

    /// A writer that writes to the USB logger buffer.
    pub struct Writer<'d, const N: usize>(&'d Pipe<CS, N>);

    impl<'d, const N: usize> core::fmt::Write for Writer<'d, N> {
        fn write_str(&mut self, s: &str) -> Result<(), core::fmt::Error> {
            // The Pipe is implemented in such way that we cannot
            // write across the wraparound discontinuity.
            let b = s.as_bytes();
            if let Ok(n) = self.0.try_write(b) {
                if n < b.len() {
                    // We wrote some data but not all, attempt again
                    // as the reason might be a wraparound in the
                    // ring buffer, which resolves on second attempt.
                    let _ = self.0.try_write(&b[n..]);
                }
            }
            Ok(())
        }
    }
}
