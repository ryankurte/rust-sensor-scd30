//! Sdc30 command-line utility
//! 
//! Copyright 2019 Ryan Kurte

extern crate embedded_hal;
extern crate linux_embedded_hal;
use linux_embedded_hal::I2cdev;

extern crate structopt;
use structopt::StructOpt;

extern crate humantime;
use humantime::{Duration as HumanDuration};

#[macro_use] extern crate log;
extern crate simplelog;
use simplelog::{TermLogger, LevelFilter};

extern crate sensor_sdc30;
use sensor_sdc30::Sdc30;

#[derive(StructOpt)]
#[structopt(name = "sdc30-util")]
/// A Command Line Interface (CLI) for interacting with a local Sdc30 environmental sensor over I2C
pub struct Options {

    /// Specify the i2c interface to use to connect to the sdc30 device
    #[structopt(short="d", long = "i2c", default_value = "/dev/i2c-1", env = "SDC30_I2C")]
    i2c: String,

    /// Specify period for taking measurements
    #[structopt(short = "p", long = "sample-period", default_value="10s")]
    pub period: HumanDuration,

    /// Delay between sensor poll operations
    #[structopt(long = "poll-delay", default_value="100ms")]
    pub poll_delay: HumanDuration,

    /// Enable verbose logging
    #[structopt(long = "log-level", default_value = "info")]
    level: LevelFilter,
}

fn main() {
    // Load options
    let opts = Options::from_args();

    // Setup logging
    TermLogger::init(opts.level, simplelog::Config::default()).unwrap();

    debug!("Connecting to I2C device");
    let i2c = match I2cdev::new(&opts.i2c) {
        Ok(v) => v,
        Err(e) => {
            error!("Error opening I2C device '{}': {:?}", &opts.i2c, e);
            std::process::exit(-1);
        }
    };

    debug!("Connecting to SDC30");
    let mut sensor = match Sdc30::new(i2c) {
        Ok(v) => v,
        Err(e) => {
            error!("Error connecting to SDC30: {:?}", e);
            std::process::exit(-2);
        }
    };

    debug!("Starting sensor polling");
    if let Err(e) = sensor.start_continuous(opts.period.as_secs() as u16) {
        error!("Error starting continuous mode: {:?}", e);
        std::process::exit(-3);
    }

    debug!("Waiting for sensor to initialise");
    std::thread::sleep(*opts.period);

    loop {
        debug!("Starting sensor read cycle");

        // Poll for sensor ready
        let mut ready = false;
        for _i in 0..100 {
            match sensor.data_ready() {
                Ok(true) => {
                    ready = true;
                    break;
                },
                Ok(false) => {
                    std::thread::sleep(*opts.poll_delay);
                }
                Err(e) => {
                    error!("Error polling for sensor ready: {:?}", e);
                    std::process::exit(-4);
                }
            };
        }

        debug!("Sensor data ready state: {:?}", ready);

        // If we're ready, attempt to read the data
        match (ready, sensor.read_data()) {
            (false, _) => (),
            (true, Ok(m)) => {
                info!("New measurement: {:?}", m);
                break;
            },
            (true, Err(e)) => {
                error!("Error reading sensor data: {:?}", e);
                std::process::exit(-5);
            },
        };

        // Wait for enough time for another sensor reading
        std::thread::sleep(*opts.period);
    }
}
