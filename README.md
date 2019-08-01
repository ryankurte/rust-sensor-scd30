# Rust Sensor SCD30

A rust driver (and CLI utility) for the [scd30](https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors-co2/) CO2, Temperature and Humidity sensor.

## Status

[![GitHub tag](https://img.shields.io/github/tag/ryankurte/rust-sensor-scd30.svg)](https://github.com/ryankurte/rust-sensor-scd30)
[![Travis Build Status](https://travis-ci.com/ryankurte/rust-sensor-scd30.svg?branch=master)](https://travis-ci.com/ryankurte/rust-sensor-scd30)
[![Crates.io](https://img.shields.io/crates/v/sensor-scd30.svg)](https://crates.io/crates/sensor-scd30)
[![Docs.rs](https://docs.rs/sensor-scd30/badge.svg)](https://docs.rs/sensor-scd30)

[Open Issues](https://github.com/ryankurte/rust-sensor-scd30/issues)

## Usage

Add the library to your project with `cargo add sensor-scd30` or with `sensor-scd30 = { version = "0.1.0", features = [] }` in your `Cargo.toml`.

Install the utility with one of the following methods:

- using a precompiled binary from the [releases](https://github.com/ryankurte/rust-sensor-scd30/releases/) page
- from source using cargo with `cargo install sensor-scd30`

Run `scd30-util` to communicate with the sensor.

Help:
```
pi@raspberrypi:~ $ sudo ./scd30-util --help
scd30-util 0.1.0
Ryan Kurte <ryankurte@gmail.com>
A Command Line Interface (CLI) for interacting with a local Scd30 environmental sensor over I2C

USAGE:
    scd30-util [OPTIONS]

FLAGS:
    -h, --help       Prints help information
    -V, --version    Prints version information

OPTIONS:
    -d, --i2c <i2c>                  Specify the i2c interface to use to connect to the scd30 device [env: SCD30_I2C=]
                                     [default: /dev/i2c-1]
        --log-level <level>          Enable verbose logging [default: info]
    -p, --sample-period <period>     Specify period for taking measurements [default: 10s]
        --poll-delay <poll_delay>    Delay between sensor poll operations [default: 100ms]
```

Example output:
```
pi@raspberrypi:~ $ ./scd30-util -p 2s
04:01:00 [INFO] CO2: 556.21 ppm, Temperature: 19.15 C, Humidity: 49.39 %
04:01:02 [INFO] CO2: 553.72 ppm, Temperature: 19.13 C, Humidity: 49.44 %
```
