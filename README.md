# Rust Sensor SDC30

A rust driver (and CLI utility) for the [sdc30](https://www.sensirion.com/en/environmental-sensors/carbon-dioxide-sensors-co2/) CO2, Temperature and Humidity sensor.

## Status

[![GitHub tag](https://img.shields.io/github/tag/ryankurte/rust-sensor-sdc30.svg)](https://github.com/ryankurte/rust-sensor-sdc30)
[![Travis Build Status](https://travis-ci.com/ryankurte/rust-sensor-sdc30.svg?branch=master)](https://travis-ci.com/ryankurte/rust-sensor-sdc30)
[![Crates.io](https://img.shields.io/crates/v/sensor-sdc30.svg)](https://crates.io/crates/sensor-sdc30)
[![Docs.rs](https://docs.rs/sensor-sdc30/badge.svg)](https://docs.rs/sensor-sdc30)

[Open Issues](https://github.com/ryankurte/rust-sensor-sdc30/issues)

## Usage

Add the library to your project with `cargo add sensor-sdc30` or with `sensor-sdc30 = { version = "0.1.0", features = [] }` in your `Cargo.toml`.

Install the utility with one of the following methods:

- using a precompiled binary from the [releases](https://github.com/ryankurte/rust-sensor-sdc30/releases/) page
- from source using cargo with `cargo install sensor-sdc30`

Run `sdc30-util` to communicate with the sensor.

```
pi@raspberrypi:~ $ sudo ./sdc30-util --help
sdc30-util 0.1.0
Ryan Kurte <ryankurte@gmail.com>
A Command Line Interface (CLI) for interacting with a local Sdc30 environmental sensor over I2C

USAGE:
    sdc30-util [OPTIONS]

FLAGS:
    -h, --help       Prints help information
    -V, --version    Prints version information

OPTIONS:
    -d, --i2c <i2c>                  Specify the i2c interface to use to connect to the sdc30 device [env: SDC30_I2C=]
                                     [default: /dev/i2c-1]
        --log-level <level>          Enable verbose logging [default: info]
    -p, --sample-period <period>     Specify period for taking measurements [default: 10s]
        --poll-delay <poll_delay>    Delay between sensor poll operations [default: 100ms]
```
