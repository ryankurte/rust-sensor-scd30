
//#![no_std]

use core::fmt::Debug;
use core::marker::PhantomData;

extern crate embedded_hal;
use embedded_hal::blocking::i2c;


/// Sdc30 sensor object
/// This is generic over an I2C connector and associated error type
pub struct Sdc30<Conn, Err> {
    conn: Conn,
    _err: PhantomData<Err>,
}

/// Sdc30 default I2C address
pub const DEFAULT_ADDRESS: u8 = 0x61;

/// Sdc30 error object
#[derive(PartialEq, Clone, Debug)]
pub enum Error<ConnErr> {
    Conn(ConnErr),
    Crc(u8, u8),
}

/// Sdc30 measurement object
#[derive(PartialEq, Clone, Debug)]
pub struct Measurement {
    /// CO2 concentration in parts-per-million (PPM)
    /// Range: 0 - 10,000
    pub co2: f32,
    /// Temperature in degrees celsius
    /// Range: -40 - 125 C
    pub temp: f32,
    /// Relative Humidity (%)
    /// Range: 0 - 100
    pub rh: f32,
}

/// Sdc30 I2C Command
/// Command and data are big endian 16-bit unsigned integers, all Command with data are followed by a CRC-8 checksum
#[derive(PartialEq, Clone, Debug)]
pub enum Command {
    /// Start continuous mode
    /// Data is a u16 representing pressure in mBar for compensation
    /// or zero for no pressure compensation
    StartContinuousMode = 0x0010,
    /// Stop continuous mode
    /// No associated data, no CRC
    StopContinuousMode = 0x0104,
    /// Set interval for continuous measurement mode
    /// Data is a u16 in seconds between 2 and 1800
    SetMeasurementInterval = 0x4600,

    /// Fetch data ready status
    /// This returns 1 if data is available in the buffer, 0 otherwise
    GetDataReady = 0x0202,

    /// Read a measurement from the buffer
    ReadMeasurement = 0x0300,

    /// Enable or Disable Automatic Self Calibration (ASC)
    /// Data is a u16, 1 enables ASC and 0 disables ASC
    SetAfc = 0x5306,

    /// Set Forced Recalibration Value (FRC)
    /// This is used to compensate for sensor drift when a CO2 reference value is available
    /// Data is a u16 CO2 concentration in ppm
    SetFrc = 0x5204,

    /// Set temperature offset
    /// Data is a uint16 in degrees celsius * 100, ie. 43 degrees -> 430u16
    SetTempOffset = 0x5403,
}


impl <Conn, Err> Sdc30 <Conn, Err> where
    Conn: i2c::Read + i2c::Write + i2c::WriteRead,
    Err: PartialEq + Clone + Debug,
{
    /// Create a new Sdc30 sensor instance
    pub fn new(conn: Conn) -> Result<Self, Error<Err>> {
        // Create sensor object
        let mut s = Sdc30{ conn, _err: PhantomData };

        // TODO: check communication

        // Return sensor
        Ok(s)
    }

    pub fn start_continuous(&mut self, pressure_compensation: u16) -> Result<Measurement, Error<Err>> {
        unimplemented!()
    }

    pub fn stop_continuous(&mut self) -> Result<Measurement, Error<Err>> {
        unimplemented!()
    }

    pub fn set_measurement_interval(&mut self, interval: u16) -> Result<Measurement, Error<Err>> {
        unimplemented!()
    }

    /// Enable or disable Automatic Self-Calibration
    pub fn set_afc(&mut self, enabled: bool) -> Result<Measurement, Error<Err>> {
        unimplemented!()
    }


    pub fn data_ready(&mut self) -> Result<bool, Error<Err>> {
        unimplemented!()
    }

    pub fn read_data(&mut self) -> Result<Measurement, Error<Err>> {
        unimplemented!()
    }


    /// Convert from a 6-byte response line into an F32 value
    fn convert(line: &[u8]) -> Result<f32, Error<Err>> {
        // Lines MUST be 6 bytes long (MMSB, MLSB, CRC, LMSB, LLSB, CRC)
        assert_eq!(line.len(), 6);

        // Check CRC
        let crc1 = Self::crc(&line[0..2]);
        if crc1 != line[2] {
            return Err(Error::Crc(crc1, line[2]));
        }

        let crc2 = Self::crc(&line[3..5]);
        if crc2 != line[5] {
            return Err(Error::Crc(crc2, line[5]));
        }
        
        // Build temporary u32
        // Note the returned data is _big endian_
        let u: u32 = ((line[0] as u32) << 24 )| ((line[1] as u32) << 16 )
            | ((line[3] as u32) << 8) | ((line[4] as u32) << 0);

        // Transmute into float
        let v = f32::from_bits(u);

        Ok(v)
    }


    fn crc(data: &[u8]) -> u8 {
        let poly = 0x31;
        let init = 0xff;
        let xor = 0x00;

        let mut crc = init;

        // For each byte
        for v in data {
            // XOR with current byte
            crc ^= v;

            // For each bit (in -ve order)
            for bit in (0..8).rev() {
                if crc & 0x80 != 0 {
                    crc = (crc << 1) ^ poly;
                } else {
                    crc = (crc << 1);
                }
            }
        }

        // Apply final xor    
        crc ^= xor;

        crc
    }
}

/// Base API for reading and writing to the device
/// This should not be required by consumers, but is exposed to support alternate use
pub trait Base<Err> {
    fn write_command(&mut self, command: Command, data: &[u8]) -> Result<(), Error<Err>>;
    fn read_command(&mut self, command: Command, data: &mut [u8]) -> Result<usize, Error<Err>>;
}

impl <Conn, Err> Base<Err> for Sdc30 <Conn, Err> where
    Conn: i2c::Read + i2c::Write + i2c::WriteRead,
    Err: PartialEq + Clone + Debug,
{
    fn write_command(&mut self, command: Command, data: &[u8]) -> Result<(), Error<Err>> {
        unimplemented!()
    }
    fn read_command(&mut self, command: Command, data: &mut [u8]) -> Result<usize, Error<Err>> {
        unimplemented!()
    }
}

#[cfg(test)]
mod test {
    extern crate std;
    use std::vec;

    extern crate embedded_hal;
    use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

    extern crate embedded_hal_mock;
    use embedded_hal_mock::MockError;
    use embedded_hal_mock::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    extern crate assert_approx_eq;
    use assert_approx_eq::assert_approx_eq;

    use super::*;

    #[test]
    fn test_start_continuous() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(0xc2, vec![0x00, 0x10, 0x03, 0xf8, 0x81]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30::<I2cMock, MockError>::new(i2c.clone()).unwrap();

        // Start continuous mode
        sensor.start_continuous(1016u16).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_stop_continuous() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(0xc2, vec![0x01, 0x07]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30::<I2cMock, MockError>::new(i2c.clone()).unwrap();

        // Stop continuous mode
        sensor.stop_continuous().unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_set_measurement_interval() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(0xc2, vec![0x46, 0x00, 0x00, 0x02, 0xE3]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30::<I2cMock, MockError>::new(i2c.clone()).unwrap();

        // Set measurement interval
        sensor.set_measurement_interval(2).unwrap();

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_read_data_ready() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(0xc2, vec![0x02, 0x02]),
            I2cTransaction::read(0xc3, vec![0x01, 0xb0]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30::<I2cMock, MockError>::new(i2c.clone()).unwrap();

        // Read data ready
        let ready = sensor.data_ready().unwrap();
        assert!(!ready);

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_read_measurement() {
        // Set up expectations
        let expectations = [
            I2cTransaction::write(0xc2, vec![0x03, 0x00]),
            I2cTransaction::read(0xc3, vec![
                0x43, 0xDB, 0xCB, 0x8C, 0x2E, 0x8F, // CO2: 439 ppm
                0x41, 0xD9, 0x70, 0xE7, 0xFF, 0xF5, // Temperature: 27.2 C
                0x42, 0x43, 0xBF, 0x3A, 0x1B, 0x74, // Relative humidity, 48.8 %
            ]),
        ];
        let mut i2c = I2cMock::new(&expectations);

        // Create sensor object
        let mut sensor = Sdc30::<I2cMock, MockError>::new(i2c.clone()).unwrap();

        // Read measurement
        let measurement = sensor.read_data().unwrap();
        assert_eq!(measurement, Measurement{co2: 439.0, temp: 27.2, rh: 48.8 });

        // Finalize expectations
        i2c.done();
    }

    #[test]
    fn test_crc() {
        // Test vectors from datasheet
        let tests = &[
            ([0xbe, 0xef], 0x92),
            ([0x00, 0x00], 0x81),
            ([0x43, 0xDB], 0xCB),
        ];

        for t in tests {
            let v = Sdc30::<I2cMock, MockError>::crc(&t.0);
            assert_eq!(v, t.1);
        }
        
    }

    #[test]
    fn test_convert() {
        // Test vectors from datasheet
        let tests = &[
            ([0x43, 0xDB, 0xCB, 0x8C, 0x2E, 0x8F], 439.0),
            ([0x41, 0xD9, 0x70, 0xE7, 0xFF, 0xF5], 27.2),
            ([0x42, 0x43, 0xBF, 0x3A, 0x1B, 0x74], 48.8),
        ];

        for t in tests {
            let v = Sdc30::<I2cMock, MockError>::convert(&t.0).unwrap();
            assert_approx_eq!(v, t.1, 0.1);
        }
    }
}