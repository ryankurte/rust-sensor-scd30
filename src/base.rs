//! Base communication implementation for interacting with Sdc30 device
//! 
//! Copyright 2019 Ryan Kurte

use core::fmt::Debug;

use embedded_hal::blocking::i2c;

use crate::{Sdc30, Error};
use crate::device::*;

/// Base API for reading and writing to the device
/// This should not be required by consumers, but is exposed to support alternate use (or in future provide ModBus support)
pub trait Base<Err> {
    /// Write a command to the device with optional data
    fn write_command(&mut self, command: Command, data: Option<u16>) -> Result<(), Error<Err>>;
    /// Read information from the device
    fn read_command(&mut self, command: Command, data: &mut [u8]) -> Result<(), Error<Err>>;
}

impl <Conn, Err> Base<Err> for Sdc30 <Conn, Err> where
    Conn: i2c::Read<Error=Err> + i2c::Write<Error=Err> + i2c::WriteRead<Error=Err>,
    Err: Debug,
{
    fn write_command(&mut self, command: Command, data: Option<u16>) -> Result<(), Error<Err>> {
        let c = command as u16;

        let mut buff: [u8; 5] = [
            (c >> 8) as u8,
            (c & 0xFF) as u8,
            0,
            0,
            0,
        ];

        let len = match data {
            Some(d) => {
                buff[2] = (d >> 8) as u8;
                buff[3] = (d & 0xFF) as u8;
                buff[4] = Self::crc(&buff[2..4]);
                5
            },
            None => 2,
        };

        trace!("Writing command: {:?} data: {:?}", c, data);

        self.conn.write(DEFAULT_ADDRESS | I2C_WRITE_FLAG, &buff[..len]).map_err(|e| Error::Conn(e) )
    }

    fn read_command(&mut self, command: Command, data: &mut [u8]) -> Result<(), Error<Err>> {
        // Write command to initialise read
        let c = command as u16;

        trace!("Writing command: {:x?}", c);

        self.conn.write(DEFAULT_ADDRESS | I2C_WRITE_FLAG, &[(c >> 8) as u8, (c & 0xFF) as u8])
            .map_err(|e| Error::Conn(e) )?;

        // Read data back
        self.conn.read(DEFAULT_ADDRESS | I2C_READ_FLAG, data)
            .map_err(|e| Error::Conn(e) )?;

        trace!("Read data: {:x?}", data);

        Ok(())
    }
}
