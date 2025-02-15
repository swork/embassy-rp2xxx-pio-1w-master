//! Support for 1Wire devices in the DS18 family of temperature sensors.
//!
//! | Device      | Code | Tested | Notes                                     |
//! | ----------- | ---- | ------ | ----------------------------------------- |
//! | DS18S20     |  10  |        |                                           |
//! | DS18S20-PAR |  10  |        |                                           |
//! | DS1825      |  3b  |        |                                           |
//! | DS28EA00    |  42  |        | CHAIN and GPIO are TODO                   |
//! | MAX30207    |  54  |        |                                           |
//! | MAX31888    |  54  |        |                                           |
//! | MAX31825    |  3b  |        |                                           |
//! | MAX31820    |  28  |        |                                           |
//! | MAX31820PAR |  28  |        |                                           |
//! | MAX31850    |  3b  |        |                                           |
//! | MAX31826    |  3b  |        |                                           |
//!
//! "Tested" above means that functionality listed in the corresponding
//! datasheet is represented by test functions in this crate's `tests/`
//! directory, and that those tests pass when run on-chip. Tests are not
//! comprehensive.

/// Temperature sensor devices and their family code
pub struct TemperatureSensorFamilyCodes {}
impl TemperatureSensorFamilyCodes {
    const DS18S20: u8 = b'\x10';
    const DS18S20PAR: u8 = b'\x10';
    const DS18B20: u8 = b'\x28';
    const MAX31820: u8 = b'\x28';
    const MAX31820PAR: u8 = b'\x28';
    const DS1825: u8 = b'\x3b';
    const MAX31825: u8 = b'\x3b';
    const MAX31850: u8 = b'\x3b';
    const MAX31826: u8 = b'\x3b';
    const DS28EA00: u8 = b'\x42';
    const MAX30207: u8 = b'\x54';
    const MAX31888: u8 = b'\x54';
}

/// Family codes
pub enum TemperatureSensorFamily {
    OneZero,
    TwoEight,
    ThreeBravo,
    FourTwo,
    FiveFour,
}
impl TemperatureSensorFamily {
    pub fn from_code(code: u8) -> Result<TemperatureSensorFamily, ()> {
        let r = if code == TemperatureSensorFamilyCodes::DS18S20
            || code == TemperatureSensorFamilyCodes::DS18S20PAR
        {
            Ok(TemperatureSensorFamily::OneZero)
        } else if code == TemperatureSensorFamilyCodes::MAX31820
            || code == TemperatureSensorFamilyCodes::MAX31820PAR
            || code == TemperatureSensorFamilyCodes::DS18B20
        {
            Ok(TemperatureSensorFamily::TwoEight)
        } else if code == TemperatureSensorFamilyCodes::DS1825
            || code == TemperatureSensorFamilyCodes::MAX31825
            || code == TemperatureSensorFamilyCodes::MAX31850
            || code == TemperatureSensorFamilyCodes::MAX31826
        {
            Ok(TemperatureSensorFamily::ThreeBravo)
        } else if code == TemperatureSensorFamilyCodes::DS28EA00 {
            Ok(TemperatureSensorFamily::FourTwo)
        } else if code == TemperatureSensorFamilyCodes::MAX30207
            || code == TemperatureSensorFamilyCodes::MAX31888
        {
            Ok(TemperatureSensorFamily::FiveFour)
        } else {
            Err(())
        };
        r
    }
}

/// Reporting precision. Not all precisions are available at
/// all devices.
pub enum Precision {
    Nine,
    Ten,
    Eleven,
    Twelve,
    Sixteen,
}

pub struct Ds18 {}
impl Ds18 {
    pub fn t_conv(family: &u8, precision: &Precision) -> Result<usize, ()> {
        if *family == b'\x10' {
            match *precision {
                crate::devices::Precision::Nine
                | crate::devices::Precision::Ten
                | crate::devices::Precision::Eleven
                | crate::devices::Precision::Twelve => Ok(750),
                _ => Err(()),
            }
        } else if *family == b'\x28' {
            match *precision {
                crate::devices::Precision::Nine => Ok(94),
                crate::devices::Precision::Ten => Ok(188),
                crate::devices::Precision::Eleven => Ok(375),
                crate::devices::Precision::Twelve => Ok(750),
                _ => Err(()),
            }
        } else {
            Err(())
        }
        // many more to look up. TODO how to handle no-spec items, like DS1825
        // outside 10-bit mode? Clearly driver is expected to poll, but what's
        // an optimal period?
    }
}
