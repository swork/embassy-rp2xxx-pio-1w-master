/// Support for 1Wire devices in the DS18 family of temperature sensors.
///
/// | Device      | Code | Tested | Notes                                     |
/// | ----------- | ---- | ------ | ----------------------------------------- |
/// | DS18S20     |  10  |        |                                           |
/// | DS18S20-PAR |  10  |        |                                           |
/// | DS1825      |  3b  |        |                                           |
/// | DS28EA00    |  42  |        | CHAIN and GPIO are TODO                   |
/// | MAX30207    |  54  |        |                                           |
/// | MAX31888    |  54  |        |                                           |
/// | MAX31825    |  3b  |        |                                           |
/// | MAX31820    |  28  |        |                                           |
/// | MAX31820PAR |  28  |        |                                           |
/// | MAX31850    |  3b  |        |                                           |
/// | MAX31826    |  3b  |        |                                           |
///
/// "Tested" above means that functionality listed in the corresponding
/// datasheet is represented by test functions in this crate's `tests/`
/// directory, and that those tests pass when run on-chip. Tests are not
/// comprehensive.

/// Family codes
const FC_DS18S20     = b'\x10';
const FC_DS18S20-PAR = b'\x10';
const FC_MAX31820    = b'\x28';
const FC_MAX31820PAR = b'\x28';
const FC_DS1825      = b'\x3b';
const FC_MAX31825    = b'\x3b';
const FC_MAX31850    = b'\x3b';
const FC_MAX31826    = b'\x3b';
const FC_DS28EA00    = b'\x42';
const FC_MAX30207    = b'\x54';
const FC_MAX31888    = b'\x54';

struct Ds18 {
    /// The device's reporting precision. Not all precisions are available at
    /// all devices.
    enum Precision {
        Nine,
        Ten,
        Eleven,
        Twelve,
        Sixteen,
    }
}

impl Ds18 {
    pub fn t_conv(family: &u8, precision: &Precision) -> ms: Result<usize, ()> {
        match family {
            FC_DS18S20 | FC_DS18S20PAR => match precision {
                Nine | Ten | Eleven | Twelve => Ok(750),
                _ => Err(())
            },
            FC_DS1825 => match precision {
                Nine => Ok(94),
                Ten => Ok(188),
                Eleven => Ok(375),
                Twelve => Ok(750),
                _ => Err(())
            },
            _ => Err(())
        }
        // many more to look up. TODO how to handle no-spec items, like DS1825
        // outside 10-bit mode? Clearly driver is expected to poll, but what's
        // an optimal period?
    }
}

