#![no_std]
#![no_main]

use core::panic::PanicInfo;
use defmt::{Display2Format, flush, trace, debug, info, warn, error};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use heapless::Vec;
use rp2xxx_pio_1_wire_master_rs::{
    devices::TemperatureSensorFamily, OneWireMaster, OneWireMasterProgram, RomId, SearchState,
};
use defmt_rtt as _;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("Panic: {}", Display2Format(info));
    flush();
    loop {}
}

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);
    info!("\u{2501}\u{2501}\u{2501}\u{2501}\u{252b} start \u{2523}\u{2501}\u{2501}\u{2501}\u{2501}");


    let owmp = OneWireMasterProgram::<PIO0>::new(&mut common);
    let mut owm = OneWireMaster::<PIO0, 0>::new(&mut common, sm0, p.PIN_15, p.PIN_16, owmp);

    // No device attached, some line twiddling
    if false {
        // owm.blindly_set_thresholds(2);
        // owm.read_blocking().await;
        owm.safety().await;
        owm.blindly_set_thresholds(1);
        owm.read_blocking().await;
        owm.safety().await;
        owm.blindly_set_thresholds(3);
        owm.read_blocking().await;
    }

    // Write and read back scratchpad, working at [bootstrap 414c102]
    if true {
        let mut b: [u8; 9] = [0; 9];

        //info!("Write scratchpad and read back...");
        owm.reset().await;
        trace!("skip rom, write scratchpad, 'H'ello, 'w'orld");
        owm.safely_write_byte_blocking(b'\xcc').await;
        owm.safely_write_byte_blocking(b'\x4e').await;
        owm.safely_write_byte_blocking(b'H').await;
        owm.safely_write_byte_blocking(b'w').await;
        owm.safety().await; // let that byte get all the way out before reset
        owm.reset().await;  // TODO Should every reset() (except at init) be "safely_"? Awfully easy to chop the end off the last bit sent above.
        //trace!("skip rom, read scratchpad");
        trace!("tx empty:{}, rx empty:{}", owm.sm.tx().empty(), owm.sm.rx().empty());
        owm.safely_write_byte_blocking(b'\xcc').await;
        owm.safely_write_byte_blocking(b'\xbe').await;
        for i in 0..9 {
            b[i] = owm.safely_read_byte_blocking().await;

        }
        trace!("read {:x}", b);
        assert!(b[2] == b'H', "First user byte should be H for Hello");
        assert!(b[3] == b'w', "Second user byte should be w for world");

        // Now prove we actually wrote something
        b = [0; 9];
        assert!(owm.reset().await);
        //trace!("skip rom, write scratchpad, 'G'oodbye 'c'ruel world");
        owm.safely_write_byte_blocking(b'\xcc').await;
        owm.safely_write_byte_blocking(b'\x4e').await;
        owm.safely_write_byte_blocking(b'G').await;
        owm.safely_write_byte_blocking(b'c').await;
        owm.safety().await;
        assert!(owm.reset().await);
        //trace!("skip rom, read scratchpad");
        owm.safely_write_byte_blocking(b'\xcc').await;
        owm.safely_write_byte_blocking(b'\xbe').await;
        for i in 0..9 {
            b[i] = owm.safely_read_byte_blocking().await;
        }
        trace!("read {:x}", b);
        assert!(b[2] == b'G', "First user byte should be G for Goodbye");
        assert!(b[3] == b'c', "Second user byte should be c for cruel world");

    }

    if true {
        info!("Search...");
        owm.reset().await;
        let mut st: SearchState = SearchState::start_general();
        const SEARCH_LIMIT: usize = 10; // or whatever
        let mut roms: Vec<RomId, SEARCH_LIMIT> = Vec::new();
        let mut n_others: usize = 0;
        while let Some(r) = owm.search(&mut st).await {
            info!(" found {:x}", r);
            match TemperatureSensorFamily::from_code(r[0]) {
                Ok(_fam) => {
                    roms.push(r).unwrap(); // correct: fails exceeding N, checked next
                    if roms.len() >= SEARCH_LIMIT {
                        break;
                    }
                }
                _ => n_others += 1,
            }
        }
        info!("Search found {:?} DS, {:?} others.", roms.len(), n_others);
    }

    // Read power situation
    if false {
        info!("Read power situation...");
        let presence = owm.reset().await;
        if presence {
            info!("At least one device is present at reset");
        } else {
            warn!("No devices responded to 1W bus reset");
        }
        trace!("skip rom, read power");
        owm.safely_write_byte_blocking(b'\xcc').await;
        owm.safely_write_byte_blocking(b'\xb4').await;
        owm.safety().await;  // wait for stall
        owm.blindly_set_thresholds(1);
        let result = owm.safely_read_blocking().await;
        info!("Result bit: {:?}", result >> 31);
        owm.safety().await;  // wait for stall
        owm.blindly_set_thresholds(2);
        let result = owm.safely_read_blocking().await;
        info!("Result two bits: {:?}", result >> 30);
        owm.safety().await;  // wait for stall
        owm.blindly_set_thresholds(3);
        let result = owm.safely_read_blocking().await;
        info!("Result three bits: {:?}", result >> 29);
    }

    loop {
        /*
        for rom in roms {
            owm.rom_select(&rom).await;
            ds_query(&owm, &rom).await;
            ds_wait(Precision::One).await;
            info!(" {:?}", ds_read(&owm).await);
        }
        info!("--");
        */
        Timer::after(Duration::from_secs(10)).await;
    }
}
