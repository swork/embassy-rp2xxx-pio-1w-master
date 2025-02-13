#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use heapless::Vec;
use rp2xxx_pio_1_wire_master_rs::{PioOneWireMaster, PioOneWireMasterProgram, SearchState, RomId, devices::TemperatureSensorFamily}; //Ds18::{Precision, ds_query, ds_wait, ds_read}};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    defmt::info!("hello.");

    let p = embassy_rp::init(Default::default());
    let Pio {
        mut common, sm0, ..
    } = Pio::new(p.PIO0, Irqs);

    let owmp = PioOneWireMasterProgram::<PIO0>::new(&mut common);
    let mut owm = PioOneWireMaster::<PIO0, 0>::new(&mut common, sm0, p.PIN_15, &owmp);

    if false {
        let mut st: SearchState = SearchState::start_general();
        const SEARCH_LIMIT: usize = 10;  // or whatever
        let mut roms: Vec<RomId, SEARCH_LIMIT> = Vec::new();
        let mut n_others: usize = 0;
        defmt::info!("Search:");
        while let Some(r) = owm.search(&mut st).await {
            info!("  {:?}", r);
            match TemperatureSensorFamily::from_code(r[0]) {
                Ok(_fam) => {
                    roms.push(r).unwrap();  // correct: fails exceeding N, checked next
                    if roms.len() >= SEARCH_LIMIT {
                        break;
                    }
                },
                _ => n_others += 1,
            }
        }
        info!("Search found {:?} DS, {:?} others.", roms.len(), n_others);
    }

    // Last I saw this one always retrieves all-ones bytes.
    if true {
        let presence = owm.reset().await;
        if presence {
            defmt::info!("At least one device is present at reset");
        } else {
            defmt::warn!("No devices responded to 1W bus reset");
        }

        // defmt::info!("Scratchpad in 2s:");
        // Timer::after(Duration::from_secs(2)).await;

        // defmt::info!("CC");
        owm.safely_write_byte_blocking(b'\xcc').await;

        // Timer::after(Duration::from_secs(2)).await;
        // defmt::info!("4e, write scratchpad");
        owm.safely_write_byte_blocking(b'\x4e').await;

        Timer::after(Duration::from_secs(2)).await;
        defmt::info!("H (ello) w (orld)");
        owm.safely_write_byte_blocking(b'H').await;
        owm.safely_write_byte_blocking(b'w').await;

        defmt::info!("Now reset and read back in 5s:");
        owm.reset().await;
        owm.safely_write_byte_blocking(b'\xcc').await;
        owm.safely_write_byte_blocking(b'\xbe').await;
        Timer::after(Duration::from_secs(5)).await;
        for i in 0..9 {
            let b = owm.safely_read_byte_blocking().await;
            defmt::info!("read {:?}: {:?}", i, b);
        }
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

