#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use heapless::Vec;
use rp2xxx_pio_1_wire_master_rs::{PioOneWireMaster, PioOneWireMasterProgram, SearchState, RomId};  // , devices::Ds18::{Precision, ds_query, ds_wait, ds_read}};
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

    defmt::trace!("make master pgm");
    let owmp = PioOneWireMasterProgram::<PIO0>::new(&mut common);
    defmt::trace!(" program origin {:?}", owmp.prg.origin);

    Timer::after(Duration::from_secs(1)).await;
    defmt::trace!("make master itself");
    let mut owm = PioOneWireMaster::<PIO0, 0>::new(&mut common, sm0, p.PIN_15, &owmp)
        .await  // not really async, just slowing things down to test
        ;

    Timer::after(Duration::from_secs(1)).await;
    defmt::debug!("Safety first! (just to test)");
    owm.safety(true).await;
    Timer::after(Duration::from_secs(2)).await;

    defmt::trace!("Test reset: {:?}", owm.reset().await);
    loop {}

    let mut st: SearchState = SearchState::start_general();

    const SEARCH_LIMIT: usize = 10;  // or whatever
    let mut roms: Vec<RomId, SEARCH_LIMIT> = Vec::new();
    let mut n_others: usize = 0;
    defmt::info!("Search:");
    while let Some(r) = owm.search(&mut st).await {
        info!("  {:?}", r);
        match r[0..=1] {
            [b'\x12', b'\x34'] => {
                roms.push(r).unwrap();  // correct: fails exceeding N, checked elsewhere
                if roms.len() >= SEARCH_LIMIT {
                    break;
                }
            },
            _ => n_others += 1,
        }
    }
    info!("Search found {:?} DS, {:?} others.", roms.len(), n_others);

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

