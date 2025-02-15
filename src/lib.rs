#![no_std]
#![no_main]

//! From code at github stefanalt/RP2040-PIO-1-Wire-Master, marked as follows:
//!-
//!-; SPDX-License-Identifier: BSD-3-Clause
//!-;
//!-; 1-Wire is a tradmark of Maxim Integrated
//!
//! Please refer to stefanalt's original work for much helpful commentation.
//!
//! See also github embassy-rs/embassy for some Rust code ripoffs.
//!
//! Note there's no deliberate support yet for 1Wire's overdrive speed mode.
//!
//! embassy is a work in progress, and these notes should be read with that in
//! mind:
//!
//! 1. PIO implementations "out there" commonly adjust the clock divisor, adjust
//!    the push and pull thresholds, and query these values and others. AFAICT at
//!    the time of this writing embassy-rp doesn't give a way to do this. Docs
//!    say "drop down to underlying HAL when needed" but (again, at this writing,
//!    AFAICT) HAL functionality is private to user code. If this isn't true,
//!    it's a knock against docs that info on how to accomplish these common
//!    tasks is hard enough to find that I didn't. I prepped a push-request
//!    supplying the operations I needed, and linked against my hacked code while
//!    it is being considered.
//!
//! 2. Examples pin pio-proc and pio to specific git revisions, and fail at
//!    release revisions (that I tried). This is really likely something that
//!    just hasn't caught up with many things changing rapidly, but is a
//!    good-sized wrench to throw into the process of getting simple programs
//!    running.
//!
//! 3. Docs generally are minimal. No point investing in docs for stuff that we
//!    know will change, but even bare outlines are better than nothing but
//!    highly abstracted code. RTSL, sure, but this is a big system with a lot of
//!    dependencies and it's easy to get lost in the tree.
//!
//! 4. This lib exists because embassy-rp/pio/onewire (again, AFAICT) isn't
//!    useful beyond simplest demo programs. Enumerating the bus is a basic
//!    operation, and with the current implementation it would require running a
//!    separate program, tearing it down and starting up the intended program
//!    with the info gathered from the first. If that's what's intended, it
//!    should be doc'd up front.
//!

pub mod devices;

use defmt::*;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::pio::{
    instr, Common, Config, Instance, LoadedProgram, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_time::{Duration, Timer};
use fixed::traits::ToFixed;
use fixed::{types::extra::U8, FixedU32};

/// All 1Wire devices include an 8-bit ROM ID: 8-bit CRC, 48-bit ID, 8-bit
/// family code. We keep the CRC here, for no good reason.
pub type RomId = [u8; 8];

/// All 1Wire devices respond to a search algorithm that can be used to
/// enumerate a 1Wire bus. This struct tracks state for a search in progress.
pub struct SearchState {
    /// Set true going into `search` to use ALARM SEARCH command, else general SEARCH
    pub alarm_search: bool,
    // remaining values are private
    last_discrepancy: usize,
    last_family_discrepancy: usize, // TODO Unused; is it necessary or helpful?
    rom: RomId,
    last_device_flag: bool,
    crc8: u8,
}

impl Default for SearchState {
    fn default() -> Self {
        Self {
            alarm_search: false,
            last_discrepancy: 0,
            last_device_flag: false,
            last_family_discrepancy: 0,
            rom: [0; 8],
            crc8: 0,
        }
    }
}

impl SearchState {
    /// Initialize SearchState for a general (all devices) search.
    pub fn start_general() -> Self {
        Default::default()
    }

    /// Initialize SearchState for a conditional ("alarm") search.
    pub fn start_alarm() -> Self {
        Self {
            alarm_search: true,
            ..Default::default()
        }
    }
}

/// Incrementally calculate CRC8 from starting value and one new byte.
fn crc8_incremental(crc8: u8, data: &u8) -> u8 {
    // See Maxim Application Note 27
    let mut crc8 = crc8 ^ data;
    for _ in 0..8 {
        if crc8 & 1 != 0 {
            crc8 = (crc8 >> 1) & 0x8c_u8;
        } else {
            crc8 = crc8 >> 1
        }
    }
    crc8
}

/// Calculate 1Wire CRC8 of a sequence of bytes.
pub fn crc8(data: &[u8]) -> u8 {
    let mut crc = 0;
    for b in data {
        crc = crc8_incremental(crc, b);
    }
    crc
}

/// Represent a PIO program, including its code and wraps.
///
/// TODO Look into exposing labels.
pub struct OneWireMasterProgram<'a, PIO: Instance> {
    pub prg: LoadedProgram<'a, PIO>,
}

impl<'a, PIO: Instance> OneWireMasterProgram<'a, PIO> {
    /// Load program
    pub fn new(common: &mut Common<'a, PIO>) -> Self {
        let prg = pio_proc::pio_asm!(
            r#"
; .program onewire
.side_set 1 pindirs

public reset:
    nop           side 1 [6]     ; (1+6)*70us = 490us low
    nop           side 0         ; 1*70us = 70us high
    in pins, 1    side 0 [6]     ; will sample pin state
                                 ; and (1+6)*70us = 480us high delay
                                 ; to next operation
    jmp start     side 0;

; The rx/tx-branch assumes 3us instruction timing (CLKDIV = CPU-MHz*3)
.wrap_target
do_0:
    in pins, 1    side 1 [15]   ; will sample s.th. (value does not care)
                                ; and provides (1+15)*3us = 48us low
    jmp get_bit   side 1  [1]   ; (1+1)*3us = 6us low
do_1:
    nop           side 0  [2]   ; (1+2)*3us = 9us high
    in pins, 1    side 0 [14]   ; will sample pin state at samplepoint
                                ; and provides (1+14)*3us = 45us high
public start:
get_bit:
    mov pins, y   side 0  [2]   ; set pinctlz from y-register. This is
                                ; to implement strong external pullup
                                ; transistor from ARM code
                                ; and provides (1+2)*3us = 9us low between bits
public waiting:
    out x, 1      side 0        ; stalls if no data available
                                ; ARM code checks that this instruction is
                                ; reached to make sure that everything is
                                ; done: Therefore, this instruction must not have
                                ; any delay cycles
                                ; and provides additional 3us high between
                                ; bits, more if stalling
    jmp x-- do_1  side 1  [1]   ; (1+1)*3us = 6us low to start a bit cycle
.wrap
        "#,
        );
        let prg = common.load_program(&prg.program);
        Self { prg }
    }
}

/// Until labels are exposed we have to manually count locations.
const INSTR_LABEL_RESET: u8 = 0;
const INSTR_LABEL_START: u8 = 8;
const INSTR_LABEL_WAITING: u8 = 9;

/// Pio-backed 1Wire driver
pub struct OneWireMaster<'d, PIO: Instance, const SM: usize> {
    /// The state machine we're running on
    pub sm: StateMachine<'d, PIO, SM>,
    /// The offset in program memory of the start of our program
    pub origin: u8,
    /// True if strong pull-up GPIO is configured, false otherwise
    pub spu: bool,
    // /// Last-set FIFO threshold, for tuning transmit/receive waits
    // threshold: u8,
}

impl<'d, PIO: Instance, const SM: usize> OneWireMaster<'d, PIO, SM> {
    fn new_common(
        common: &mut Common<'d, PIO>,
        mut sm: &mut StateMachine<'d, PIO, SM>,
        pin: impl PioPin,
        program: OneWireMasterProgram<'d, PIO>,
    ) -> Config<'d, PIO> {
        let pin = common.make_pio_pin(pin);
        let mut cfg = Config::default();
        // sideset for output
        cfg.use_program(&program.prg, &[&pin]);
        cfg.set_in_pins(&[&pin]); // data line is read directly
        let threshold = 32;
        cfg.shift_in = ShiftConfig {
            auto_fill: true,
            direction: ShiftDirection::Right,
            threshold,
        };
        cfg.shift_out = ShiftConfig {
            auto_fill: true,
            direction: ShiftDirection::Right,
            threshold,
        };
        let d: FixedU32<U8> = ((clk_sys_freq() / 1_000_000) * 3).to_fixed();
        defmt::trace!("POWM::new_impl: divider {:?}", d.to_num::<f32>());
        cfg.clock_divider = d;

        // Safety: This block is safe because we are the only writer, and the program isn't running.
        unsafe {
            // preload register y with 1 ==> SPU high (where low ==> asserted)
            instr::set_y(&mut sm, 1);
        }
        sm.clear_fifos(); // why does set_y leave bits in the OSR?
        if sm.tx().empty() {
            defmt::trace!(" 2 tx fifo is properly empty.");
        } else {
            defmt::trace!(" 2 tx fifo level: {:?}", sm.tx().level());
        }
        cfg
    }

    pub fn new_with_spu(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        pin: impl PioPin,
        pin_spu: impl PioPin,
        program: OneWireMasterProgram<'d, PIO>,
    ) -> Self {
        let origin = program.prg.origin;
        let pin_spu = common.make_pio_pin(pin_spu);
        let mut cfg = Self::new_common(common, &mut sm, pin, program);
        cfg.set_out_pins(&[&pin_spu]);
        cfg.set_set_pins(&[&pin_spu]);
        Self {
            sm,
            origin,
            spu: true,
        }
        .setup(cfg)
    }

    pub fn new(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        pin: impl PioPin,
        program: OneWireMasterProgram<'d, PIO>,
    ) -> Self {
        let origin = program.prg.origin;
        let cfg = Self::new_common(common, &mut sm, pin, program);
        Self {
            sm,
            origin,
            spu: false,
        }
        .setup(cfg)
    }

    fn setup(mut self: Self, cfg: Config<'d, PIO>) -> Self {
        self.sm.set_config(&cfg);

        // Safe because we're the only writer, and the SM isn't enabled.
        unsafe {
            // jump to the start location
            instr::exec_jmp(&mut self.sm, self.origin + INSTR_LABEL_START);
        }

        self.sm.set_enable(true);
        self
    }

    /// Wait until the SM is stalled (TX FIFO and OSR empty, PC at "waiting",
    /// and RX FIFO empty if `drain_rx`) as required for safe use of several API
    /// functions. De-assert the strong-pull-up, if configured and enabled.
    pub async fn safety(self: &mut Self) -> () {
        let three_us = Duration::from_micros(3);
        let fifty_us = Duration::from_micros(50);
        'outer: loop {
            'tx_empty: loop {
                if self.sm.tx().empty() {
                    break 'tx_empty;
                } else {
                    Timer::after(fifty_us).await;
                }
            }
            let mut timeout = 2000; // 2mS
            'rx_idle_wait: loop {
                if self.sm.tx().stalled() && self.sm.get_addr() == self.origin + INSTR_LABEL_WAITING {
                    if self.sm.rx().empty() {
                        break 'outer;
                    } else {
                        self.sm.clear_fifos();
                        continue 'rx_idle_wait;
                    }
                }
                Timer::after(three_us).await;
                self.sm.rx().pull();
                timeout -= 1;
                if timeout <= 0 {
                    error!("SM failed safety");
                    break 'outer;
                }
            }
        }
        self.sm.clear_fifos();  // TODO Thrashing here, fifos should be clear already
        if self.spu {
            self.blindly_end_spu();
        }
    }

    pub async fn rx_fifo_nonempty_wait(self: &mut Self) -> () {
        let threshold = self.sm.get_rx_threshold();
        const fn guess_us(threshold: u64) -> u64 {
            threshold * 40 // start point. Tune this to minimum additional waits, overruns only for .await latency
        }
        Timer::after(Duration::from_micros(guess_us(threshold.into()))).await;
        let short_delay = Duration::from_micros(2);
        let mut timer = 2000_i32;
        while self.sm.rx().empty() {
            Timer::after(short_delay).await;
            timer -= 1;
            if timer <= 0 {
                error!("wait failure");
                break;
            }
        }
        defmt::debug!(
            "rx_fifo_nonempty_wait thresh:{:?} residual:{:?}",
            threshold,
            timer
        );
    }

    pub fn blindly_set_thresholds(self: &mut Self, bits: u8) -> () {
        defmt::assert!(bits <= 32);
        self.sm.set_thresholds(bits & 0x1f);  // 0 means 32
    }

    /// The general "send bits" function, simply puts a 32-bit word on the TX
    /// FIFO if space is available. No effect and no error if the FIFO is full.
    ///
    /// On a stalled SM this word is immediately transferred to the OSR. The SM
    /// begins running (assuming RX FIFO is not the cause of the stall),
    /// counting bits shifted out until the tx threshold value is reached, at
    /// which point the SM stalls again (assuming no other activity).
    ///
    /// As each bit is shifted out the bus is sampled. If any device is pulling
    /// the line low at the moment of sampling, a `1` bit is shifted into the
    /// ISR; else a `0` bit is shifted in. If RX and TX thresholds are set to
    /// the same value, these bits will be waiting in the RX FIFO after (a) this
    /// function finishes, and then (b) the SM stalls (for empty TX OSR and
    /// FIFO). This is normal 1Wire bus operation, so this function front-ends
    /// the `read_blocking` operation.
    pub async fn blindly_write(self: &mut Self, data: u32) -> () {
        self.sm.tx().push(data);
    }

    /// The general "receive bits" function. "Sends" `1` bits to the current rx
    /// threshold, blocks until the SM stalls (presumably for TX FIFO empty) and
    /// returns the RX FIFO contents. If preconditions are met this will be the
    /// bits sampled from the bus during each bit write cycle, per 1Wire docs.
    /// Preconditions assumed include ISR and RX FIFO empty at start, SM stalled
    /// on output, and TX threshold set equal or greater than RX threshold.
    ///
    /// Returns one 32-bit RX FIFO word after waiting for output stall. Assuming
    /// ISR shift direction is set `ShiftDirection::Right` (as arranged by
    /// `Self::new`) the last-received bit is MSB of the return value, which is
    /// labeled "_raw_" because you must mask/shift the return value according
    /// to the current threshold setting. LSBs are undefined when read threshold
    /// is not 32 ("0" in the five-bit threshold register).
    pub async fn read_raw_blocking(self: &mut Self) -> u32 {
        let n = self.sm.get_rx_threshold();
        let thresh_bits = (0b1_u32 << n) - 1;
        defmt::trace!("read_raw_blocking: thresh is {:?} so bits {:?}", n, thresh_bits);
        self.blindly_write(thresh_bits).await;
        self.rx_fifo_nonempty_wait().await;
        self.sm.rx().pull() // "Get from RX FIFO", not the PIO "pull" operation
    }

    /// As `read_raw_blocking` but ensure SM is stalled first.
    pub async fn safely_read_raw_blocking(self: &mut Self) -> u32 {
        self.safety().await;
        self.read_raw_blocking().await
    }

    /// Wait until a safe moment (TX FIFO is empty and SM is at "waiting") then
    /// sequence one byte onto the bus. Return incoming bits (which are only
    /// valid if they correspond to write-1 bits) when incoming bits appear
    /// from the RX FIFO, so after the last write bit goes out but maybe
    /// something less than 54uS (subtract for .await latency) before the SM
    /// stalls at "waiting".
    pub async fn safely_write_byte_blocking(self: &mut Self, data: u8) -> u32 {
        self.safety().await;
        self.blindly_set_thresholds(8);
        self.sm.tx().push(data as u32);

        // return this value - we're part of the read impl. It's 32-bits, with
        // the interesting part at the MSB end, but don't waste time fixing it
        // up here because it's discarded if the intent was not to read the bus.
        self.sm.rx().wait_pull().await
    }

    /*  BUSTED TODO BUSTED
    /// As `safely_write_byte_blocking` but one or more bytes, buffering through
    /// FIFOs. Read-back bytes are discarded. TODO Send them back in data,
    /// supporting a multibyte read operation.
    pub async fn safely_write_bytes_blocking(self: &mut Self, data: &[u8]) -> () {
        self.safety().await;
        let mut balance: isize = 0;
        let n: usize = data.len();
        let mut i: usize = 0;  // index into data
        for s in (1..=4).rev() {
            if n - i >= s {
                self.blindly_set_thresholds((s * 8) as u8);
                while n - i >= s {
                    while !self.sm.rx().empty() {
                        self.sm.rx().pull();
                        defmt::trace!("bal -= 1 A");
                        balance -= 1;
                    }
                    let mut w: u32 = data[i].into();
                    for j in 1..s {
                        w |= (data[i + j] as u32) << (j * 8);
                    }
                    if !self.sm.tx().full() {
                        defmt::trace!("push {:x}", w);
                        self.sm.tx().push(w);
                        defmt::trace!("bal += 1 A");
                        balance += 1;
                    } else {
                        defmt::trace!("wait_push {:x}", w);
                        self.sm.tx().wait_push(w).await;
                        defmt::trace!("bal += 1 B");
                        balance += 1;
                    }
                    i += s;
                }
                while balance > 0 {
                    defmt::trace!("> wait_pull, txempty:{:?}, rxempty:{:?}", self.sm.tx().empty(), self.sm.rx().empty());
                    defmt::trace!("bal -= 1 B");
                    self.sm.rx().wait_pull().await;
                    balance -= 1;
                    defmt::trace!("< wait_pull");
                }
            }
        }
    }
    */

    /// As safely_write_byte_blocking(), but then raise the Strong Pull-Up line
    /// as required for parasite-power bus. Match each call with self.end_spu()
    /// at a time of your choosing (but certainly before the next bus operation,
    /// else there's a risk of frying the PIO output pin's drain transistor).
    /// This function takes more CPU cycles and introduces an .await point
    /// during a 54uS window before the last bit goes out, raising the
    /// possibility that bit is delayed - which is within 1Wire specifications
    /// (any length of delay is allowed between bits while the master leaves the
    /// line floating) but arguably messy. It is allowable and safe to use this
    /// function even if SPU is not configured, just costlier than the non-spu
    /// version.
    pub async fn safely_write_byte_blocking_spu(self: &mut Self, data: u8) -> u32 {
        self.safety().await;
        self.blindly_set_thresholds(7);
        self.sm.tx().push(data as u32);
        self.rx_fifo_nonempty_wait().await;
        self.sm.rx().pull(); // toss "incoming" bits

        // Last bit, with SPU asserted immediately after
        self.blindly_set_thresholds(1);
        unsafe {
            // Safety: we are the only writer
            instr::set_y(&mut self.sm, 0);
        }
        self.sm.tx().push((data >> 7) as u32);
        self.sm.rx().wait_pull().await // return this (raw, 32-bit, MSB-interesting)
    }

    /// Wait until a safe moment (TX FIFO is empty, SM is at "waiting") then
    /// ask the bus for eight bits. Might return as much as 54uS before the SM
    /// stalls at "waiting".
    pub async fn safely_read_byte_blocking(self: &mut Self) -> u8 {
        // Received byte is bits 31..24 of FIFO word
        ((self.safely_write_byte_blocking(0xff).await >> 24) & 0xff) as u8
    }

    /// Wait until a safe moment (TX FIFO is empty, SM is at "waiting") then ask
    /// the bus for eight bits. Might return as much as 54uS before the SM
    /// stalls at "waiting". Match each call with self.end_spu() at a time of
    /// your choosing. It is allowable and safe to use this function even if SPU
    /// is not configured, just costlier than the non-spu version.
    pub async fn safely_read_byte_blocking_spu(self: &mut Self) -> u8 {
        ((self.safely_write_byte_blocking_spu(0xff).await >> 24) & 0xff) as u8
    }

    /// End strong pull-up, presumably after preceding ..._write_..._spu() or
    /// ..._read_..._spu() enables it. It's allowable to call this function even
    /// if SPU is not asserted, and even if SPU is not enabled.
    pub async fn safely_end_spu_blocking(self: &mut Self) -> () {
        self.safety().await; // which calls blindly_end_spu()
    }

    /// End strong pull-up, presumably after preceding ..._write_..._spu() or
    /// ..._read_..._spu() has enabled it. It's allowable to call this function
    /// even if SPU is not asserted, and even if SPU is not enabled. You'll
    /// probably screw up bus operations in progress if you call this when the
    /// SM is not stalled.
    pub fn blindly_end_spu(self: &mut Self) -> () {
        unsafe {
            // Preset y register so no SPU during next bit
            instr::set_y(&mut self.sm, 1);
            // Set control pin high right now
            instr::set_pin(&mut self.sm, 1);
        }
        // Safety: we are the only writer.
    }

    /// Reset the bus, then silence all but one device by rom_id.
    /// Return 1, or 0 if the reset indicates no devices present.
    pub async fn rom_select(self: &mut Self, rom_id: &RomId) -> usize {
        if !self.reset().await {
            return 0;
        }
        self.safely_write_byte_blocking(0x55).await; // "Match ROM"
        for i in 0..8 {
            self.safely_write_byte_blocking(rom_id[i]).await;
        }
        1
    }

    /// Reset the bus, without concern for current state - we wipe it out.
    /// Consider calling .safety() first if the previous operation could still
    /// be in flight. Return a bool indicating whether any devices signaled
    /// their presence during the cycle.
    pub async fn reset(self: &mut Self) -> bool {
        // Adjust timing to match reset section of PIO program, and
        // set thresholds for bit-by-bit operation
        let d: FixedU32<U8> = ((clk_sys_freq() / 1_000_000) * 70).to_fixed();
        defmt::trace!("reset 1: divider {:?}", d.to_num::<f32>());
        self.blindly_set_thresholds(1);
        self.sm.clear_fifos();
        self.sm.set_clock_divider(d);
        self.sm.clkdiv_restart();

        // Kick off the reset
        unsafe {
            instr::exec_jmp(&mut self.sm, self.origin + INSTR_LABEL_RESET);
        }
        // Safety: we're the only writer; the reset puts the bus in a safe state
        // regardless of its state to start.

        // any devices present pulled the line low, shifting 1 into MSB of ISR.
        // The rest of the LSB are undefined.
        let presence = ((self.sm.rx().wait_pull().await & 0x8000_0000) >> 31) == 0;
        if !presence {
            warn!("No devices responded to 1W bus reset");
        }

        // Now wait until SM is idle to reset the clock rate.
        self.safety().await;
        let d: FixedU32<U8> = ((clk_sys_freq() / 1_000_000) * 3).to_fixed();
        defmt::trace!("reset 2: divider {:?}", d.to_num::<f32>());
        self.sm.set_clock_divider(d);
        self.sm.clkdiv_restart();

        // Return the presence result
        presence
    }

    /// From stefanalt's original source:
    /// /* Do a ROM search triplet.
    ///    Receive two bits and store the read values to
    ///    id_bit and cmp_id_bit respectively.
    ///    Then transmit a bit with this logic:
    ///     id_bit | cmp_id_bit | tx-bit
    ///          0 |          1 |      0
    ///          1 |          0 |      1
    ///          0 |          0 | search_direction
    ///          1 |          1 |      1
    ///    The actually transmitted bit is returned via search_direction.
    ///
    ///    Refer to MAXIM APPLICATION NOTE 187 "1-Wire Search Algorithm"
    /// */
    ///
    /// Expect goodness only if the SM is stalled on entry.
    async fn search_triplet(
        self: &mut Self,
        id_bit: &mut u32,
        cmp_id_bit: &mut u32,
        search_direction: &mut u32,
    ) -> () {
        self.blindly_set_thresholds(2);
        let fiforx = self.read_raw_blocking().await;
        *id_bit = (fiforx >> 30) & 1;
        *cmp_id_bit = (fiforx >> 31) & 1;
        match (*id_bit, *cmp_id_bit) {
            (0, 0) => (), // no change to search direction
            (0, 1) => *search_direction = 0,
            (1, 0) | (1, 1) => *search_direction = 1,
            _ => defmt::error!("Badly busted bit values"),
        }
        defmt::debug!(
            "search_triplet: read {:?}, {:?}; write {:?}",
            id_bit,
            cmp_id_bit,
            search_direction
        );
        self.blindly_set_thresholds(1);
        self.blindly_write(*search_direction).await;
    }

    /// Start or continue a search. Returns Some(rom_id), in which case call
    /// this function again with `st` unchanged to find more, or None if no more
    /// exist.
    pub async fn search(self: &mut Self, st: &mut SearchState) -> Option<RomId> {
        if st.last_device_flag {
            *st = Default::default();
            return None;
        }

        if !self.reset().await {
            *st = Default::default();
            // return None;  TODO commented out for testing
        }

        defmt::info!("Send general-search in 2s");
        Timer::after(Duration::from_secs(2)).await;

        self.safely_write_byte_blocking(if st.alarm_search {
            0xec_u8 // "alarm" or "conditional" search command
        } else {
            0xf0_u8 // "normal" search command, all devices participate
        })
        .await;

        let mut found1 = false;
        let mut id_bit: u32 = 0;
        let mut cmp_id_bit: u32 = 0;
        let mut id_bit_number: usize = 0;
        let mut last_zero: usize = 0;
        let mut rom_byte_number: usize = 0;
        let mut rom_byte_mask: u8 = 1;
        let mut search_direction: u32;
        loop {
            // Pick next-bit search direction, 0 or 1: If this discrepancy is
            // before last_discrepancy from a previous call, pick the same bit
            // as last time.
            if id_bit_number < st.last_discrepancy {
                if st.rom[rom_byte_number] & rom_byte_mask != 0 {
                    search_direction = 1;
                } else {
                    search_direction = 0;
                }
            } else {
                // If equal to last pick 1, else pick 0
                if id_bit_number == st.last_discrepancy {
                    search_direction = 1;
                } else {
                    search_direction = 0
                }
            }

            // Do the next write-two, read-one search operation
            defmt::debug!(
                "search: bit {:?} byte {:?} mask {:?} dir {:?} in 3s",
                id_bit_number,
                rom_byte_number,
                rom_byte_mask,
                search_direction
            );
            Timer::after(Duration::from_secs(3)).await;
            self.safety().await;
            self.search_triplet(&mut id_bit, &mut cmp_id_bit, &mut search_direction)
                .await;

            // Check for no devices on the bus (or maybe last device hot-removed?)
            if id_bit != 0 && cmp_id_bit != 0 {
                break;
            } else {
                if id_bit == 0 && cmp_id_bit == 0 && search_direction == 0 {
                    last_zero = id_bit_number;

                    // check for last discrepancy in family
                    if last_zero < 9 {
                        st.last_family_discrepancy = last_zero;
                    }
                }

                // set or clear the bit in the current ROM byte
                if search_direction != 0 {
                    st.rom[rom_byte_number] |= rom_byte_mask;
                } else {
                    st.rom[rom_byte_number] &= !rom_byte_mask;
                }

                // prep rom counters for next bit
                id_bit_number += 1;
                let overflow;
                (rom_byte_mask, overflow) = rom_byte_mask.overflowing_shl(1);
                if overflow {
                    st.crc8 = crc8_incremental(st.crc8, &(rom_byte_number as u8));
                    rom_byte_number += 1;
                    rom_byte_mask = 0b1_u8;
                }

                // loop for all rom bytes, 0 through 7
                if rom_byte_number >= 8 {
                    break;
                }
            }

            // Determine if the search was successful
            if id_bit_number >= 65 && st.crc8 == 0 {
                // Yes, found a ROM ID. Set up to find the next one?
                st.last_discrepancy = last_zero;
                if st.last_discrepancy == 0 {
                    st.last_device_flag = true;
                }
                found1 = true;
            }
        }

        // If nothing foiund reset so next 'search' will be like a first search.
        if !found1 || st.rom[0] == 0 {
            *st = Default::default();
        }

        // return success or not
        match found1 {
            true => Some(st.rom),
            _ => None,
        }
    }
}
