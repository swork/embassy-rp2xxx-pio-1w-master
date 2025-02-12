# rp2xx-pio-1wire-master-rs

A straight design ripoff from github stefanalt/RP2040-PIO-1-Wire-Master,
in async Rust.

Async system latency can be added at any .await point in the Rust code, but the
intent is that wire operations stay within devices' timing specifications: that
any time added to wire operations because Rust async operations are blocked
appears between bits, where 1wire operations allow indefinite pauses. In effect,
system decisions about async executors and priorities and interrupt operations
can affect the overall throughput of the 1wire bus but not the correctness of
bus operations.





