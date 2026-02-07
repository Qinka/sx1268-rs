#![no_std]
//! # sx1268-rs
//!
//! A `no_std` Rust driver for the Semtech SX1268 LoRa transceiver.
//!
//! This crate uses [`embedded-hal`](https://crates.io/crates/embedded-hal) SPI
//! traits for hardware communication and [`defmt`](https://crates.io/crates/defmt)
//! for structured logging.
//!
//! ## Usage
//!
//! ```rust,no_run
//! use sx1268_rs::Sx1268;
//! use sx1268_rs::types::*;
//!
//! fn example<SPI: embedded_hal::spi::SpiDevice>(spi: SPI) {
//!     let mut radio = Sx1268::new(spi);
//!     // Put the radio in standby mode
//!     radio.set_standby(StandbyConfig::StbyRc).ok();
//! }
//! ```

pub mod error;
pub mod opcode;
pub mod sx1268;
pub mod types;

pub use error::Error;
pub use sx1268::Sx1268;
