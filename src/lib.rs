//! # sx1268-rs

#![no_std]

pub mod config;
pub mod control;

pub mod codes;

mod sx1268;
pub use config::Sx1268Config;
pub use sx1268::{Error, Status, Sx1268};
