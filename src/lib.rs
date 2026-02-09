//! # sx1268-rs

#![no_std]

pub mod config;
pub mod control;

pub mod codes;

mod sx1268;
pub use sx1268::{Sx1268, Error, Status};
pub use config::Sx1268Config;
