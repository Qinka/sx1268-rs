// 该文件是 sx1268-rs 项目的一部分。
// src/lib.rs - 主库文件
//
// 本文件根据 Apache 许可证第 2.0 版（以下简称“许可证”）授权使用；
// 除非遵守该许可证条款，否则您不得使用本文件。
// 您可通过以下网址获取许可证副本：
// http://www.apache.org/licenses/LICENSE-2.0
// 除非适用法律要求或书面同意，根据本许可协议分发的软件均按“原样”提供，
// 不附带任何形式的明示或暗示的保证或条件。
// 有关许可权限与限制的具体条款，请参阅本许可协议。
//
// Copyright (C) 2026 Johann Li <me@qinka.pro>, Wareless Group


//! # sx1268-rs
//!
//! A `no_std` Rust driver for the Semtech SX1268 sub-GHz LoRa transceiver.
//!
//! This crate provides a hardware-agnostic driver that communicates with the
//! SX1268 chip through the [`Control`](control::Control) trait.  Users supply
//! an implementation of `Control` (typically backed by SPI + GPIO) and then
//! call [`Sx1268::init`] with an [`Sx1268Config`] to bring the radio up.
//!
//! ## Crate structure
//!
//! | Module          | Description |
//! |-----------------|-------------|
//! | [`config`]      | Configuration types and the builder-pattern [`Sx1268Config`] |
//! | [`control`]     | Hardware abstraction trait for SPI / GPIO access |
//! | [`codes`]       | Raw SX1268 SPI command opcodes and register addresses |
//!
//! ## Quick start
//!
//! ```rust,ignore
//! use sx1268_rs::{Sx1268, Sx1268Config};
//! use sx1268_rs::config::*;
//!
//! // Build configuration with the builder API
//! let config = Sx1268Config::default()
//!     .with_frequency_hz(433_000_000).unwrap()
//!     .with_tx_power(22)
//!     .with_lora_modulation(
//!         LoRaModulationParams::default()
//!             .with_spreading_factor(LoRaSpreadingFactor::Sf7)
//!             .with_bandwidth(LoRaBandwidth::Bw125)
//!             .with_coding_rate(LoRaCodingRate::Cr4_5),
//!     )
//!     .with_dio2_as_rf_switch();
//!
//! let mut radio = Sx1268::new(my_control);
//! radio.init(config).unwrap();
//!
//! // Transmit a packet
//! radio.send_lora(b"Hello LoRa", 0).unwrap();
//! ```

#![no_std]

pub mod config;
pub mod control;

pub mod codes;

mod sx1268;
pub use config::Sx1268Config;
pub use sx1268::{Error, Status, Sx1268};
