// 该文件是 sx1268-rs 项目的一部分。
// src/config.rs - 配置类型
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

//! Configuration types for the SX1268 driver.
//!
//! This module provides the [`Sx1268Config`] builder and all associated
//! sub-types needed to fully configure an SX1268 for LoRa operation:
//!
//! - **Modulation** — [`LoRaSpreadingFactor`], [`LoRaBandwidth`],
//!   [`LoRaCodingRate`], [`LoRaModulationParams`]
//! - **PA** — [`PaConfig`] with factory-tested power-level presets
//! - **Packet** — [`PacketType`], [`LoRaHeaderType`], [`LoRaPacketParams`]
//! - **System** — [`RegulatorMode`], [`RampTime`], [`FallbackMode`],
//!   [`SleepConfig`], [`StandbyConfig`], [`CalibrationParams`],
//!   [`TcxoConfig`], [`TcxoVoltage`], [`CadSymbols`], [`CadExitMode`]
//!
//! The builder pattern (`with_*` methods) is used to construct a config
//! and then pass it to [`Sx1268::init`](crate::Sx1268::init).

mod modulation;
mod pa_config;
mod package;
mod types;

pub use modulation::{LoRaBandwidth, LoRaCodingRate, LoRaModulationParams, LoRaSpreadingFactor};
pub use pa_config::PaConfig;
pub use package::{LoRaHeaderType, LoRaPacketParams, PacketType};
pub use types::{
  CadExitMode, CadSymbols, CalibrationParams, FallbackMode, RampTime, RegulatorMode, SleepConfig,
  StandbyConfig, TcxoConfig, TcxoVoltage,
};

/// Minimum supported RF frequency in Hz (410 MHz).
const FREQ_MIN_HZ: u32 = 410_000_000;
/// Maximum supported RF frequency in Hz (510 MHz).
const FREQ_MAX_HZ: u32 = 510_000_000;
/// Default RF frequency in Hz (433 MHz, ISM band).
const FREQ_DEFAULT_HZ: u32 = 433_000_000;

/// Error returned when an [`Sx1268Config`] builder method receives an
/// out-of-range or otherwise invalid value.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ConfigError {
  /// The requested frequency is outside the supported range
  /// [`FREQ_MIN_HZ`]–[`FREQ_MAX_HZ`].
  InvalidFrequency {
    /// The frequency value that was rejected.
    freq: u32,
    /// Upper bound (inclusive).
    max: u32,
    /// Lower bound (inclusive).
    min: u32,
  },
}

/// Aggregated configuration for the SX1268 peripheral.
///
/// All fields use `pub(super)` visibility so that the driver implementation
/// in [`crate::sx1268`] can read them directly while external code must go
/// through the builder API or the `get_*` accessors.
///
/// # Default values
///
/// | Parameter       | Default                 |
/// |-----------------|-------------------------|
/// | Packet type     | LoRa                    |
/// | Frequency       | 433 MHz                 |
/// | TX power        | 0 dBm                   |
/// | Ramp time       | 10 µs                   |
/// | SF / BW / CR    | SF7 / 125 kHz / 4/5     |
/// | Regulator       | LDO                     |
/// | Sync word       | `0x14` (private)        |
/// | DIO2 RF switch  | disabled                |
/// | Fallback mode   | STDBY_RC                |
/// | TCXO            | none                    |
/// | Calibration     | all blocks              |
///
/// Use the `with_*` builder methods to customise individual parameters.
#[derive(Debug, Clone, defmt::Format)]
pub struct Sx1268Config {
  /// Packet type (GFSK or LoRa).
  pub(super) package_type: PacketType,
  /// RF carrier frequency in Hz.
  pub(super) frequency_hz: u32,
  /// Power Amplifier configuration (duty cycle, hpMax, deviceSel, paLut).
  pub(super) pa_config: PaConfig,
  /// TX output power in dBm (−9 to +22 for SX1268 high-power PA).
  pub(super) tx_power: i8,
  /// PA ramp time — the rise time from PA-off to full power.
  pub(super) ramp_time: RampTime,
  /// LoRa modulation parameters (SF, BW, CR, LDRO).
  pub(super) lora_modulation: LoRaModulationParams,
  /// LoRa packet parameters (preamble, header, payload, CRC, IQ).
  pub(super) lora_packet: LoRaPacketParams,
  /// Voltage regulator mode (LDO or DC-DC + LDO).
  pub(super) regulator_mode: RegulatorMode,
  /// LoRa sync word — `0x3444` for public (LoRaWAN), `0x1424` for private.
  pub(super) lora_sync_word: u16,
  /// Base address of the TX region inside the 256-byte data buffer.
  pub(super) tx_base_address: u8,
  /// Base address of the RX region inside the 256-byte data buffer.
  pub(super) rx_base_address: u8,
  /// When `true`, DIO2 is used to control an external RF switch
  /// (high during TX, low otherwise).
  pub(super) dio2_as_rf_switch: bool,
  /// Operating mode the device enters after TX or RX completes.
  pub(super) fallback_mode: FallbackMode,
  /// Optional TCXO configuration for DIO3. When set, the driver will
  /// call `SetDIO3AsTCXOCtrl` during initialisation and wait for the
  /// TCXO to stabilise.
  pub(super) tcxo: Option<TcxoConfig>,
  /// Bitmask of calibration blocks to run during initialisation
  /// (see [`CalibrationParams`]).
  pub(super) calibration: CalibrationParams,
}

impl Default for Sx1268Config {
  fn default() -> Self {
    Self {
      package_type: PacketType::default(),
      frequency_hz: FREQ_DEFAULT_HZ,
      pa_config: PaConfig::default(),
      tx_power: 0,
      ramp_time: RampTime::Ramp10Us,
      lora_modulation: LoRaModulationParams::default(),
      lora_packet: LoRaPacketParams::default(),
      regulator_mode: RegulatorMode::Ldo,
      lora_sync_word: 0x14,
      tx_base_address: 0x00,
      rx_base_address: 0x00,
      dio2_as_rf_switch: false,
      fallback_mode: FallbackMode::StbyRc,
      tcxo: None,
      calibration: CalibrationParams::ALL,
    }
  }
}

impl Sx1268Config {
  /// Set the packet type to GFSK.
  pub fn with_package_gfsk(mut self) -> Self {
    self.package_type = PacketType::Gfsk;
    self
  }

  /// Set the packet type to LoRa.
  pub fn with_package_lora(mut self) -> Self {
    self.package_type = PacketType::LoRa;
    self
  }

  /// Set the RF carrier frequency in Hz.
  ///
  /// Returns [`ConfigError::InvalidFrequency`] if `freq` is outside the
  /// supported range (410–510 MHz for the SX1268).
  pub fn with_frequency_hz(mut self, freq: u32) -> Result<Self, ConfigError> {
    if !(FREQ_MIN_HZ..=FREQ_MAX_HZ).contains(&freq) {
      Err(ConfigError::InvalidFrequency {
        freq,
        max: FREQ_MAX_HZ,
        min: FREQ_MIN_HZ,
      })
    } else {
      self.frequency_hz = freq;
      Ok(self)
    }
  }

  /// Set the Power Amplifier configuration.
  ///
  /// See [`PaConfig`] for factory-tested presets such as
  /// [`PaConfig::best_22dbm`] and [`PaConfig::best_14dbm`].
  pub fn with_pa_config(mut self, pa_config: PaConfig) -> Self {
    self.pa_config = pa_config;
    self
  }

  /// Set the TX output power in dBm.
  ///
  /// 输出功率以 1 dB 步长定义：
  /// - 低功率 PA：−17 (`0xEF`) ~ +14 (`0x0E`) dBm
  /// - 高功率 PA：−9 (`0xF7`) ~ +22 (`0x16`) dBm
  ///
  /// PA 的选择通过 `SetPaConfig` 命令中的 `deviceSel` 参数完成。
  /// 默认选择低功率 PA，输出 +14 dBm。
  pub fn with_tx_power(mut self, tx_power: i8) -> Self {
    self.tx_power = tx_power;
    self
  }

  /// Set the PA ramp time.
  ///
  /// PA ramp time 定义了功率放大器从关闭到满功率的上升时间。
  /// 较长的上升时间可有效抑制邻道谐波泄漏，但会增加发射前延迟。
  /// 可选值：10 / 20 / 40 / 80 / 200 / 800 / 1700 / 3400 µs。
  /// 默认值为 10 µs。
  pub fn with_ramp_time(mut self, ramp_time: RampTime) -> Self {
    self.ramp_time = ramp_time;
    self
  }

  /// Set the LoRa modulation parameters (SF, BW, CR, LDRO).
  ///
  /// See [`LoRaModulationParams`] for details on each parameter and
  /// trade-offs between range, data rate and airtime.
  pub fn with_lora_modulation(mut self, modulation: LoRaModulationParams) -> Self {
    self.lora_modulation = modulation;
    self
  }

  /// Set the LoRa packet parameters (preamble, header, payload, CRC, IQ).
  pub fn with_lora_packet(mut self, packet: LoRaPacketParams) -> Self {
    self.lora_packet = packet;
    self
  }

  /// Set the voltage regulator mode.
  ///
  /// SX1268 支持两种调节模式：
  /// - **LDO** — 仅使用线性稳压器。电源更干净，但效率较低，适合低功耗场景。
  /// - **DC-DC + LDO** — 主调节由高效 DC-DC 完成，LDO 仅为敏感模块供电。
  ///   效率更高，尤其在高功率输出时优势明显。
  ///
  /// 默认值为 LDO 模式。
  pub fn with_regulator_mode(mut self, mode: RegulatorMode) -> Self {
    self.regulator_mode = mode;
    self
  }

  /// Set the LoRa sync word.
  ///
  /// Sync word 用于区分不同 LoRa 网络：
  /// - `0x3444` — 公共网络（LoRaWAN 使用）
  /// - `0x1424` — 私有网络
  ///
  /// 使用正确的 sync word 可确保设备只与预期网络通信。
  pub fn with_lora_sync_word(mut self, sync_word: u16) -> Self {
    self.lora_sync_word = sync_word;
    self
  }

  /// Shorthand: set the public LoRaWAN sync word (`0x3444`).
  pub fn with_lora_sync_word_public(self) -> Self {
    self.with_lora_sync_word(0x3444)
  }

  /// Shorthand: set the private network sync word (`0x1424`).
  pub fn with_lora_sync_word_private(self) -> Self {
    self.with_lora_sync_word(0x1424)
  }

  /// Set the TX buffer base address.
  ///
  /// TX buffer base address 定义了 256 字节内部缓冲区中发送区域的起始位置。
  /// 正确设置该地址确保待发送数据被加载到正确的缓冲区位置。
  pub fn with_tx_base_address(mut self, address: u8) -> Self {
    self.tx_base_address = address;
    self
  }

  /// Set the RX buffer base address.
  ///
  /// RX buffer base address 定义了 256 字节内部缓冲区中接收区域的起始位置。
  /// 正确设置该地址确保接收数据存储在可被处理器读取的区域。
  pub fn with_rx_base_address(mut self, address: u8) -> Self {
    self.rx_base_address = address;
    self
  }

  /// Enable DIO2 as an RF switch control signal.
  ///
  /// 启用后 DIO2 引脚在 TX 期间输出高电平，其余时间为低电平，可直接驱动
  /// 外部射频开关完成天线的收发切换，简化硬件设计。
  pub fn with_dio2_as_rf_switch(mut self) -> Self {
    self.dio2_as_rf_switch = true;
    self
  }

  /// Set the fallback mode after TX/RX completes.
  ///
  /// Fallback mode 决定了发送或接收结束后设备自动进入的状态：
  /// - **StbyRc** — RC 振荡器待机，功耗最低但唤醒较慢。
  /// - **StbyXosc** — 晶振待机，唤醒更快，适合需要快速响应的场景。
  /// - **Fs** — 频率合成器模式，功耗最高但可实现最快的 TX/RX 切换。
  pub fn with_fallback_mode(mut self, mode: FallbackMode) -> Self {
    self.fallback_mode = mode;
    self
  }

  /// Set the TCXO configuration for DIO3.
  ///
  /// 当使用外部温补晶振（TCXO）作为参考时钟时，必须配置此项。
  /// `voltage` 设置 DIO3 输出的供电电压，`timeout` 为 TCXO 稳定所需
  /// 等待时间（单位 15.625 µs）。正确配置可确保时钟精度从而提升通信可靠性。
  pub fn with_tcxo_config(mut self, voltage: TcxoVoltage, timeout: u32) -> Self {
    self.tcxo = Some(TcxoConfig { voltage, timeout });
    self
  }

  /// Set the calibration blocks to run during initialisation.
  ///
  /// 校准参数位掩码定义了初始化阶段要运行的校准程序（RC64k、RC13M、PLL、
  /// ADC 脉冲、ADC 批量 N/P、图像抑制等），确保器件在各种温度和电压条件下
  /// 保持最佳射频性能。
  pub fn with_calibration(mut self, calibration: CalibrationParams) -> Self {
    self.calibration = calibration;
    self
  }

  // -----------------------------------------------------------------------
  // Read-only accessors
  // -----------------------------------------------------------------------

  /// Get the configured RF frequency in Hz.
  pub fn get_frequency_hz(&self) -> u32 {
    self.frequency_hz
  }

  /// Get the configured RF frequency in MHz (floating-point).
  pub fn get_frequency_mhz(&self) -> f32 {
    self.frequency_hz as f32 / 1_000_000.0
  }

  /// Get the configured TX output power in dBm.
  pub fn get_power_dbm(&self) -> i8 {
    self.tx_power
  }

  /// Get the configured LoRa bandwidth in kHz.
  pub fn get_bandwidth_khz(&self) -> u32 {
    self.lora_modulation.bw.as_khz()
  }

  /// Get the configured LoRa spreading factor as an integer (5–12).
  pub fn get_sf(&self) -> u8 {
    self.lora_modulation.sf.as_sf()
  }

  /// Get the configured LoRa coding rate as a `(numerator, denominator)` tuple.
  pub fn get_cr_ratio(&self) -> (u8, u8) {
    self.lora_modulation.cr.as_cr()
  }

  /// Get the preamble length in symbols.
  pub fn get_preamble_length(&self) -> u16 {
    self.lora_packet.preamble_length
  }

  /// Returns `true` if CRC is enabled on the packet.
  pub fn get_crc_enabled(&self) -> bool {
    self.lora_packet.crc_on
  }

  /// Get the LoRa header type (explicit or implicit).
  pub fn get_header_type(&self) -> LoRaHeaderType {
    self.lora_packet.header_type
  }

  /// Get the LoRa sync word.
  pub fn get_sync_word(&self) -> u16 {
    self.lora_sync_word
  }

  /// Get the PA duty cycle register value.
  pub fn get_pa_duty_cycle(&self) -> u8 {
    self.pa_config.pa_duty_cycle
  }

  /// Get the PA hpMax register value.
  pub fn get_pa_hp_max(&self) -> u8 {
    self.pa_config.hp_max
  }
}
