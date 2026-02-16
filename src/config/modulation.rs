// 该文件是 sx1268-rs 项目的一部分。
// src/config/modulation.rs - LoRa 调制参数
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

//! LoRa modulation parameters.
//!
//! LoRa uses Chirp Spread Spectrum (CSS) modulation. Three primary
//! parameters control the trade-off between range, data rate and
//! airtime:
//!
//! | Parameter           | Effect when increased |
//! |---------------------|-----------------------|
//! | Spreading Factor    | ↑ range, ↓ data rate  |
//! | Bandwidth           | ↑ data rate, ↓ range  |
//! | Coding Rate (denom) | ↑ resilience, ↓ data rate |
//!
//! Additionally, **Low Data Rate Optimization (LDRO)** must be enabled
//! when the symbol duration exceeds 16.38 ms (i.e. SF11/BW125 or
//! SF12/BW125).

/// LoRa spreading factor (SF5–SF12).
///
/// 扩频因子（Spreading Factor, SF）定义了单个符号中包含的 chirp 数量
/// （`2^SF` 个 chirp/symbol）。更高的 SF 提供更大的处理增益（更远的
/// 通信距离），但每个符号的空中时间也成倍增长，因而降低有效数据速率。
///
/// 不同 SF 之间具有正交性——同一频率上使用不同 SF 的信号互不干扰，
/// 这是 LoRaWAN 网络容量管理的基础。
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum LoRaSpreadingFactor {
  Sf5 = 0x05,
  Sf6 = 0x06,
  Sf7 = 0x07,
  Sf8 = 0x08,
  Sf9 = 0x09,
  Sf10 = 0x0A,
  Sf11 = 0x0B,
  Sf12 = 0x0C,
}

impl LoRaSpreadingFactor {
  /// Return the spreading factor as a plain integer (5–12).
  pub fn as_sf(&self) -> u8 {
    match self {
      LoRaSpreadingFactor::Sf5 => 5,
      LoRaSpreadingFactor::Sf6 => 6,
      LoRaSpreadingFactor::Sf7 => 7,
      LoRaSpreadingFactor::Sf8 => 8,
      LoRaSpreadingFactor::Sf9 => 9,
      LoRaSpreadingFactor::Sf10 => 10,
      LoRaSpreadingFactor::Sf11 => 11,
      LoRaSpreadingFactor::Sf12 => 12,
    }
  }
}

/// LoRa signal bandwidth.
///
/// 带宽（BW）决定了 chirp 扫频的频率范围。更大的带宽使符号时间更短，
/// 从而提升数据速率并降低对时钟误差的敏感度；但占用更多频谱且接收灵敏度
/// 下降（噪底升高）。
///
/// SX1268 支持从 7.81 kHz 到 500 kHz 的多档带宽选择。
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum LoRaBandwidth {
  /// 7.81 kHz
  Bw7 = 0x00,
  /// 10.42 kHz
  Bw10 = 0x08,
  /// 15.63 kHz
  Bw15 = 0x01,
  /// 20.83 kHz
  Bw20 = 0x09,
  /// 31.25 kHz
  Bw31 = 0x02,
  /// 41.67 kHz
  Bw41 = 0x0A,
  /// 62.5 kHz
  Bw62 = 0x03,
  /// 125 kHz
  Bw125 = 0x04,
  /// 250 kHz
  Bw250 = 0x05,
  /// 500 kHz
  Bw500 = 0x06,
}

impl LoRaBandwidth {
  /// Return the bandwidth as an approximate integer in kHz.
  pub fn as_khz(&self) -> u32 {
    match self {
      LoRaBandwidth::Bw7 => 7,
      LoRaBandwidth::Bw10 => 10,
      LoRaBandwidth::Bw15 => 15,
      LoRaBandwidth::Bw20 => 20,
      LoRaBandwidth::Bw31 => 31,
      LoRaBandwidth::Bw41 => 41,
      LoRaBandwidth::Bw62 => 62,
      LoRaBandwidth::Bw125 => 125,
      LoRaBandwidth::Bw250 => 250,
      LoRaBandwidth::Bw500 => 500,
    }
  }
}

/// LoRa forward error-correction coding rate.
///
/// 编码率（Coding Rate, CR）定义了前向纠错（FEC）冗余。LoRa 使用 4/N
/// 编码，其中 N = 5 ~ 8。分母越大，冗余越多，抗干扰能力越强，但有效
/// 数据速率越低（空中时间增加）。
///
/// 在显式报头模式下 CR 会被写入报头，接收端自动匹配；在隐式报头模式下
/// 收发两端必须配置相同的 CR。
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum LoRaCodingRate {
  /// 4/5
  Cr4_5 = 0x01,
  /// 4/6
  Cr4_6 = 0x02,
  /// 4/7
  Cr4_7 = 0x03,
  /// 4/8
  Cr4_8 = 0x04,
}

impl LoRaCodingRate {
  /// Return the coding rate as a `(numerator, denominator)` tuple.
  pub fn as_cr(&self) -> (u8, u8) {
    match self {
      LoRaCodingRate::Cr4_5 => (4, 5),
      LoRaCodingRate::Cr4_6 => (4, 6),
      LoRaCodingRate::Cr4_7 => (4, 7),
      LoRaCodingRate::Cr4_8 => (4, 8),
    }
  }
}

/// Combined LoRa modulation parameters.
///
/// 四个参数共同决定了 LoRa 链路的物理层特性：
///
/// - **sf** — 扩频因子，控制处理增益（范围/灵敏度 vs 数据速率）
/// - **bw** — 带宽，控制符号速率和噪底
/// - **cr** — 编码率，控制前向纠错冗余
/// - **low_data_rate_optimize** — 当符号时间 > 16.38 ms 时必须启用
///   （即 SF11/BW125 或 SF12/BW125）
///
/// 默认值：SF7 / BW125 / CR4_5 / LDRO off — 这是最常用的 LoRa 配置
/// 之一，在数据速率和链路预算之间取得良好平衡。
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct LoRaModulationParams {
  pub(crate) sf: LoRaSpreadingFactor,
  pub(crate) bw: LoRaBandwidth,
  pub(crate) cr: LoRaCodingRate,
  pub(crate) low_data_rate_optimize: bool,
}

impl Default for LoRaModulationParams {
  fn default() -> Self {
    Self {
      sf: LoRaSpreadingFactor::Sf7,
      bw: LoRaBandwidth::Bw125,
      cr: LoRaCodingRate::Cr4_5,
      low_data_rate_optimize: false,
    }
  }
}

impl LoRaModulationParams {
  /// Set the spreading factor.
  ///
  /// 扩频因子（SF）定义了每个符号中的码片数 (2^SF chips/symbol)。
  /// SF 越大，接收灵敏度越高（更远距离），但符号时间成倍增长，
  /// 数据速率下降，空中占用时间增加。
  pub fn with_spreading_factor(mut self, sf: LoRaSpreadingFactor) -> Self {
    self.sf = sf;
    self
  }

  /// Set the signal bandwidth.
  ///
  /// 带宽越大，符号时间越短、数据速率越高，但接收灵敏度下降（噪底升高）。
  /// 常用组合：SF7/BW125（高速）、SF12/BW125（远距离）。
  pub fn with_bandwidth(mut self, bw: LoRaBandwidth) -> Self {
    self.bw = bw;
    self
  }

  /// Set the forward error-correction coding rate.
  ///
  /// 编码率分母越大，冗余越多，纠错能力越强，但有效数据速率越低。
  /// 通常 CR4/5 已足够，仅在干扰严重时选择更高冗余。
  pub fn with_coding_rate(mut self, cr: LoRaCodingRate) -> Self {
    self.cr = cr;
    self
  }

  /// Enable or disable Low Data Rate Optimization (LDRO).
  ///
  /// 当符号持续时间超过 16.38 ms 时（如 SF11/BW125 或 SF12/BW125），
  /// 必须开启 LDRO 以避免因时钟漂移导致的解调失败。
  pub fn with_low_data_rate_optimize(mut self, optimize: bool) -> Self {
    self.low_data_rate_optimize = optimize;
    self
  }
}
