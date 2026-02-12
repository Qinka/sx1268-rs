/// LoRa spreading factor.
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
  /// Get spreading factor as integer.
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

/// LoRa bandwidth.
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
  /// Get bandwidth in kHz.
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

/// LoRa coding rate.
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
  /// Get coding rate as fraction.
  pub fn as_cr(&self) -> (u8, u8) {
    match self {
      LoRaCodingRate::Cr4_5 => (4, 5),
      LoRaCodingRate::Cr4_6 => (4, 6),
      LoRaCodingRate::Cr4_7 => (4, 7),
      LoRaCodingRate::Cr4_8 => (4, 8),
    }
  }
}

/// LoRa modulation parameters.
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
  /// setup spreading factor.
  /// 扩频因子（Spreading Factor, SF）定义了一个符号中所包含的码片数量，LoRa 通常支持 SF6 至 SF12。
  /// SF 的变化直接影响信号的处理增益：
  /// SF 数值越大，单位比特被扩展得越长，接收端越容易从噪声中恢复信号
  /// 相应代价是符号时间显著拉长，数据速率下降，空中占用时间增加
  pub fn with_spreading_factor(mut self, sf: LoRaSpreadingFactor) -> Self {
    self.sf = sf;
    self
  }

  /// setup bandwidth.
  pub fn with_bandwidth(mut self, bw: LoRaBandwidth) -> Self {
    self.bw = bw;
    self
  }

  /// setup coding rate.
  pub fn with_coding_rate(mut self, cr: LoRaCodingRate) -> Self {
    self.cr = cr;
    self
  }

  /// setup low data rate optimization.
  pub fn with_low_data_rate_optimize(mut self, optimize: bool) -> Self {
    self.low_data_rate_optimize = optimize;
    self
  }
}
