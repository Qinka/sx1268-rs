/// PA (Power Amplifier) configuration.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct PaConfig {
  /// PA duty cycle (raw register value).
  /// paDutyCycle 控制占空比（导通角）。
  /// 最大输出功率、功耗和谐波会随着 paDutyCycle 的变化而剧烈改变。
  /// 本数据手册中给出的数值是达到 PA 最佳效率的推荐设置。
  /// 改变 paDutyCycle 会影响谐波中的功率分布，应与给定的匹配网络协同选择。
  pub(crate) pa_duty_cycle: u8,
  /// HP max (raw register value).
  /// hpMax 选择 PA 的大小。通过减小 hpMax 的值可以降低最大输出功率。
  /// 有效范围在 0x00 到 0x07 之间，0x07 是实现 +22 dBm 输出功率的最大支持值。
  /// 将 hpMax 超过 0x07 可能会导致器件早期老化，在极端温度下使用时可能损坏器件。
  pub(crate) hp_max: u8,
  /// Device selection: 0x00 = SX1268.
  pub(crate) device_sel: u8,
  /// PA LUT (always 0x01).
  pub(crate) pa_lut: u8,
}

impl Default for PaConfig {
  fn default() -> Self {
    Self::best_22dbm()
  }
}

impl PaConfig {
  pub fn best_22dbm() -> Self {
    Self {
      pa_duty_cycle: 0x04,
      hp_max: 0x07,
      device_sel: 0x00,
      pa_lut: 0x01,
    }
  }

  pub fn best_20dbm() -> Self {
    Self {
      pa_duty_cycle: 0x03,
      hp_max: 0x05,
      device_sel: 0x00,
      pa_lut: 0x01,
    }
  }

  pub fn best_17dbm() -> Self {
    Self {
      pa_duty_cycle: 0x02,
      hp_max: 0x03,
      device_sel: 0x00,
      pa_lut: 0x01,
    }
  }

  pub fn best_14dbm() -> Self {
    Self {
      pa_duty_cycle: 0x04,
      hp_max: 0x06,
      device_sel: 0x00,
      pa_lut: 0x01,
    }
  }

  pub fn best_10dbm() -> Self {
    Self {
      pa_duty_cycle: 0x00,
      hp_max: 0x03,
      device_sel: 0x00,
      pa_lut: 0x01,
    }
  }
}
