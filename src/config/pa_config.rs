//! Power Amplifier (PA) configuration.
//!
//! The SX1268 contains a high-power PA capable of up to +22 dBm output.
//! Its behaviour is governed by four register parameters that are sent
//! via the `SetPaConfig` command (opcode `0x95`):
//!
//! | Field          | Purpose |
//! |----------------|---------|
//! | `paDutyCycle`  | Controls the PA conduction angle (duty cycle). |
//! | `hpMax`        | Selects the PA size (0x00–0x07). |
//! | `deviceSel`    | `0x00` for SX1268 (high-power), `0x01` for SX1261. |
//! | `paLut`        | Always `0x01` (reserved). |
//!
//! Semtech provides a set of *recommended* combinations in the
//! datasheet (Table 13-21) to achieve specific maximum output powers
//! with optimal efficiency.  These are exposed as named constructors
//! such as [`PaConfig::best_22dbm`].
//!
//! **Warning:** Setting `hpMax` above `0x07` may cause premature device
//! ageing or permanent damage under extreme temperatures.

/// PA (Power Amplifier) configuration.
///
/// Use the named constructors ([`best_22dbm`](Self::best_22dbm),
/// [`best_20dbm`](Self::best_20dbm), etc.) to select a Semtech-recommended
/// parameter combination for the desired maximum output power.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct PaConfig {
  /// PA duty cycle (raw register value).
  ///
  /// `paDutyCycle` 控制功放的导通占空比。
  /// 占空比会显著影响最大输出功率、整体功耗和谐波分布。
  /// 数据手册中给出的值是经过优化以在推荐匹配网络下获得最佳效率的设置。
  pub(crate) pa_duty_cycle: u8,

  /// HP max — selects the PA size.
  ///
  /// `hpMax` 选择功率放大器的大小。减小该值可降低最大输出功率。
  /// 有效范围 0x00–0x07，其中 0x07 对应 +22 dBm 最大输出。
  /// **切勿超过 0x07**，否则可能导致器件加速老化甚至在高温下损坏。
  pub(crate) hp_max: u8,

  /// Device selection: `0x00` = SX1268 (high-power PA).
  pub(crate) device_sel: u8,

  /// PA LUT configuration (always `0x01`, reserved).
  pub(crate) pa_lut: u8,
}

impl Default for PaConfig {
  /// Default: +22 dBm optimal configuration.
  fn default() -> Self {
    Self::best_22dbm()
  }
}

impl PaConfig {
  /// Recommended PA settings for **+22 dBm** maximum output.
  ///
  /// `paDutyCycle=0x04, hpMax=0x07` — 数据手册推荐的 +22 dBm 最优配置。
  pub fn best_22dbm() -> Self {
    Self {
      pa_duty_cycle: 0x04,
      hp_max: 0x07,
      device_sel: 0x00,
      pa_lut: 0x01,
    }
  }

  /// Recommended PA settings for **+20 dBm** maximum output.
  ///
  /// `paDutyCycle=0x03, hpMax=0x05` — 比 +22 dBm 配置略低的功耗。
  pub fn best_20dbm() -> Self {
    Self {
      pa_duty_cycle: 0x03,
      hp_max: 0x05,
      device_sel: 0x00,
      pa_lut: 0x01,
    }
  }

  /// Recommended PA settings for **+17 dBm** maximum output.
  ///
  /// `paDutyCycle=0x02, hpMax=0x03`.
  pub fn best_17dbm() -> Self {
    Self {
      pa_duty_cycle: 0x02,
      hp_max: 0x03,
      device_sel: 0x00,
      pa_lut: 0x01,
    }
  }

  /// Recommended PA settings for **+14 dBm** maximum output.
  ///
  /// `paDutyCycle=0x04, hpMax=0x06`.
  pub fn best_14dbm() -> Self {
    Self {
      pa_duty_cycle: 0x04,
      hp_max: 0x06,
      device_sel: 0x00,
      pa_lut: 0x01,
    }
  }

  /// Recommended PA settings for **+10 dBm** maximum output.
  ///
  /// `paDutyCycle=0x00, hpMax=0x03` — 最低功耗的预设配置。
  pub fn best_10dbm() -> Self {
    Self {
      pa_duty_cycle: 0x00,
      hp_max: 0x03,
      device_sel: 0x00,
      pa_lut: 0x01,
    }
  }
}
