//! Miscellaneous configuration types shared across the SX1268 driver.

/// PA ramp time — the rise-time of the power amplifier output.
///
/// 较长的 ramp time 可有效抑制频谱旁瓣（邻道干扰），但会增加
/// 从 PA 使能到信号稳定的延迟。根据目标频段法规和系统时序需求选择。
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum RampTime {
  /// 10 µs
  Ramp10Us = 0x00,
  /// 20 µs
  Ramp20Us = 0x01,
  /// 40 µs
  Ramp40Us = 0x02,
  /// 80 µs
  Ramp80Us = 0x03,
  /// 200 µs
  Ramp200Us = 0x04,
  /// 800 µs
  Ramp800Us = 0x05,
  /// 1700 µs
  Ramp1700Us = 0x06,
  /// 3400 µs
  Ramp3400Us = 0x07,
}

/// Voltage regulator mode.
///
/// - **Ldo** — 仅使用 LDO 调节。电源纹波最小但效率较低。
/// - **DcDcLdo** — DC-DC 为主、LDO 为辅。效率更高，推荐在高功率
///   输出场景下使用（需要外部电感）。
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum RegulatorMode {
  /// Only LDO.
  Ldo = 0x00,
  /// DC-DC + LDO (higher efficiency, requires external inductor).
  DcDcLdo = 0x01,
}

/// Fallback mode after TX/RX.
///
/// 设备在发送或接收完成后自动进入的状态，影响功耗和下一次操作的启动延迟。
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum FallbackMode {
  /// Standby with internal RC oscillator (lowest power, slower wakeup).
  StbyRc = 0x20,
  /// Standby with crystal oscillator (fast wakeup).
  StbyXosc = 0x30,
  /// Frequency-synthesis mode (highest power, fastest TX/RX turnaround).
  Fs = 0x40,
}

/// TCXO supply voltage generated on DIO3.
///
/// 当使用 TCXO 作为参考时钟时，SX1268 可通过 DIO3 引脚为 TCXO 供电。
/// 选择与所用 TCXO 器件额定电压匹配的档位。
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum TcxoVoltage {
  /// 1.6 V
  Ctrl1v6 = 0x00,
  /// 1.7 V
  Ctrl1v7 = 0x01,
  /// 1.8 V
  Ctrl1v8 = 0x02,
  /// 2.2 V
  Ctrl2v2 = 0x03,
  /// 2.4 V
  Ctrl2v4 = 0x04,
  /// 2.7 V
  Ctrl2v7 = 0x05,
  /// 3.0 V
  Ctrl3v0 = 0x06,
  /// 3.3 V
  Ctrl3v3 = 0x07,
}

/// Optional TCXO configuration for DIO3.
///
/// 当外部使用温补晶振（TCXO）时，此结构体指定 DIO3 输出电压和
/// TCXO 稳定等待超时。初始化阶段驱动会先配置 TCXO 并等待其稳定后
/// 再执行校准。
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct TcxoConfig {
  /// TCXO supply voltage.
  pub voltage: TcxoVoltage,
  /// Stabilisation timeout in units of 15.625 µs.
  pub timeout: u32,
}

/// Calibration block selection bitmask.
///
/// 每一位对应 SX1268 内部的一个校准模块。在初始化时执行校准
/// 可确保器件在不同温度和电压条件下保持最佳射频性能。
///
/// 常量 [`ALL`](Self::ALL) (`0x7F`) 校准全部模块。
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct CalibrationParams(pub u8);

impl CalibrationParams {
  /// 64 kHz RC oscillator calibration.
  pub const RC64K: Self = CalibrationParams(1 << 0);
  /// 13 MHz RC oscillator calibration.
  pub const RC13M: Self = CalibrationParams(1 << 1);
  /// PLL calibration.
  pub const PLL: Self = CalibrationParams(1 << 2);
  /// ADC pulse calibration.
  pub const ADC_PULSE: Self = CalibrationParams(1 << 3);
  /// ADC bulk N calibration.
  pub const ADC_BULK_N: Self = CalibrationParams(1 << 4);
  /// ADC bulk P calibration.
  pub const ADC_BULK_P: Self = CalibrationParams(1 << 5);
  /// Image rejection calibration.
  pub const IMAGE: Self = CalibrationParams(1 << 6);
  /// Calibrate **all** blocks.
  pub const ALL: Self = CalibrationParams(0x7F);
}

/// Sleep configuration.
///
/// 进入 Sleep 模式后器件功耗最低。通过 `warm_start` 可保留寄存器
/// 配置（热启动），`rtc_wakeup` 使能 RTC 定时唤醒。
#[derive(Clone, Copy, Debug, Default, defmt::Format)]
pub struct SleepConfig {
  /// If `true`, the device retains its configuration in sleep (warm start).
  /// Otherwise a cold start is performed on wakeup.
  pub warm_start: bool,
  /// If `true`, the internal RTC timer is used to wake the device after a
  /// configurable period.
  pub rtc_wakeup: bool,
}

impl SleepConfig {
  /// Encode the sleep configuration into the single byte expected by the
  /// `SetSleep` command.
  ///
  /// Bit layout: `[7:3] reserved | [2] warm_start | [1] reserved | [0] rtc_wakeup`.
  pub fn to_byte(self) -> u8 {
    let mut val = 0u8;
    if self.warm_start {
      val |= 1 << 2;
    }
    if self.rtc_wakeup {
      val |= 1;
    }
    val
  }
}

/// Standby mode oscillator selection.
///
/// - **StbyRc** — 13 MHz 内部 RC 振荡器，功耗低但频率精度较差。
/// - **StbyXosc** — 32 MHz 外部晶振，频率更精确、唤醒更快。
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum StandbyConfig {
  /// Standby with internal 13 MHz RC oscillator.
  StbyRc = 0x00,
  /// Standby with external 32 MHz crystal oscillator.
  StbyXosc = 0x01,
}

/// Number of symbols used for Channel Activity Detection (CAD).
///
/// 更多的检测符号提高了 CAD 灵敏度（降低漏检概率），但也增加了
/// CAD 持续时间和功耗。
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum CadSymbols {
  /// 1 symbol
  Cad1Symbol = 0x00,
  /// 2 symbols
  Cad2Symbols = 0x01,
  /// 4 symbols
  Cad4Symbols = 0x02,
  /// 8 symbols
  Cad8Symbols = 0x03,
  /// 16 symbols
  Cad16Symbols = 0x04,
}

/// CAD exit mode — what the device does after CAD completes.
///
/// - **CadOnly** — CAD 完成后设备返回 STDBY_RC，无论是否检测到活动。
/// - **CadRx** — 如果检测到信道活动，自动进入 RX 模式接收数据包。
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum CadExitMode {
  /// Return to standby after CAD regardless of result.
  CadOnly = 0x00,
  /// Enter RX mode if activity is detected.
  CadRx = 0x01,
}
