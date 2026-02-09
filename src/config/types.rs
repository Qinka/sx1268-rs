/// Ramp time for PA.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum RampTime {
  Ramp10Us = 0x00,
  Ramp20Us = 0x01,
  Ramp40Us = 0x02,
  Ramp80Us = 0x03,
  Ramp200Us = 0x04,
  Ramp800Us = 0x05,
  Ramp1700Us = 0x06,
  Ramp3400Us = 0x07,
}

/// Regulator mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum RegulatorMode {
  /// Only LDO.
  Ldo = 0x00,
  /// DC-DC + LDO.
  DcDcLdo = 0x01,
}

/// Fallback mode after TX/RX.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum FallbackMode {
  StbyRc = 0x20,
  StbyXosc = 0x30,
  Fs = 0x40,
}

/// TCXO voltage setting for DIO3.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum TcxoVoltage {
  Ctrl1v6 = 0x00,
  Ctrl1v7 = 0x01,
  Ctrl1v8 = 0x02,
  Ctrl2v2 = 0x03,
  Ctrl2v4 = 0x04,
  Ctrl2v7 = 0x05,
  Ctrl3v0 = 0x06,
  Ctrl3v3 = 0x07,
}

/// Optional TCXO configuration for DIO3.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct TcxoConfig {
  /// TCXO supply voltage.
  pub voltage: TcxoVoltage,
  /// Timeout in units of 15.625 µs for the TCXO to stabilize.
  pub timeout: u32,
}

/// Calibration parameters bitmask.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct CalibrationParams(pub u8);

impl CalibrationParams {
  pub const RC64K: Self = CalibrationParams(1 << 0);
  pub const RC13M: Self = CalibrationParams(1 << 1);
  pub const PLL: Self = CalibrationParams(1 << 2);
  pub const ADC_PULSE: Self = CalibrationParams(1 << 3);
  pub const ADC_BULK_N: Self = CalibrationParams(1 << 4);
  pub const ADC_BULK_P: Self = CalibrationParams(1 << 5);
  pub const IMAGE: Self = CalibrationParams(1 << 6);
  pub const ALL: Self = CalibrationParams(0x7F);
}

/// Sleep configuration.
#[derive(Clone, Copy, Debug, Default, defmt::Format)]
pub struct SleepConfig {
  /// If true, device retains configuration in sleep (warm start).
  pub warm_start: bool,
  /// If true, RTC timeout is enabled.
  pub rtc_wakeup: bool,
}

impl SleepConfig {
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

/// Standby mode configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum StandbyConfig {
  /// Standby with RC oscillator (13 MHz).
  StbyRc = 0x00,
  /// Standby with crystal oscillator (32 MHz).
  StbyXosc = 0x01,
}

/// CAD detection parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum CadSymbols {
  Cad1Symbol = 0x00,
  Cad2Symbols = 0x01,
  Cad4Symbols = 0x02,
  Cad8Symbols = 0x03,
  Cad16Symbols = 0x04,
}

/// CAD exit mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum CadExitMode {
  CadOnly = 0x00,
  CadRx = 0x01,
}
