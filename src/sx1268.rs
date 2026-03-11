// 该文件是 sx1268-rs 项目的一部分。
// src/sx1268.rs - 核心驱动实现
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

//! Core SX1268 driver implementation.
//!
//! This module contains the [`Sx1268`] struct and all methods for
//! initialising the radio, configuring operational modes, transmitting
//! and receiving LoRa packets, and reading diagnostic status.

#[cfg(feature = "std")]
use std::fmt::{self, Display};

#[cfg(feature = "no_std")]
use defmt::{debug, info, warn};
#[cfg(feature = "std")]
use tracing::{debug, info, warn};

use crate::codes;
use crate::config::*;
use crate::control::Control;

/// Driver error type.
///
/// Wraps low-level bus errors from the [`Control`] implementation as well
/// as configuration and protocol-level errors detected by the driver.
#[cfg_attr(feature = "std", derive(Debug))]
#[cfg_attr(feature = "no_std", derive(Debug, defmt::Format))]
pub enum Error<S> {
  /// A configuration-time error (e.g. out-of-range frequency).
  ConfigError(ConfigError),
  /// An error propagated from the underlying [`Control`] implementation.
  ControlError(S),
  /// The device returned an unexpected or invalid status byte.
  InvalidStatus,
  /// An operation exceeded the permitted timeout.
  Timeout,
  /// A parameter passed to a driver method is invalid.
  InvalidParameter,
}

impl<E> Error<E> {
  /// Wrap a raw bus error into [`Error::ControlError`].
  pub fn from_spi_error(e: E) -> Self {
    Error::ControlError(e)
  }
}

#[cfg(feature = "std")]
impl<S> Display for Error<S> {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    match self {
      Error::ConfigError(e) => write!(f, "Config error: {}", e),
      Error::ControlError(_) => write!(f, "Control error"),
      Error::InvalidStatus => write!(f, "Invalid status"),
      Error::Timeout => write!(f, "Timeout"),
      Error::InvalidParameter => write!(f, "Invalid parameter"),
    }
  }
}

#[cfg(feature = "std")]
impl<E> std::error::Error for Error<E> where E: std::error::Error {}

/// IRQ source bitmask values.
///
/// Each constant corresponds to one or more bits in the 16-bit IRQ
/// status register. Use [`All`](Self::All) to match / clear every flag.
#[cfg_attr(feature = "std", derive(Debug, Copy, Clone, PartialEq, Eq))]
#[cfg_attr(
  feature = "no_std",
  derive(Debug, defmt::Format, Copy, Clone, PartialEq, Eq)
)]
pub struct IrqMasks(u16);

#[allow(non_upper_case_globals)]
impl IrqMasks {
  /// No IRQ.
  pub const None: Self = Self(0);
  /// Packet transmission completed.
  pub const TxDone: Self = Self(0x0001);
  /// Packet reception completed.
  pub const RxDone: Self = Self(0x0002);
  /// Preamble detected during RX.
  pub const PreambleDetected: Self = Self(0x0004);
  /// Valid sync word detected (GFSK only).
  pub const SyncWordValid: Self = Self(0x0008);
  /// Valid LoRa header received.
  pub const HeaderValid: Self = Self(0x0010);
  /// LoRa header CRC error.
  pub const HeaderError: Self = Self(0x0020);
  /// Payload CRC error.
  pub const CrcError: Self = Self(0x0040);
  /// Channel Activity Detection completed.
  pub const CadDone: Self = Self(0x0080);
  /// Channel activity was detected during CAD.
  pub const CadDetected: Self = Self(0x0100);
  /// RX or TX timeout.
  pub const Timeout: Self = Self(0x0200);
  /// All IRQ flags.
  pub const All: Self = Self(0x03FF);

  fn bits(self) -> u16 {
    self.0
  }

  fn intersects(self, bits: u16) -> bool {
    bits & self.bits() != 0
  }
}

use core::ops::{BitOr, BitOrAssign};

impl BitOr for IrqMasks {
  type Output = Self;
  fn bitor(self, rhs: Self) -> Self::Output {
    Self(self.bits() | rhs.bits())
  }
}

impl BitOrAssign for IrqMasks {
  fn bitor_assign(&mut self, rhs: Self) {
    *self = *self | rhs;
  }
}

/// Chip operating mode, extracted from the status byte (bits 6:4).
#[cfg_attr(feature = "std", derive(Debug, Copy, Clone, PartialEq, Eq))]
#[cfg_attr(
  feature = "no_std",
  derive(Debug, defmt::Format, Copy, Clone, PartialEq, Eq)
)]
pub enum ChipMode {
  /// Standby — RC oscillator.
  StbyRc = 0x02,
  /// Standby — crystal oscillator.
  StbyXosc = 0x03,
  /// Frequency-synthesis mode.
  Fs = 0x04,
  /// Receive mode.
  Rx = 0x05,
  /// Transmit mode.
  Tx = 0x06,
  /// Unknown or reserved chip mode value.
  Unknown,
}

impl From<u8> for ChipMode {
  fn from(val: u8) -> Self {
    match val {
      0x02 => ChipMode::StbyRc,
      0x03 => ChipMode::StbyXosc,
      0x04 => ChipMode::Fs,
      0x05 => ChipMode::Rx,
      0x06 => ChipMode::Tx,
      _ => ChipMode::Unknown,
    }
  }
}

/// Last command status, extracted from the status byte (bits 3:1).
#[cfg_attr(feature = "std", derive(Clone, Copy, Debug, PartialEq, Eq))]
#[cfg_attr(
  feature = "no_std",
  derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)
)]
pub enum CommandStatus {
  /// Data is available to be read.
  DataAvailable = 0x02,
  /// The last command timed out.
  CommandTimeout = 0x03,
  /// A processing error occurred.
  CommandProcessingError = 0x04,
  /// The command could not be executed.
  FailureToExecute = 0x05,
  /// A TX operation completed.
  CommandTxDone = 0x06,
  /// Unknown or reserved command status value.
  Unknown,
}

impl From<u8> for CommandStatus {
  fn from(val: u8) -> Self {
    match val {
      0x02 => CommandStatus::DataAvailable,
      0x03 => CommandStatus::CommandTimeout,
      0x04 => CommandStatus::CommandProcessingError,
      0x05 => CommandStatus::FailureToExecute,
      0x06 => CommandStatus::CommandTxDone,
      _ => CommandStatus::Unknown,
    }
  }
}

/// Decoded device status byte.
///
/// The SX1268 returns a status byte on every SPI transaction.  It
/// encodes the current operating mode and the outcome of the last
/// command:
///
/// ```text
/// Bit 7 | 6:4       | 3:1            | 0
/// ------|-----------|----------------|----
/// Rsvd  | ChipMode  | CommandStatus  | Rsvd
/// ```
#[cfg_attr(feature = "std", derive(Debug, Copy, Clone))]
#[cfg_attr(feature = "no_std", derive(Debug, defmt::Format, Copy, Clone))]
pub struct Status {
  /// Current chip operating mode.
  pub chip_mode: ChipMode,
  /// Status of the last executed command.
  pub command_status: CommandStatus,
}

impl From<u8> for Status {
  fn from(val: u8) -> Self {
    Status {
      chip_mode: ChipMode::from((val >> 4) & 0x07),
      command_status: CommandStatus::from((val >> 1) & 0x07),
    }
  }
}

/// RX buffer status returned by `GetRxBufferStatus` (opcode `0x13`).
#[cfg_attr(feature = "std", derive(Clone, Copy, Debug))]
#[cfg_attr(feature = "no_std", derive(Clone, Copy, Debug, defmt::Format))]
pub struct RxBufferStatus {
  /// Number of bytes in the last received payload.
  pub payload_length: u8,
  /// Start offset of the payload inside the 256-byte data buffer.
  pub buffer_start_pointer: u8,
}

/// LoRa packet status returned by `GetPacketStatus` (opcode `0x14`).
#[cfg_attr(feature = "std", derive(Clone, Copy, Debug))]
#[cfg_attr(feature = "no_std", derive(Clone, Copy, Debug, defmt::Format))]
pub struct LoRaPacketStatus {
  /// Average RSSI over the last received packet (dBm).
  pub rssi_pkt: i16,
  /// Estimated SNR on the last received packet (dB).
  pub snr_pkt: i8,
  /// RSSI of the LoRa signal (dBm).
  pub signal_rssi_pkt: i16,
}

#[cfg(feature = "std")]
impl Display for RxBufferStatus {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      f,
      "RxBufferStatus {{ payload_length: {}, buffer_start_pointer: {} }}",
      self.payload_length, self.buffer_start_pointer
    )
  }
}

#[cfg(feature = "std")]
impl Display for LoRaPacketStatus {
  fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
    write!(
      f,
      "LoRaPacketStatus {{ rssi_pkt: {}, snr_pkt: {}, signal_rssi_pkt: {} }}",
      self.rssi_pkt, self.snr_pkt, self.signal_rssi_pkt
    )
  }
}

/// SX1268 LoRa transceiver driver.
///
/// Generic over a [`Control`] implementation that provides the low-level
/// SPI and GPIO operations.  After construction with [`new`](Self::new),
/// call [`init`](Self::init) with an [`Sx1268Config`] to bring the radio
/// to an operational state.
pub struct Sx1268<C> {
  control: C,
  /// Retained configuration — set by [`init`](Self::init) and used by
  /// higher-level methods like [`send_lora`](Self::send_lora).
  config: Option<Sx1268Config>,
}

impl<C, E> Sx1268<C>
where
  C: Control<Error = Error<E>>,
{
  /// Crystal oscillator frequency (32 MHz).
  const SX126X_XTAL_FREQ: u32 = 32_000_000;

  /// Internal RTC frequency used for timeout calculations (64 kHz).
  const SX126X_RTC_FREQ_IN_HZ: u32 = 64000;

  /// Number of fractional bits used in the PLL step calculation.
  const SX126X_PLL_STEP_SHIFT_AMOUNT: u32 = 14;

  /// Pre-scaled PLL step size.
  ///
  /// ```text
  /// PLL_STEP_SCALED = F_XTAL >> (25 - PLL_STEP_SHIFT_AMOUNT)
  /// ```
  ///
  /// This constant avoids a 64-bit divide at runtime; instead
  /// [`convert_freq_in_hz_to_pll_step`](Self::convert_freq_in_hz_to_pll_step)
  /// uses only 32-bit arithmetic.
  const SX126X_PLL_STEP_SCALED: u32 =
    (Self::SX126X_XTAL_FREQ >> (25 - Self::SX126X_PLL_STEP_SHIFT_AMOUNT));

  /// Create a new SX1268 driver instance.
  pub fn new(control: C) -> Self {
    Self {
      control,
      config: None,
    }
  }

  /// Initialise the SX1268 with the given configuration.
  ///
  /// This method performs the full chip bring-up sequence recommended by
  /// the SX1268 datasheet:
  ///
  /// 1. **Hardware reset** — pulse NRESET via [`Control::reset`].
  /// 2. **Wakeup** — bring the chip out of sleep via [`Control::wakeup`].
  /// 3. **Standby (RC)** — enter STDBY_RC as the base configuration state.
  /// 4. **Regulator** — select LDO or DC-DC+LDO.
  /// 5. **DIO2 RF switch** — optionally enable DIO2 for antenna switch.
  /// 6. **TCXO** — optionally configure DIO3 for TCXO supply.
  /// 7. **Calibrate** — run the selected calibration blocks.
  /// 8. **Packet type** — set GFSK or LoRa.
  /// 9. **RF frequency** — program the PLL.
  /// 10. **PA config** — configure duty cycle and HP max.
  /// 11. **TX params** — set output power and ramp time.
  /// 12. **Fallback mode** — define post-TX/RX state.
  /// 13. **RX gain** — disable boosted gain (power-saving default).
  /// 14. **Modulation params** — SF, BW, CR, LDRO.
  /// 15. **Packet params** — preamble, header, payload, CRC, IQ.
  /// 16. **Sync word** — configure the LoRa network sync word.
  ///
  /// After a successful return the radio is in STDBY_RC and ready for
  /// [`send_lora`](Self::send_lora) or
  /// [`wait_lora_packet`](Self::wait_lora_packet).
  pub fn init(&mut self, config: Sx1268Config) -> Result<(), Error<E>> {
    info!("Initializing SX1268");
    debug!("Config={:?}", config);

    // 硬件复位
    self.control.reset()?;
    info!("Hardware reset complete ...");

    // 唤醒设备
    self.control.wakeup()?;
    info!("Device wakeup complete ...");

    // 进入 standby 模式，使用内部 RC 作为时钟源，等待后续配置
    self.set_standby(StandbyConfig::StbyRc)?;
    info!("Device reset complete");

    // TODO: 保留寄存器设置
    // sx126x_init_retention_list( &context_e22 );

    // 内部电源模式 (DCDC功耗更小)
    self.set_regulator_mode(config.regulator_mode)?;
    info!("Regulator mode set to {}", config.regulator_mode);

    // DIO2切换射频开关
    self.set_dio2_as_rf_switch_ctrl(config.dio2_as_rf_switch)?;
    info!(
      "DIO2 as RF switch control {}",
      if config.dio2_as_rf_switch {
        "enabled"
      } else {
        "disabled"
      }
    );

    if let Some(tcxo) = config.tcxo {
      self.set_dio3_as_tcxo_ctrl(tcxo.voltage, tcxo.timeout)?;
      info!("TCXO configured on DIO3");
    }

    // 修正内部状态
    self.calibrate(config.calibration)?;
    info!("Calibration complete");

    self.set_packet_type(config.package_type)?;
    info!("Packet type set to {}", config.package_type);

    self.set_rf_frequency(config.frequency_hz)?;
    info!("RF frequency set to {} Hz", config.frequency_hz);

    self.set_pa_config(config.pa_config)?;
    info!("PA config set to {}", config.pa_config);

    self.set_tx_params(config.tx_power, config.ramp_time)?;
    info!(
      "TX params set to power={}dBm ramp={}",
      config.tx_power, config.ramp_time
    );

    self.set_rx_tx_fallback_mode(config.fallback_mode)?;
    info!("RX/TX fallback mode set to {}", config.fallback_mode);

    // 关闭增强接收 (开启会增加接收灵敏度，但功耗会增加)
    self.set_rx_gain(false)?;
    // TODO: 改成配置

    self.set_lora_modulation_params(config.lora_modulation)?;
    info!("LoRa modulation params set to {:?}", config.lora_modulation);

    self.set_lora_packet_params(config.lora_packet)?;
    info!("LoRa packet params set to {:?}", config.lora_packet);

    self.set_lora_sync_word(config.lora_sync_word)?;
    info!("LoRa sync word set to 0x{:02X}", config.lora_sync_word);

    self.config = Some(config);
    info!("SX1268 initialization complete");

    Ok(())
  }

  // -----------------------------------------------------------------------
  // Operational Modes
  // -----------------------------------------------------------------------

  /// Set the device into sleep mode.
  pub fn set_sleep(&mut self, config: SleepConfig) -> Result<(), Error<E>> {
    info!("SetSleep config={}", config);
    self
      .control
      .write_command(codes::SET_SLEEP, &[config.to_byte()])
  }

  /// Set the device into standby mode.
  pub fn set_standby(&mut self, config: StandbyConfig) -> Result<(), Error<E>> {
    info!("SetStandby config={}", config);
    self
      .control
      .write_command(codes::SET_STANDBY, &[config as u8])
  }

  /// Set the device into frequency synthesis mode.
  pub fn set_fs(&mut self) -> Result<(), Error<E>> {
    info!("SetFs");
    self.control.write_command(codes::SET_FS, &[])
  }

  /// Convert a millisecond timeout into SX1268 RTC step units.
  ///
  /// The SX1268 internal RTC runs at 64 kHz, so one RTC step = 15.625 µs.
  ///
  /// ```text
  /// rtc_steps = timeout_ms × (RTC_FREQ / 1000)
  /// ```
  pub fn timeout_to_rtc_step(timeout: u32) -> u32 {
    timeout * (Self::SX126X_RTC_FREQ_IN_HZ / 1000)
  }

  /// Set the device into transmit mode.
  ///
  /// `timeout` is in units of 15.625 µs (0 = no timeout, TX single mode).
  pub fn set_tx(&mut self, timeout: u32) -> Result<(), Error<E>> {
    info!("SetTx timeout={}", timeout);
    let params = [
      ((timeout >> 16) & 0xFF) as u8,
      ((timeout >> 8) & 0xFF) as u8,
      (timeout & 0xFF) as u8,
    ];
    self.control.write_command(codes::SET_TX, &params)
  }

  /// Set the device into receive mode.
  ///
  /// `timeout` is in units of 15.625 µs.
  /// - 0x000000 = no timeout (RX single mode)
  /// - 0xFFFFFF = continuous RX mode
  pub fn set_rx(&mut self, timeout: u32) -> Result<(), Error<E>> {
    let params = [
      ((timeout >> 16) & 0xFF) as u8,
      ((timeout >> 8) & 0xFF) as u8,
      (timeout & 0xFF) as u8,
    ];
    self.control.write_command(codes::SET_RX, &params)
  }

  /// Enable or disable stopping the RX timer on preamble detection.
  pub fn stop_timer_on_preamble(&mut self, enable: bool) -> Result<(), Error<E>> {
    self
      .control
      .write_command(codes::STOP_TIMER_ON_PREAMBLE, &[enable as u8])
  }

  /// Set the device into RX duty cycle mode.
  pub fn set_rx_duty_cycle(&mut self, rx_period: u32, sleep_period: u32) -> Result<(), Error<E>> {
    info!(
      "SetRxDutyCycle rx_period=0x{:06X} sleep_period=0x{:06X}",
      rx_period, sleep_period
    );
    let params = [
      ((rx_period >> 16) & 0xFF) as u8,
      ((rx_period >> 8) & 0xFF) as u8,
      (rx_period & 0xFF) as u8,
      ((sleep_period >> 16) & 0xFF) as u8,
      ((sleep_period >> 8) & 0xFF) as u8,
      (sleep_period & 0xFF) as u8,
    ];
    self
      .control
      .write_command(codes::SET_RX_DUTY_CYCLE, &params)
  }

  /// Set the RX gain mode.
  ///
  /// - `gain = false` — power-saving gain mode (register value `0x94`).
  /// - `gain = true`  — boosted gain mode (register value `0x96`),
  ///   which increases receive sensitivity at the cost of higher current
  ///   consumption.
  pub fn set_rx_gain(&mut self, gain: bool) -> Result<(), Error<E>> {
    info!("SetRxGain gain={}", gain);
    let code = if gain { 0x96 } else { 0x94 };

    self.control.write_register(codes::REG_RX_GAIN, &[code])
  }

  /// Set the device into CAD (Channel Activity Detection) mode.
  pub fn set_cad(&mut self) -> Result<(), Error<E>> {
    info!("SetCad");
    self.control.write_command(codes::SET_CAD, &[])
  }

  /// Set the device to transmit a continuous wave (for testing).
  pub fn set_tx_continuous_wave(&mut self) -> Result<(), Error<E>> {
    info!("SetTxContinuousWave");
    self
      .control
      .write_command(codes::SET_TX_CONTINUOUS_WAVE, &[])
  }

  /// Set the device to transmit an infinite preamble (for testing).
  pub fn set_tx_infinite_preamble(&mut self) -> Result<(), Error<E>> {
    info!("SetTxInfinitePreamble");
    self
      .control
      .write_command(codes::SET_TX_INFINITE_PREAMBLE, &[])
  }

  // -----------------------------------------------------------------------
  // Power / Regulator Configuration
  // -----------------------------------------------------------------------

  /// Set the regulator mode (LDO only or DC-DC+LDO).
  pub fn set_regulator_mode(&mut self, mode: RegulatorMode) -> Result<(), Error<E>> {
    info!("SetRegulatorMode mode={}", mode);
    self
      .control
      .write_command(codes::SET_REGULATOR_MODE, &[mode as u8])
  }

  /// Run calibration of the specified blocks.
  pub fn calibrate(&mut self, params: CalibrationParams) -> Result<(), Error<E>> {
    info!("Calibrate params=0x{:02X}", params.0);
    self.control.write_command(codes::CALIBRATE, &[params.0])
  }

  /// Calibrate the image rejection for a given frequency range.
  pub fn calibrate_image(&mut self, freq1: u8, freq2: u8) -> Result<(), Error<E>> {
    info!("CalibrateImage freq1=0x{:02X} freq2=0x{:02X}", freq1, freq2);
    self
      .control
      .write_command(codes::CALIBRATE_IMAGE, &[freq1, freq2])
  }

  /// Configure the Power Amplifier.
  pub fn set_pa_config(&mut self, config: PaConfig) -> Result<(), Error<E>> {
    info!("SetPaConfig config={}", config);
    self.control.write_command(
      codes::SET_PA_CONFIG,
      &[
        config.pa_duty_cycle,
        config.hp_max,
        config.device_sel,
        config.pa_lut,
      ],
    )
  }

  /// Set the fallback mode after TX or RX operation completes.
  pub fn set_rx_tx_fallback_mode(&mut self, mode: FallbackMode) -> Result<(), Error<E>> {
    info!("SetRxTxFallbackMode mode={}", mode);
    self
      .control
      .write_command(codes::SET_RX_TX_FALLBACK_MODE, &[mode as u8])
  }

  // -----------------------------------------------------------------------
  // DIO and IRQ
  // -----------------------------------------------------------------------

  /// Configure IRQ masks for DIO1, DIO2 and DIO3.
  pub fn set_dio_irq_params(
    &mut self,
    irq_mask: IrqMasks,
    dio1_mask: IrqMasks,
    dio2_mask: IrqMasks,
    dio3_mask: IrqMasks,
  ) -> Result<(), Error<E>> {
    let params = [
      (irq_mask.bits() >> 8) as u8,
      irq_mask.bits() as u8,
      (dio1_mask.bits() >> 8) as u8,
      dio1_mask.bits() as u8,
      (dio2_mask.bits() >> 8) as u8,
      dio2_mask.bits() as u8,
      (dio3_mask.bits() >> 8) as u8,
      dio3_mask.bits() as u8,
    ];
    self
      .control
      .write_command(codes::SET_DIO_IRQ_PARAMS, &params)
  }

  /// Get the current IRQ status.
  pub fn get_irq_status(&mut self) -> Result<u16, Error<E>> {
    let param = [0u8; 1];
    let mut buf = [0u8; 2];
    self
      .control
      .read_command(codes::GET_IRQ_STATUS, &param, &mut buf)?;
    let irq = u16::from_be_bytes([buf[0], buf[1]]);
    Ok(irq)
  }

  /// Clear the specified IRQ flags.
  pub fn clear_irq_status(&mut self, mask: IrqMasks) -> Result<(), Error<E>> {
    debug!("ClearIrqStatus mask=0x{:04X}", mask.bits());
    self.control.write_command(
      codes::CLEAR_IRQ_STATUS,
      &[(mask.bits() >> 8) as u8, mask.bits() as u8],
    )
  }

  /// Set DIO2 as RF switch control.
  pub fn set_dio2_as_rf_switch_ctrl(&mut self, enable: bool) -> Result<(), Error<E>> {
    info!("SetDIO2AsRfSwitchCtrl enable={}", enable);
    self
      .control
      .write_command(codes::SET_DIO2_AS_RF_SWITCH_CTRL, &[enable as u8])
  }

  /// Set DIO3 as TCXO control with the specified voltage and timeout.
  ///
  /// `timeout` is in units of 15.625 µs.
  pub fn set_dio3_as_tcxo_ctrl(
    &mut self,
    voltage: TcxoVoltage,
    timeout: u32,
  ) -> Result<(), Error<E>> {
    info!(
      "SetDIO3AsTCXOCtrl voltage={} timeout=0x{:06X}",
      voltage, timeout
    );
    let params = [
      voltage as u8,
      ((timeout >> 16) & 0xFF) as u8,
      ((timeout >> 8) & 0xFF) as u8,
      (timeout & 0xFF) as u8,
    ];
    self
      .control
      .write_command(codes::SET_DIO3_AS_TCXO_CTRL, &params)
  }

  // -----------------------------------------------------------------------
  // RF, Modulation and Packet Configuration
  // -----------------------------------------------------------------------

  /// Convert a frequency in Hz to a 32-bit PLL step value.
  ///
  /// The SX1268 PLL frequency register expects:
  ///
  /// ```text
  /// freq_reg = freq_hz × 2²⁵ / F_XTAL
  /// ```
  ///
  /// Because `2²⁵ / F_XTAL(32 MHz) = 1.048576`, a naïve multiplication
  /// would overflow 32 bits for frequencies above ~4 GHz.  This
  /// implementation avoids 64-bit arithmetic by using a pre-scaled
  /// constant (`SX126X_PLL_STEP_SCALED`) and splitting the computation
  /// into integer and fractional parts:
  ///
  /// ```text
  /// steps_int  = freq_hz / PLL_STEP_SCALED
  /// steps_frac = freq_hz - steps_int × PLL_STEP_SCALED
  /// result     = (steps_int << 14) + round(steps_frac << 14 / PLL_STEP_SCALED)
  /// ```
  fn convert_freq_in_hz_to_pll_step(freq_hz: u32) -> u32 {
    let steps_int = freq_hz / Self::SX126X_PLL_STEP_SCALED;
    let steps_frac = freq_hz - (steps_int * Self::SX126X_PLL_STEP_SCALED);
    (steps_int << Self::SX126X_PLL_STEP_SHIFT_AMOUNT)
      + (((steps_frac << Self::SX126X_PLL_STEP_SHIFT_AMOUNT) + (Self::SX126X_PLL_STEP_SCALED >> 1))
        / Self::SX126X_PLL_STEP_SCALED)
  }

  /// Set the RF frequency in Hz.
  ///
  /// The SX1268 uses a 32-bit frequency word: `freq_word = freq_hz * 2^25 / 32_000_000`.
  pub fn set_rf_frequency(&mut self, frequency_hz: u32) -> Result<(), Error<E>> {
    let freq_reg = Self::convert_freq_in_hz_to_pll_step(frequency_hz);
    info!(
      "SetRfFrequency freq={}Hz reg=0x{:08X}",
      frequency_hz, freq_reg
    );
    let params = [
      ((freq_reg >> 24) & 0xFF) as u8,
      ((freq_reg >> 16) & 0xFF) as u8,
      ((freq_reg >> 8) & 0xFF) as u8,
      (freq_reg & 0xFF) as u8,
    ];
    self.control.write_command(codes::SET_RF_FREQUENCY, &params)
  }

  /// Set the packet type (GFSK or LoRa).
  pub fn set_packet_type(&mut self, packet_type: PacketType) -> Result<(), Error<E>> {
    info!("SetPacketType type={}", packet_type);
    self
      .control
      .write_command(codes::SET_PACKET_TYPE, &[packet_type as u8])
  }

  /// Get the current packet type.
  pub fn get_packet_type(&mut self) -> Result<PacketType, Error<E>> {
    let param = [0u8; 1];
    let mut buf = [0u8; 1];
    self
      .control
      .read_command(codes::GET_PACKET_TYPE, &param, &mut buf)?;
    let pt = match buf[0] {
      0x00 => PacketType::Gfsk,
      0x01 => PacketType::LoRa,
      _ => return Err(Error::InvalidStatus),
    };
    debug!("GetPacketType type={}", pt);
    Ok(pt)
  }

  /// Set TX output power and ramp time.
  ///
  /// `power` is in dBm (range: -9 to +22 for SX1268).
  pub fn set_tx_params(&mut self, power: i8, ramp_time: RampTime) -> Result<(), Error<E>> {
    info!("SetTxParams power={}dBm ramp={}", power, ramp_time);
    self
      .control
      .write_command(codes::SET_TX_PARAMS, &[power as u8, ramp_time as u8])
  }

  /// Set LoRa modulation parameters.
  pub fn set_lora_modulation_params(
    &mut self,
    params: LoRaModulationParams,
  ) -> Result<(), Error<E>> {
    info!("SetModulationParams(LoRa) params={}", params);
    let data = [
      params.sf as u8,
      params.bw as u8,
      params.cr as u8,
      params.low_data_rate_optimize as u8,
    ];
    self
      .control
      .write_command(codes::SET_MODULATION_PARAMS, &data)?;
    self.tx_modulation_workaround(PacketType::LoRa, params.bw)?;

    Ok(())
  }

  /// Apply the TX modulation quality workaround (Errata §15.1).
  ///
  /// When using **LoRa BW500**, bit 2 of `REG_TX_MODULATION` (`0x0889`)
  /// must be **cleared**; for all other bandwidths (and GFSK) it must be
  /// **set**.  Failure to do so may degrade the modulation quality.
  ///
  /// This method is called automatically by
  /// [`set_lora_modulation_params`](Self::set_lora_modulation_params).
  fn tx_modulation_workaround(
    &mut self,
    package_type: PacketType,
    bandwidth: LoRaBandwidth,
  ) -> Result<(), Error<E>> {
    let mut reg_value = [0u8; 1];
    self
      .control
      .read_register(codes::REG_TX_MODULATION, &mut reg_value)?;
    if package_type == PacketType::LoRa {
      if bandwidth == LoRaBandwidth::Bw500 {
        reg_value[0] &= !(1 << 2); // Set bit 0 for 500 kHz bandwidth
      } else {
        reg_value[0] |= 1 << 2; // Clear bit 0 for other bandwidths
      }
    } else {
      reg_value[0] |= 1 << 2;
    }

    self
      .control
      .write_register(codes::REG_TX_MODULATION, &reg_value)?;

    Ok(())
  }

  /// Set LoRa packet parameters.
  ///
  /// After sending the `SetPacketParams` command this method also applies
  /// the IQ polarity workaround (Errata §15.4): bit 2 of
  /// `REG_IQ_POLARITY` (`0x0736`) is **cleared** when IQ inversion is
  /// enabled and **set** when using normal IQ, to ensure correct receive
  /// behaviour.
  pub fn set_lora_packet_params(&mut self, params: LoRaPacketParams) -> Result<(), Error<E>> {
    let data = [
      (params.preamble_length >> 8) as u8,
      params.preamble_length as u8,
      params.header_type as u8,
      params.payload_length,
      params.crc_on as u8,
      params.invert_iq as u8,
    ];
    self
      .control
      .write_command(codes::SET_PACKET_PARAMS, &data)?;

    let mut reg_value = [0u8; 1];
    self
      .control
      .read_register(codes::REG_IQ_POLARITY, &mut reg_value)?;
    if params.invert_iq {
      reg_value[0] &= !(1 << 2); // Set bit 2 to invert IQ
    } else {
      reg_value[0] |= 1 << 2; // Clear bit 2 for normal IQ
    }

    self
      .control
      .write_register(codes::REG_IQ_POLARITY, &reg_value)?;

    Ok(())
  }

  /// Set CAD parameters.
  pub fn set_cad_params(
    &mut self,
    num_symbols: CadSymbols,
    det_peak: u8,
    det_min: u8,
    exit_mode: CadExitMode,
    timeout: u32,
  ) -> Result<(), Error<E>> {
    info!(
      "SetCadParams symbols={} peak={} min={} exit={} timeout=0x{:06X}",
      num_symbols, det_peak, det_min, exit_mode, timeout
    );
    let params = [
      num_symbols as u8,
      det_peak,
      det_min,
      exit_mode as u8,
      ((timeout >> 16) & 0xFF) as u8,
      ((timeout >> 8) & 0xFF) as u8,
      (timeout & 0xFF) as u8,
    ];
    self.control.write_command(codes::SET_CAD_PARAMS, &params)
  }

  /// Set the TX and RX buffer base addresses.
  pub fn set_buffer_base_address(&mut self, tx_base: u8, rx_base: u8) -> Result<(), Error<E>> {
    debug!(
      "SetBufferBaseAddress tx=0x{:02X} rx=0x{:02X}",
      tx_base, rx_base
    );
    self
      .control
      .write_command(codes::SET_BUFFER_BASE_ADDRESS, &[tx_base, rx_base])
  }

  /// Set the LoRa symbol number timeout (for RX).
  pub fn set_lora_symb_num_timeout(&mut self, symb_num: u8) -> Result<(), Error<E>> {
    self
      .control
      .write_command(codes::SET_LORA_SYMB_NUM_TIMEOUT, &[symb_num])
  }

  // -----------------------------------------------------------------------
  // Status and Diagnostics
  // -----------------------------------------------------------------------

  /// Get the instantaneous RSSI value (dBm).
  pub fn get_rssi_inst(&mut self) -> Result<i16, Error<E>> {
    let param = [0u8; 1];
    let mut buf = [0u8; 1];
    self
      .control
      .read_command(codes::GET_RSSI_INST, &param, &mut buf)?;
    let rssi = -(buf[0] as i16) / 2;
    debug!("GetRssiInst rssi={}dBm", rssi);
    Ok(rssi)
  }

  /// Get the RX buffer status (payload length and start pointer).
  pub fn get_rx_buffer_status(&mut self) -> Result<RxBufferStatus, Error<E>> {
    let param = [0u8; 1];
    let mut buf = [0u8; 2];
    self
      .control
      .read_command(codes::GET_RX_BUFFER_STATUS, &param, &mut buf)?;
    let status = RxBufferStatus {
      payload_length: buf[0],
      buffer_start_pointer: buf[1],
    };
    debug!("GetRxBufferStatus status={}", status);
    Ok(status)
  }

  /// Get the LoRa packet status.
  pub fn get_lora_packet_status(&mut self) -> Result<LoRaPacketStatus, Error<E>> {
    let param = [0u8; 1];
    let mut buf = [0u8; 3];
    self
      .control
      .read_command(codes::GET_PACKET_STATUS, &param, &mut buf)?;
    let status = LoRaPacketStatus {
      rssi_pkt: -(buf[0] as i16) / 2,
      snr_pkt: (buf[1] as i8) / 4,
      signal_rssi_pkt: -(buf[2] as i16) / 2,
    };
    debug!("GetLoRaPacketStatus status={}", status);
    Ok(status)
  }

  /// Get the device error flags.
  pub fn get_device_errors(&mut self) -> Result<u16, Error<E>> {
    let param = [0u8; 1];
    let mut buf = [0u8; 2];
    self
      .control
      .read_command(codes::GET_DEVICE_ERRORS, &param, &mut buf)?;
    let errors = u16::from_be_bytes(buf);
    debug!("GetDeviceErrors errors=0x{:04X}", errors);
    Ok(errors)
  }

  /// Clear all device errors.
  pub fn clear_device_errors(&mut self) -> Result<(), Error<E>> {
    debug!("ClearDeviceErrors");
    self
      .control
      .write_command(codes::CLEAR_DEVICE_ERRORS, &[0x00, 0x00])
  }

  // -----------------------------------------------------------------------
  // Convenience (higher-level) methods
  // -----------------------------------------------------------------------

  /// Set the LoRa sync word.
  ///
  /// Common values: `0x3444` for public network, `0x1424` for private network.
  pub fn set_lora_sync_word(&mut self, sync_word: u16) -> Result<(), Error<E>> {
    info!("SetLoRaSyncWord sync_word=0x{:04X}", sync_word);

    self
      .control
      .write_register(codes::REG_LORA_SYNC_WORD_MSB, &sync_word.to_be_bytes())?;
    info!("Current LoRa sync word register value 3",);
    Ok(())
  }

  /// Send a LoRa packet.
  ///
  /// This is a convenience method that:
  /// 1. Updates the packet-params payload length to match `data.len()`.
  /// 2. Writes the payload into the TX buffer at offset 0.
  /// 3. Configures DIO1 for `TxDone` IRQ and clears pending flags.
  /// 4. Switches the platform-level RF path to TX.
  /// 5. Starts the transmission with the given timeout.
  ///
  /// The caller should monitor the `TxDone` IRQ (via DIO1 or polling
  /// [`get_irq_status`](Self::get_irq_status)) to know when the
  /// transmission has completed.
  ///
  /// `timeout` is in RTC step units (15.625 µs each); `0` means no
  /// timeout.
  ///
  /// # Note
  ///
  /// Returns `Ok(())` immediately without sending anything if
  /// [`init`](Self::init) has not been called yet.
  pub fn send_lora(&mut self, data: &[u8], timeout: u32) -> Result<(), Error<E>> {
    info!("SendLoRa len={} timeout={}", data.len(), timeout);
    if let Some(config) = &self.config {
      let mut package = config.lora_packet;
      package.payload_length = data.len().min(255) as u8;
      self.set_lora_packet_params(package)?;
      self.control.write_buffer(0x00, data)?;
      self.set_dio_irq_params(
        IrqMasks::TxDone,
        IrqMasks::TxDone,
        IrqMasks::None,
        IrqMasks::None,
      )?;
      self.clear_irq_status(IrqMasks::All)?;
      self.control.switch_tx(0)?;
      self.set_tx(timeout)?;
      // self.control.switch_rx(0)?;
    } else {
      warn!("Device not initialized, cannot send LoRa packet");
    }
    Ok(())
  }

  /// 进入 LoRa 接收模式。
  ///
  /// 配置 DIO1 为 `RxDone` IRQ、清除所有 pending 标志、切换 RF 通路为接收，
  /// 然后以给定超时启动接收器。
  ///
  /// 调用此方法后，通过轮询 DIO1 引脚（高电平表示 `RxDone` 触发）或调用
  /// [`get_irq_status`](Self::get_irq_status) 来检测数据包到达，
  /// 再调用 [`recv_lora`](Self::recv_lora) 读取载荷。
  ///
  /// `timeout` 单位为 RTC 步进（15.625 µs/步）：
  /// - `0x000000` = 单次接收（收到一个包或超时后退出）
  /// - `0xFFFFFF` = 持续接收（永不超时，芯片保持在 RX 模式）
  pub fn start_lora_rx(&mut self, timeout: u32) -> Result<(), Error<E>> {
    if let Some(config) = &self.config {
      let mut package = config.lora_packet;
      package.payload_length = 255; // RX 时 payload length 由接收的包决定
      let rx_irq_mask =
        IrqMasks::RxDone | IrqMasks::Timeout | IrqMasks::HeaderError | IrqMasks::CrcError;
      let tx_base = config.tx_base_address;
      let rx_base = config.rx_base_address;
      self.set_lora_packet_params(package)?;
      // 每次进入 RX 前重置 buffer 指针，防止多次接收后 buffer_start_pointer 累积漂移
      self.set_buffer_base_address(tx_base, rx_base)?;
      self.set_dio_irq_params(rx_irq_mask, rx_irq_mask, IrqMasks::None, IrqMasks::None)?;
      self.clear_irq_status(IrqMasks::All)?;
      self.control.switch_rx(0)?;
      self.set_rx(timeout)?;
    } else {
      warn!("Device not initialized, cannot start LoRa RX");
    }
    Ok(())
  }

  /// 读取已接收的 LoRa 数据包到 `buf`。
  ///
  /// 当 `RxDone` IRQ 已触发（DIO1 高电平或
  /// [`get_irq_status`](Self::get_irq_status) 返回 `RxDone`）时调用。
  ///
  /// 返回值：
  /// - `Ok(Some(len))` — 数据包已读入 `buf[..len]`
  /// - `Ok(None)` — `RxDone` IRQ 尚未触发，暂无数据
  ///
  /// 在持续接收模式下（`timeout = 0xFFFFFF`），芯片每次收包后自动重新
  /// 进入 RX，无需重新调用 [`start_lora_rx`](Self::start_lora_rx)。
  /// 在单次接收模式下，每次成功读取后需重新调用 `start_lora_rx`。
  pub fn recv_lora(&mut self, buf: &mut [u8]) -> Result<Option<usize>, Error<E>> {
    let irq = self.get_irq_status()?;
    self.clear_irq_status(IrqMasks::All)?;
    // info!("IRQ status: 0b{:b}", irq);
    if irq == IrqMasks::None.bits() {
      warn!("No IRQ flags set, no packet received yet");
      return Ok(None);
    }

    let rx_error_mask = IrqMasks::Timeout | IrqMasks::HeaderError | IrqMasks::CrcError;
    if rx_error_mask.intersects(irq) {
      // self.clear_irq_status(rx_error_mask)?;
      warn!("RX error IRQ set: 0x{:04X}", irq & rx_error_mask.bits());
      return Ok(None);
    }

    if !IrqMasks::RxDone.intersects(irq) {
      // self.clear_irq_status(IrqMasks::PreambleDetected)?;
      warn!("RxDone IRQ not set, ignoring irq=0x{:04X}", irq);
      return Ok(None);
    }

    let _packet_status = self.get_lora_packet_status()?;
    // info!("LoRa packet status: {}", packet_status);

    let buffer_status = self.get_rx_buffer_status()?;
    if buffer_status.payload_length == 0 {
      // self.clear_irq_status(IrqMasks::RxDone)?;
      warn!("RxDone IRQ but payload_length=0, discarding");
      return Ok(None);
    }
    let len = (buffer_status.payload_length as usize).min(buf.len());
    self
      .control
      .read_buffer(buffer_status.buffer_start_pointer, &mut buf[..len])?;
    // self.clear_irq_status(IrqMasks::All)?;
    // info!("LoRa received {} bytes\n{:02X}", len, &buf[..len]);
    Ok(Some(len))
  }

  /// Begin listening for a LoRa packet and return a *reader* closure.
  ///
  /// This method:
  /// 1. Updates the packet-params payload length to match `buf.len()`.
  /// 2. Configures DIO1 for `RxDone` IRQ and clears pending flags.
  /// 3. Switches the platform-level RF path to RX.
  /// 4. Starts the receiver with the given timeout.
  ///
  /// The returned closure captures `buf` and, when invoked with the
  /// buffer start offset obtained from [`get_rx_buffer_status`], reads
  /// the received payload from the chip's data buffer.
  ///
  /// Use [`try_receive_lora_packet`](Self::try_receive_lora_packet) in a
  /// loop (or after a DIO1 interrupt) to check for completion and
  /// invoke the reader.
  ///
  /// `timeout` is in RTC step units (15.625 µs each); `0` = no timeout,
  /// `0xFFFFFF` = continuous RX.
  #[allow(clippy::type_complexity)]
  pub fn wait_lora_packet(
    &mut self,
    buf: &mut [u8],
    timeout: u32,
  ) -> Result<impl FnOnce(&mut Self, u8) -> Result<usize, Error<E>>, Error<E>> {
    if let Some(config) = &self.config {
      let mut package = config.lora_packet;
      package.payload_length = buf.len().min(255) as u8;
      debug!("payload_length={}", package.payload_length);
      self.set_lora_packet_params(package)?;
      self.set_dio_irq_params(
        IrqMasks::RxDone,
        IrqMasks::RxDone,
        IrqMasks::None,
        IrqMasks::None,
      )?;
      self.clear_irq_status(IrqMasks::All)?;
      self.control.switch_rx(0)?;
      self.set_rx(timeout)?;
      info!("Waiting for LoRa packet...");
    }

    let reader = move |driver: &mut Self, offset: u8| {
      debug!("Reading LoRa packet from buffer with offset={}", offset);
      driver.control.read_buffer(offset, buf)?;
      let size = buf.len().min(255);
      info!("Read {} bytes from RX buffer", size);
      Ok(size)
    };

    Ok(reader)
  }

  /// Poll for a received LoRa packet and optionally read it.
  ///
  /// Call this method after [`wait_lora_packet`](Self::wait_lora_packet)
  /// to check whether the `RxDone` IRQ has fired.
  ///
  /// # Return value
  ///
  /// - `Ok((Some(len), None))` — a packet was received; `len` bytes have
  ///   been written into the buffer captured by `reader`.  The reader has
  ///   been consumed.
  /// - `Ok((None, Some(reader)))` — no packet yet; the reader is handed
  ///   back so it can be re-used on the next poll.
  /// - `Ok((None, None))` — the `RxDone` IRQ fired but the payload length
  ///   was zero (spurious).
  pub fn try_receive_lora_packet<F>(
    &mut self,
    reader: F,
  ) -> Result<(Option<usize>, Option<F>), Error<E>>
  where
    F: FnOnce(&mut Self, u8) -> Result<usize, Error<E>>,
  {
    let irq = self.get_irq_status()?;

    if irq == IrqMasks::None.bits() {
      debug!("No IRQ flags set, no packet received yet");
      return Ok((None, Some(reader)));
    }

    let rx_error_mask = IrqMasks::Timeout | IrqMasks::HeaderError | IrqMasks::CrcError;
    if rx_error_mask.intersects(irq) {
      self.clear_irq_status(rx_error_mask)?;
      warn!("RX error IRQ set: 0x{:04X}", irq & rx_error_mask.bits());
      return Ok((None, Some(reader)));
    }

    if IrqMasks::PreambleDetected.intersects(irq) {
      self.clear_irq_status(IrqMasks::PreambleDetected)?;
      debug!("LoRa preamble detected");
      return Ok((None, Some(reader)));
    }

    if IrqMasks::RxDone.intersects(irq) {
      let buffer_status = self.get_rx_buffer_status()?;
      debug!("RX buffer status: {}", buffer_status);

      if buffer_status.payload_length == 0 {
        warn!("RX done IRQ but payload length is 0, clearing IRQ and returning");
        self.clear_irq_status(IrqMasks::RxDone)?;
        return Ok((None, None));
      }

      let size = reader(self, buffer_status.buffer_start_pointer)?;
      self.clear_irq_status(IrqMasks::All)?;
      info!("LoRa packet received, reading from buffer...");

      Ok((Some(size), None))
    } else {
      debug!("No LoRa packet received yet (IRQ=0x{:04X})", irq);
      Ok((None, Some(reader)))
    }
  }
}
