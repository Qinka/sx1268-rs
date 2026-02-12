use crate::codes;
/// Core SX1268 driver implementation.
use crate::config::*;
use crate::control::Control;

/// Driver error type wrapping SPI errors.
#[derive(Debug, defmt::Format)]
pub enum Error<S> {
  /// ConfigError
  ConfigError(ConfigError),
  /// SPI bus error.
  ControlError(S),
  /// Invalid or unexpected device status.
  InvalidStatus,
  /// Operation timed out.
  Timeout,
  /// Invalid parameter.
  InvalidParameter,
}

impl<E> Error<E> {
  pub fn from_spi_error(e: E) -> Self {
    Error::ControlError(e)
  }
}

#[derive(Debug, defmt::Format, Copy, Clone)]
pub enum IrqMasks {
  None = 0,
  TxDone = 0x0001,
  RxDone = 0x0002,
  PreambleDetected = 0x0004,
  SyncWordValid = 0x0008,
  HeaderValid = 0x0010,
  HeaderError = 0x0020,
  CrcError = 0x0040,
  CadDone = 0x0080,
  CadDetected = 0x0100,
  Timeout = 0x0200,
  All = 0x43FF,
}

/// Chip status mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum ChipMode {
  StbyRc = 0x02,
  StbyXosc = 0x03,
  Fs = 0x04,
  Rx = 0x05,
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

/// Command status.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum CommandStatus {
  DataAvailable = 0x02,
  CommandTimeout = 0x03,
  CommandProcessingError = 0x04,
  FailureToExecute = 0x05,
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

/// Decoded device status.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Status {
  pub chip_mode: ChipMode,
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
/// RX buffer status.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct RxBufferStatus {
  pub payload_length: u8,
  pub buffer_start_pointer: u8,
}

/// LoRa packet status.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct LoRaPacketStatus {
  /// Average over last packet received of RSSI (dBm).
  pub rssi_pkt: i16,
  /// Estimation of SNR on last packet received (dB).
  pub snr_pkt: i8,
  /// RSSI of the LoRa signal (dBm).
  pub signal_rssi_pkt: i16,
}

/// SX1268 LoRa transceiver driver.
///
/// This driver communicates with the SX1268 chip over SPI using the
/// `embedded-hal` `SpiDevice` trait.
pub struct Sx1268<C> {
  control: C,
  config: Option<Sx1268Config>,
}

// impl<S: SpiDevice> Control for Sx1268<S> {
//   type Status = Status;
//   type Error = Error<S::Error>;

//   // -----------------------------------------------------------------------
//   // Low-level SPI helpers
//   // -----------------------------------------------------------------------

//   /// Write a command with parameters.
//   fn write_command(&mut self, opcode: u8, params: &[u8]) -> Result<(), Error<S::Error>> {
//     let cmd = [opcode];
//     self
//       .spi
//       .transaction(&mut [Operation::Write(&cmd), Operation::Write(params)])
//       .map_err(Error::Spi)?;
//     defmt::trace!("SPI write cmd=0x{:02X} params={:?}", opcode, params);
//     Ok(())
//   }

//   /// Read a command response.
//   /// The SX1268 protocol sends a status byte after the opcode + NOP, then
//   /// returns the response data.
//   fn read_command(&mut self, opcode: u8, response: &mut [u8]) -> Result<Status, Error<S::Error>> {
//     let cmd = [opcode, 0x00]; // opcode + NOP
//     let mut status_buf = [0u8; 1];
//     self
//       .spi
//       .transaction(&mut [
//         Operation::Write(&cmd),
//         Operation::Read(&mut status_buf),
//         Operation::Read(response),
//       ])
//       .map_err(Error::Spi)?;
//     let status = Status::from(status_buf[0]);
//     defmt::trace!(
//       "SPI read cmd=0x{:02X} status={} resp={:?}",
//       opcode,
//       status,
//       response
//     );
//     Ok(status)
//   }

//   /// Write to registers starting at the given address.
//   fn write_register(&mut self, address: u16, data: &[u8]) -> Result<(), Error<S::Error>> {
//     let header = [opcode::WRITE_REGISTER, (address >> 8) as u8, address as u8];
//     self
//       .spi
//       .transaction(&mut [Operation::Write(&header), Operation::Write(data)])
//       .map_err(Error::Spi)?;
//     defmt::trace!("WriteRegister addr=0x{:04X} data={:?}", address, data);
//     Ok(())
//   }

//   /// Read from registers starting at the given address.
//   fn read_register(&mut self, address: u16, data: &mut [u8]) -> Result<Status, Error<S::Error>> {
//     let header = [
//       opcode::READ_REGISTER,
//       (address >> 8) as u8,
//       address as u8,
//       0x00, // NOP (status)
//     ];
//     self
//       .spi
//       .transaction(&mut [Operation::Write(&header), Operation::Read(data)])
//       .map_err(Error::Spi)?;
//     defmt::trace!("ReadRegister addr=0x{:04X} data={:?}", address, data);
//     // Status not reliably returned from read_register, return default
//     Ok(Status::from(0))
//   }

//   /// Write data to the TX buffer at the given offset.
//   fn write_buffer(&mut self, offset: u8, data: &[u8]) -> Result<(), Error<S::Error>> {
//     let header = [opcode::WRITE_BUFFER, offset];
//     self
//       .spi
//       .transaction(&mut [Operation::Write(&header), Operation::Write(data)])
//       .map_err(Error::Spi)?;
//     defmt::trace!("WriteBuffer offset={} len={}", offset, data.len());
//     Ok(())
//   }

//   /// Read data from the RX buffer at the given offset.
//   fn read_buffer(&mut self, offset: u8, data: &mut [u8]) -> Result<(), Error<S::Error>> {
//     let header = [opcode::READ_BUFFER, offset, 0x00]; // offset + NOP (status)
//     self
//       .spi
//       .transaction(&mut [Operation::Write(&header), Operation::Read(data)])
//       .map_err(Error::Spi)?;
//     defmt::trace!("ReadBuffer offset={} len={}", offset, data.len());
//     Ok(())
//   }

//   /// Get the device status.
//   fn get_status(&mut self) -> Result<Status, Error<S::Error>> {
//     let mut status_byte = [0u8; 1];
//     self
//       .spi
//       .transaction(&mut [
//         Operation::Write(&[opcode::GET_STATUS]),
//         Operation::Read(&mut status_byte),
//       ])
//       .map_err(Error::Spi)?;
//     let status = Status::from(status_byte[0]);
//     defmt::debug!("GetStatus status={}", status);
//     Ok(status)
//   }
// }

impl<C, E> Sx1268<C>
where
  C: Control<Error = Error<E>>,
{
  const SX126X_XTAL_FREQ: u32 = 32_000_000;
  const SX126X_RTC_FREQ_IN_HZ: u32 = 64000;
  const SX126X_PLL_STEP_SHIFT_AMOUNT: u32 = 14;
  const SX126X_PLL_STEP_SCALED: u32 =
    (Self::SX126X_XTAL_FREQ >> (25 - Self::SX126X_PLL_STEP_SHIFT_AMOUNT));

  /// Create a new SX1268 driver instance.
  pub fn new(control: C) -> Self {
    Self {
      control,
      config: None,
    }
  }

  pub fn init(&mut self, config: Sx1268Config) -> Result<(), Error<E>> {
    defmt::info!("Initializing SX1268");
    defmt::debug!("Config={:?}", config);

    // 硬件复位
    self.control.reset()?;
    defmt::info!("Hardware reset complete ...");

    // 唤醒设备
    self.control.wakeup()?;
    defmt::info!("Device wakeup complete ...");

    // 进入 standby 模式，使用内部 RC 作为时钟源，等待后续配置
    self.set_standby(StandbyConfig::StbyRc)?;
    defmt::info!("Device reset complete");

    // TODO: 保留寄存器设置
    // sx126x_init_retention_list( &context_e22 );

    // 内部电源模式 (DCDC功耗更小)
    self.set_regulator_mode(config.regulator_mode)?;
    defmt::info!("Regulator mode set to {}", config.regulator_mode);

    // DIO2切换射频开关
    self.set_dio2_as_rf_switch_ctrl(config.dio2_as_rf_switch)?;
    defmt::info!(
      "DIO2 as RF switch control {}",
      if config.dio2_as_rf_switch {
        "enabled"
      } else {
        "disabled"
      }
    );

    if let Some(tcxo) = config.tcxo {
      self.set_dio3_as_tcxo_ctrl(tcxo.voltage, tcxo.timeout)?;
      defmt::info!("TCXO configured on DIO3");
    }

    // 修正内部状态
    self.calibrate(config.calibration)?;
    defmt::info!("Calibration complete");

    self.set_packet_type(config.package_type)?;
    defmt::info!("Packet type set to {}", config.package_type);

    self.set_rf_frequency(config.frequency_hz)?;
    defmt::info!("RF frequency set to {} Hz", config.frequency_hz);

    self.set_pa_config(config.pa_config)?;
    defmt::info!("PA config set to {}", config.pa_config);

    self.set_tx_params(config.tx_power, config.ramp_time)?;
    defmt::info!(
      "TX params set to power={}dBm ramp={}",
      config.tx_power,
      config.ramp_time
    );

    self.set_rx_tx_fallback_mode(config.fallback_mode)?;
    defmt::info!("RX/TX fallback mode set to {}", config.fallback_mode);

    // 关闭增强接收 (开启会增加接收灵敏度，但功耗会增加)
    self.set_rx_gain(false)?;
    // TODO: 改成配置

    self.set_lora_modulation_params(config.lora_modulation)?;
    defmt::info!("LoRa modulation params set to {:?}", config.lora_modulation);

    self.set_lora_packet_params(config.lora_packet)?;
    defmt::info!("LoRa packet params set to {:?}", config.lora_packet);

    self.set_lora_sync_word(config.lora_sync_word)?;
    defmt::info!("LoRa sync word set to 0x{:02X}", config.lora_sync_word);

    // other

    // self.set_dio2_as_rf_switch_ctrl(config.dio2_as_rf_switch)?;
    // defmt::info!(
    //   "DIO2 as RF switch control {}",
    //   if config.dio2_as_rf_switch {
    //     "enabled"
    //   } else {
    //     "disabled"
    //   }
    // );

    // self.set_buffer_base_address(config.tx_base_address, config.rx_base_address)?;
    // defmt::info!(
    //   "Buffer base addresses set to TX=0x{:02X} RX=0x{:02X}",
    //   config.tx_base_address,
    //   config.rx_base_address
    // );

    self.config = Some(config);
    defmt::info!("SX1268 initialization complete");

    Ok(())
  }

  // -----------------------------------------------------------------------
  // Operational Modes
  // -----------------------------------------------------------------------

  /// Set the device into sleep mode.
  pub fn set_sleep(&mut self, config: SleepConfig) -> Result<(), Error<E>> {
    defmt::info!("SetSleep config={}", config);
    self
      .control
      .write_command(codes::SET_SLEEP, &[config.to_byte()])
  }

  /// Set the device into standby mode.
  pub fn set_standby(&mut self, config: StandbyConfig) -> Result<(), Error<E>> {
    defmt::info!("SetStandby config={}", config);
    self
      .control
      .write_command(codes::SET_STANDBY, &[config as u8])
  }

  /// Set the device into frequency synthesis mode.
  pub fn set_fs(&mut self) -> Result<(), Error<E>> {
    defmt::info!("SetFs");
    self.control.write_command(codes::SET_FS, &[])
  }

  pub fn timeout_to_rtc_step(timeout: u32) -> u32 {
    timeout * (Self::SX126X_RTC_FREQ_IN_HZ / 1000)
  }

  /// Set the device into transmit mode.
  ///
  /// `timeout` is in units of 15.625 µs (0 = no timeout, TX single mode).
  pub fn set_tx(&mut self, timeout: u32) -> Result<(), Error<E>> {
    defmt::info!("SetTx timeout={}", timeout);
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
    defmt::info!("SetRx timeout=0x{:06X}", timeout);
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
    defmt::info!(
      "SetRxDutyCycle rx_period=0x{:06X} sleep_period=0x{:06X}",
      rx_period,
      sleep_period
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

  pub fn set_rx_gain(&mut self, gain: bool) -> Result<(), Error<E>> {
    defmt::info!("SetRxGain gain={}", gain);
    let code = if gain { 0x96 } else { 0x94 };

    self.control.write_register(codes::REG_RX_GAIN, &[code])
  }

  /// Set the device into CAD (Channel Activity Detection) mode.
  pub fn set_cad(&mut self) -> Result<(), Error<E>> {
    defmt::info!("SetCad");
    self.control.write_command(codes::SET_CAD, &[])
  }

  /// Set the device to transmit a continuous wave (for testing).
  pub fn set_tx_continuous_wave(&mut self) -> Result<(), Error<E>> {
    defmt::info!("SetTxContinuousWave");
    self
      .control
      .write_command(codes::SET_TX_CONTINUOUS_WAVE, &[])
  }

  /// Set the device to transmit an infinite preamble (for testing).
  pub fn set_tx_infinite_preamble(&mut self) -> Result<(), Error<E>> {
    defmt::info!("SetTxInfinitePreamble");
    self
      .control
      .write_command(codes::SET_TX_INFINITE_PREAMBLE, &[])
  }

  // -----------------------------------------------------------------------
  // Power / Regulator Configuration
  // -----------------------------------------------------------------------

  /// Set the regulator mode (LDO only or DC-DC+LDO).
  pub fn set_regulator_mode(&mut self, mode: RegulatorMode) -> Result<(), Error<E>> {
    defmt::info!("SetRegulatorMode mode={}", mode);
    self
      .control
      .write_command(codes::SET_REGULATOR_MODE, &[mode as u8])
  }

  /// Run calibration of the specified blocks.
  pub fn calibrate(&mut self, params: CalibrationParams) -> Result<(), Error<E>> {
    defmt::info!("Calibrate params=0x{:02X}", params.0);
    self.control.write_command(codes::CALIBRATE, &[params.0])
  }

  /// Calibrate the image rejection for a given frequency range.
  pub fn calibrate_image(&mut self, freq1: u8, freq2: u8) -> Result<(), Error<E>> {
    defmt::info!("CalibrateImage freq1=0x{:02X} freq2=0x{:02X}", freq1, freq2);
    self
      .control
      .write_command(codes::CALIBRATE_IMAGE, &[freq1, freq2])
  }

  /// Configure the Power Amplifier.
  pub fn set_pa_config(&mut self, config: PaConfig) -> Result<(), Error<E>> {
    defmt::info!("SetPaConfig config={}", config);
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
    defmt::info!("SetRxTxFallbackMode mode={}", mode);
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
    defmt::info!(
      "SetDioIrqParams irq=0x{:04X} dio1=0x{:04X} dio2=0x{:04X} dio3=0x{:04X}",
      irq_mask,
      dio1_mask,
      dio2_mask,
      dio3_mask
    );
    let params = [
      (irq_mask as u16 >> 8) as u8,
      irq_mask as u8,
      (dio1_mask as u16 >> 8) as u8,
      dio1_mask as u8,
      (dio2_mask as u16 >> 8) as u8,
      dio2_mask as u8,
      (dio3_mask as u16 >> 8) as u8,
      dio3_mask as u8,
    ];
    self
      .control
      .write_command(codes::SET_DIO_IRQ_PARAMS, &params)
  }

  /// Get the current IRQ status.
  pub fn get_irq_status(&mut self) -> Result<u16, Error<E>> {
    let mut buf = [0u8; 2];
    self.control.read_command(codes::GET_IRQ_STATUS, &mut buf)?;
    let irq = u16::from_be_bytes(buf);
    defmt::debug!("GetIrqStatus irq=0x{:04X}", irq);
    Ok(irq)
  }

  /// Clear the specified IRQ flags.
  pub fn clear_irq_status(&mut self, mask: IrqMasks) -> Result<(), Error<E>> {
    defmt::debug!("ClearIrqStatus mask=0x{:04X}", mask);
    self.control.write_command(
      codes::CLEAR_IRQ_STATUS,
      &[(mask as u16 >> 8) as u8, mask as u8],
    )
  }

  /// Set DIO2 as RF switch control.
  pub fn set_dio2_as_rf_switch_ctrl(&mut self, enable: bool) -> Result<(), Error<E>> {
    defmt::info!("SetDIO2AsRfSwitchCtrl enable={}", enable);
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
    defmt::info!(
      "SetDIO3AsTCXOCtrl voltage={} timeout=0x{:06X}",
      voltage,
      timeout
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
    defmt::info!(
      "SetRfFrequency freq={}Hz reg=0x{:08X}",
      frequency_hz,
      freq_reg
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
    defmt::info!("SetPacketType type={}", packet_type);
    self
      .control
      .write_command(codes::SET_PACKET_TYPE, &[packet_type as u8])
  }

  /// Get the current packet type.
  pub fn get_packet_type(&mut self) -> Result<PacketType, Error<E>> {
    let mut buf = [0u8; 1];
    self
      .control
      .read_command(codes::GET_PACKET_TYPE, &mut buf)?;
    let pt = match buf[0] {
      0x00 => PacketType::Gfsk,
      0x01 => PacketType::LoRa,
      _ => return Err(Error::InvalidStatus),
    };
    defmt::debug!("GetPacketType type={}", pt);
    Ok(pt)
  }

  /// Set TX output power and ramp time.
  ///
  /// `power` is in dBm (range: -9 to +22 for SX1268).
  pub fn set_tx_params(&mut self, power: i8, ramp_time: RampTime) -> Result<(), Error<E>> {
    defmt::info!("SetTxParams power={}dBm ramp={}", power, ramp_time);
    self
      .control
      .write_command(codes::SET_TX_PARAMS, &[power as u8, ramp_time as u8])
  }

  /// Set LoRa modulation parameters.
  pub fn set_lora_modulation_params(
    &mut self,
    params: LoRaModulationParams,
  ) -> Result<(), Error<E>> {
    defmt::info!("SetModulationParams(LoRa) params={}", params);
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
  pub fn set_lora_packet_params(&mut self, params: LoRaPacketParams) -> Result<(), Error<E>> {
    defmt::info!("SetPacketParams(LoRa) params={}", params);
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
    defmt::info!(
      "SetCadParams symbols={} peak={} min={} exit={} timeout=0x{:06X}",
      num_symbols,
      det_peak,
      det_min,
      exit_mode,
      timeout
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
    defmt::debug!(
      "SetBufferBaseAddress tx=0x{:02X} rx=0x{:02X}",
      tx_base,
      rx_base
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
    let mut buf = [0u8; 1];
    self.control.read_command(codes::GET_RSSI_INST, &mut buf)?;
    let rssi = -(buf[0] as i16) / 2;
    defmt::debug!("GetRssiInst rssi={}dBm", rssi);
    Ok(rssi)
  }

  /// Get the RX buffer status (payload length and start pointer).
  pub fn get_rx_buffer_status(&mut self) -> Result<RxBufferStatus, Error<E>> {
    let mut buf = [0u8; 2];
    self
      .control
      .read_command(codes::GET_RX_BUFFER_STATUS, &mut buf)?;
    let status = RxBufferStatus {
      payload_length: buf[0],
      buffer_start_pointer: buf[1],
    };
    defmt::debug!("GetRxBufferStatus status={}", status);
    Ok(status)
  }

  /// Get the LoRa packet status.
  pub fn get_lora_packet_status(&mut self) -> Result<LoRaPacketStatus, Error<E>> {
    let mut buf = [0u8; 3];
    self
      .control
      .read_command(codes::GET_PACKET_STATUS, &mut buf)?;
    let status = LoRaPacketStatus {
      rssi_pkt: -(buf[0] as i16) / 2,
      snr_pkt: (buf[1] as i8) / 4,
      signal_rssi_pkt: -(buf[2] as i16) / 2,
    };
    defmt::debug!("GetLoRaPacketStatus status={}", status);
    Ok(status)
  }

  /// Get the device error flags.
  pub fn get_device_errors(&mut self) -> Result<u16, Error<E>> {
    let mut buf = [0u8; 2];
    self
      .control
      .read_command(codes::GET_DEVICE_ERRORS, &mut buf)?;
    let errors = u16::from_be_bytes(buf);
    defmt::debug!("GetDeviceErrors errors=0x{:04X}", errors);
    Ok(errors)
  }

  /// Clear all device errors.
  pub fn clear_device_errors(&mut self) -> Result<(), Error<E>> {
    defmt::debug!("ClearDeviceErrors");
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
    defmt::info!("SetLoRaSyncWord sync_word=0x{:04X}", sync_word);

    self
      .control
      .write_register(codes::REG_LORA_SYNC_WORD_MSB, &sync_word.to_be_bytes())?;
    defmt::info!("Current LoRa sync word register value 3",);
    Ok(())
  }

  /// Send a packet of data using LoRa modulation.
  ///
  /// This writes the payload to the TX buffer, configures the payload length,
  /// and initiates transmission. The caller should check IRQ status for TX_DONE.
  ///
  /// `timeout` is the TX timeout in units of 15.625 µs (0 = no timeout).
  pub fn send_lora(&mut self, data: &[u8], timeout: u32) -> Result<(), Error<E>> {
    defmt::info!("SendLoRa len={} timeout={}", data.len(), timeout);
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
      defmt::warn!("Device not initialized, cannot send LoRa packet");
    }
    Ok(())
  }

  /// Start to wait a received LoRa packet from the RX buffer.
  ///
  /// Returns the number of bytes read. `buf` must be large enough to hold the
  /// received payload.
  ///
  /// `timeout` is the TX timeout in units of 15.625 µs (0 = no timeout).
  #[allow(clippy::type_complexity)]
  pub fn wait_lora_packet(
    &mut self,
    buf: &mut [u8],
    timeout: u32,
  ) -> Result<impl FnOnce(&mut Self, u8) -> Result<usize, Error<E>>, Error<E>> {
    if let Some(config) = &self.config {
      let mut package = config.lora_packet;
      package.payload_length = buf.len().min(255) as u8;
      defmt::debug!("payload_length={}", package.payload_length);
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
      defmt::info!("Waiting for LoRa packet...");
    }

    let reader = move |driver: &mut Self, offset: u8| {
      defmt::debug!("Reading LoRa packet from buffer with offset={}", offset);
      driver.control.read_buffer(offset, buf)?;
      let size = buf.len().min(255);
      defmt::info!("Read {} bytes from RX buffer", size);
      Ok(size)
    };

    Ok(reader)
  }

  /// Try reciving a LoRa packet if available, returning the number of bytes read.
  /// The caller should first call `wait_lora_packet()` to start waiting for a packet
  /// and then call this method when an IRQ indicates a packet has been received.
  pub fn try_receive_lora_packet<F>(
    &mut self,
    reader: F,
  ) -> Result<(Option<usize>, Option<F>), Error<E>>
  where
    F: FnOnce(&mut Self, u8) -> Result<usize, Error<E>>,
  {
    let irq = self.get_irq_status()?;

    if irq == IrqMasks::None as u16 {
      defmt::debug!("No IRQ flags set, no packet received yet");
      return Ok((None, Some(reader)));
    }

    if irq & IrqMasks::PreambleDetected as u16 != 0 {
      defmt::debug!("LoRa preamble detected");
    }

    if irq & IrqMasks::RxDone as u16 != 0 {
      let buffer_status = self.get_rx_buffer_status()?;
      defmt::debug!("RX buffer status: {}", buffer_status);

      if buffer_status.payload_length == 0 {
        defmt::warn!("RX done IRQ but payload length is 0, clearing IRQ and returning");
        self.clear_irq_status(IrqMasks::RxDone)?;
        return Ok((None, None));
      }

      let size = reader(self, buffer_status.buffer_start_pointer)?;
      defmt::info!("LoRa packet received, reading from buffer...");

      Ok((Some(size), None))
    } else {
      defmt::debug!("No LoRa packet received yet (IRQ=0x{:04X})", irq);
      Ok((None, Some(reader)))
    }
  }
}
