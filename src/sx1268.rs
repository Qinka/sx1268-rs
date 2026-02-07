/// Core SX1268 driver implementation.
use embedded_hal::spi::{Operation, SpiDevice};

use crate::error::Error;
use crate::opcode;
use crate::types::*;

/// SX1268 LoRa transceiver driver.
///
/// This driver communicates with the SX1268 chip over SPI using the
/// `embedded-hal` `SpiDevice` trait.
pub struct Sx1268<SPI> {
    spi: SPI,
}

impl<SPI> Sx1268<SPI>
where
    SPI: SpiDevice,
{
    /// Create a new SX1268 driver instance.
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Release the SPI device, consuming this driver.
    pub fn release(self) -> SPI {
        self.spi
    }

    // -----------------------------------------------------------------------
    // Low-level SPI helpers
    // -----------------------------------------------------------------------

    /// Write a command with parameters.
    fn write_command(&mut self, opcode: u8, params: &[u8]) -> Result<(), Error<SPI::Error>> {
        let cmd = [opcode];
        self.spi
            .transaction(&mut [Operation::Write(&cmd), Operation::Write(params)])
            .map_err(Error::Spi)?;
        defmt::trace!("SPI write cmd=0x{:02X} params={:?}", opcode, params);
        Ok(())
    }

    /// Read a command response.
    /// The SX1268 protocol sends a status byte after the opcode + NOP, then
    /// returns the response data.
    fn read_command(
        &mut self,
        opcode: u8,
        response: &mut [u8],
    ) -> Result<Status, Error<SPI::Error>> {
        let cmd = [opcode, 0x00]; // opcode + NOP
        let mut status_buf = [0u8; 1];
        self.spi
            .transaction(&mut [
                Operation::Write(&cmd),
                Operation::Read(&mut status_buf),
                Operation::Read(response),
            ])
            .map_err(Error::Spi)?;
        let status = Status::from(status_buf[0]);
        defmt::trace!(
            "SPI read cmd=0x{:02X} status={} resp={:?}",
            opcode,
            status,
            response
        );
        Ok(status)
    }

    /// Write to registers starting at the given address.
    pub fn write_register(
        &mut self,
        address: u16,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error>> {
        let header = [
            opcode::WRITE_REGISTER,
            (address >> 8) as u8,
            address as u8,
        ];
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Write(data)])
            .map_err(Error::Spi)?;
        defmt::trace!(
            "WriteRegister addr=0x{:04X} data={:?}",
            address,
            data
        );
        Ok(())
    }

    /// Read from registers starting at the given address.
    pub fn read_register(
        &mut self,
        address: u16,
        data: &mut [u8],
    ) -> Result<Status, Error<SPI::Error>> {
        let header = [
            opcode::READ_REGISTER,
            (address >> 8) as u8,
            address as u8,
            0x00, // NOP (status)
        ];
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Read(data)])
            .map_err(Error::Spi)?;
        defmt::trace!(
            "ReadRegister addr=0x{:04X} data={:?}",
            address,
            data
        );
        // Status not reliably returned from read_register, return default
        Ok(Status::from(0))
    }

    /// Write data to the TX buffer at the given offset.
    pub fn write_buffer(
        &mut self,
        offset: u8,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error>> {
        let header = [opcode::WRITE_BUFFER, offset];
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Write(data)])
            .map_err(Error::Spi)?;
        defmt::trace!(
            "WriteBuffer offset={} len={}",
            offset,
            data.len()
        );
        Ok(())
    }

    /// Read data from the RX buffer at the given offset.
    pub fn read_buffer(
        &mut self,
        offset: u8,
        data: &mut [u8],
    ) -> Result<Status, Error<SPI::Error>> {
        let header = [opcode::READ_BUFFER, offset, 0x00]; // offset + NOP (status)
        self.spi
            .transaction(&mut [Operation::Write(&header), Operation::Read(data)])
            .map_err(Error::Spi)?;
        defmt::trace!(
            "ReadBuffer offset={} len={}",
            offset,
            data.len()
        );
        Ok(Status::from(0))
    }

    // -----------------------------------------------------------------------
    // Operational Modes
    // -----------------------------------------------------------------------

    /// Set the device into sleep mode.
    pub fn set_sleep(&mut self, config: SleepConfig) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetSleep config={}", config);
        self.write_command(opcode::SET_SLEEP, &[config.to_byte()])
    }

    /// Set the device into standby mode.
    pub fn set_standby(&mut self, config: StandbyConfig) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetStandby config={}", config);
        self.write_command(opcode::SET_STANDBY, &[config as u8])
    }

    /// Set the device into frequency synthesis mode.
    pub fn set_fs(&mut self) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetFs");
        self.write_command(opcode::SET_FS, &[])
    }

    /// Set the device into transmit mode.
    ///
    /// `timeout` is in units of 15.625 µs (0 = no timeout, TX single mode).
    pub fn set_tx(&mut self, timeout: u32) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetTx timeout={}", timeout);
        let params = [
            ((timeout >> 16) & 0xFF) as u8,
            ((timeout >> 8) & 0xFF) as u8,
            (timeout & 0xFF) as u8,
        ];
        self.write_command(opcode::SET_TX, &params)
    }

    /// Set the device into receive mode.
    ///
    /// `timeout` is in units of 15.625 µs.
    /// - 0x000000 = no timeout (RX single mode)
    /// - 0xFFFFFF = continuous RX mode
    pub fn set_rx(&mut self, timeout: u32) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetRx timeout=0x{:06X}", timeout);
        let params = [
            ((timeout >> 16) & 0xFF) as u8,
            ((timeout >> 8) & 0xFF) as u8,
            (timeout & 0xFF) as u8,
        ];
        self.write_command(opcode::SET_RX, &params)
    }

    /// Enable or disable stopping the RX timer on preamble detection.
    pub fn stop_timer_on_preamble(&mut self, enable: bool) -> Result<(), Error<SPI::Error>> {
        self.write_command(opcode::STOP_TIMER_ON_PREAMBLE, &[enable as u8])
    }

    /// Set the device into RX duty cycle mode.
    pub fn set_rx_duty_cycle(
        &mut self,
        rx_period: u32,
        sleep_period: u32,
    ) -> Result<(), Error<SPI::Error>> {
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
        self.write_command(opcode::SET_RX_DUTY_CYCLE, &params)
    }

    /// Set the device into CAD (Channel Activity Detection) mode.
    pub fn set_cad(&mut self) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetCad");
        self.write_command(opcode::SET_CAD, &[])
    }

    /// Set the device to transmit a continuous wave (for testing).
    pub fn set_tx_continuous_wave(&mut self) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetTxContinuousWave");
        self.write_command(opcode::SET_TX_CONTINUOUS_WAVE, &[])
    }

    /// Set the device to transmit an infinite preamble (for testing).
    pub fn set_tx_infinite_preamble(&mut self) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetTxInfinitePreamble");
        self.write_command(opcode::SET_TX_INFINITE_PREAMBLE, &[])
    }

    // -----------------------------------------------------------------------
    // Power / Regulator Configuration
    // -----------------------------------------------------------------------

    /// Set the regulator mode (LDO only or DC-DC+LDO).
    pub fn set_regulator_mode(
        &mut self,
        mode: RegulatorMode,
    ) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetRegulatorMode mode={}", mode);
        self.write_command(opcode::SET_REGULATOR_MODE, &[mode as u8])
    }

    /// Run calibration of the specified blocks.
    pub fn calibrate(&mut self, params: CalibrationParams) -> Result<(), Error<SPI::Error>> {
        defmt::info!("Calibrate params=0x{:02X}", params.0);
        self.write_command(opcode::CALIBRATE, &[params.0])
    }

    /// Calibrate the image rejection for a given frequency range.
    pub fn calibrate_image(
        &mut self,
        freq1: u8,
        freq2: u8,
    ) -> Result<(), Error<SPI::Error>> {
        defmt::info!("CalibrateImage freq1=0x{:02X} freq2=0x{:02X}", freq1, freq2);
        self.write_command(opcode::CALIBRATE_IMAGE, &[freq1, freq2])
    }

    /// Configure the Power Amplifier.
    pub fn set_pa_config(&mut self, config: PaConfig) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetPaConfig config={}", config);
        self.write_command(
            opcode::SET_PA_CONFIG,
            &[
                config.pa_duty_cycle,
                config.hp_max,
                config.device_sel,
                config.pa_lut,
            ],
        )
    }

    /// Set the fallback mode after TX or RX operation completes.
    pub fn set_rx_tx_fallback_mode(
        &mut self,
        mode: FallbackMode,
    ) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetRxTxFallbackMode mode={}", mode);
        self.write_command(opcode::SET_RX_TX_FALLBACK_MODE, &[mode as u8])
    }

    // -----------------------------------------------------------------------
    // DIO and IRQ
    // -----------------------------------------------------------------------

    /// Configure IRQ masks for DIO1, DIO2 and DIO3.
    pub fn set_dio_irq_params(
        &mut self,
        irq_mask: u16,
        dio1_mask: u16,
        dio2_mask: u16,
        dio3_mask: u16,
    ) -> Result<(), Error<SPI::Error>> {
        defmt::info!(
            "SetDioIrqParams irq=0x{:04X} dio1=0x{:04X} dio2=0x{:04X} dio3=0x{:04X}",
            irq_mask,
            dio1_mask,
            dio2_mask,
            dio3_mask
        );
        let params = [
            (irq_mask >> 8) as u8,
            irq_mask as u8,
            (dio1_mask >> 8) as u8,
            dio1_mask as u8,
            (dio2_mask >> 8) as u8,
            dio2_mask as u8,
            (dio3_mask >> 8) as u8,
            dio3_mask as u8,
        ];
        self.write_command(opcode::SET_DIO_IRQ_PARAMS, &params)
    }

    /// Get the current IRQ status.
    pub fn get_irq_status(&mut self) -> Result<u16, Error<SPI::Error>> {
        let mut buf = [0u8; 2];
        self.read_command(opcode::GET_IRQ_STATUS, &mut buf)?;
        let irq = u16::from_be_bytes(buf);
        defmt::debug!("GetIrqStatus irq=0x{:04X}", irq);
        Ok(irq)
    }

    /// Clear the specified IRQ flags.
    pub fn clear_irq_status(&mut self, mask: u16) -> Result<(), Error<SPI::Error>> {
        defmt::debug!("ClearIrqStatus mask=0x{:04X}", mask);
        self.write_command(
            opcode::CLEAR_IRQ_STATUS,
            &[(mask >> 8) as u8, mask as u8],
        )
    }

    /// Set DIO2 as RF switch control.
    pub fn set_dio2_as_rf_switch_ctrl(
        &mut self,
        enable: bool,
    ) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetDIO2AsRfSwitchCtrl enable={}", enable);
        self.write_command(opcode::SET_DIO2_AS_RF_SWITCH_CTRL, &[enable as u8])
    }

    /// Set DIO3 as TCXO control with the specified voltage and timeout.
    ///
    /// `timeout` is in units of 15.625 µs.
    pub fn set_dio3_as_tcxo_ctrl(
        &mut self,
        voltage: TcxoVoltage,
        timeout: u32,
    ) -> Result<(), Error<SPI::Error>> {
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
        self.write_command(opcode::SET_DIO3_AS_TCXO_CTRL, &params)
    }

    // -----------------------------------------------------------------------
    // RF, Modulation and Packet Configuration
    // -----------------------------------------------------------------------

    /// Set the RF frequency in Hz.
    ///
    /// The SX1268 uses a 32-bit frequency word: `freq_word = freq_hz * 2^25 / 32_000_000`.
    pub fn set_rf_frequency(&mut self, frequency_hz: u32) -> Result<(), Error<SPI::Error>> {
        let freq_reg = ((frequency_hz as u64 * (1u64 << 25)) / 32_000_000) as u32;
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
        self.write_command(opcode::SET_RF_FREQUENCY, &params)
    }

    /// Set the packet type (GFSK or LoRa).
    pub fn set_packet_type(
        &mut self,
        packet_type: PacketType,
    ) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetPacketType type={}", packet_type);
        self.write_command(opcode::SET_PACKET_TYPE, &[packet_type as u8])
    }

    /// Get the current packet type.
    pub fn get_packet_type(&mut self) -> Result<PacketType, Error<SPI::Error>> {
        let mut buf = [0u8; 1];
        self.read_command(opcode::GET_PACKET_TYPE, &mut buf)?;
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
    pub fn set_tx_params(
        &mut self,
        power: i8,
        ramp_time: RampTime,
    ) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetTxParams power={}dBm ramp={}", power, ramp_time);
        self.write_command(opcode::SET_TX_PARAMS, &[power as u8, ramp_time as u8])
    }

    /// Set LoRa modulation parameters.
    pub fn set_lora_modulation_params(
        &mut self,
        params: LoRaModulationParams,
    ) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetModulationParams(LoRa) params={}", params);
        let data = [
            params.sf as u8,
            params.bw as u8,
            params.cr as u8,
            params.low_data_rate_optimize as u8,
        ];
        self.write_command(opcode::SET_MODULATION_PARAMS, &data)
    }

    /// Set LoRa packet parameters.
    pub fn set_lora_packet_params(
        &mut self,
        params: LoRaPacketParams,
    ) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetPacketParams(LoRa) params={}", params);
        let data = [
            (params.preamble_length >> 8) as u8,
            params.preamble_length as u8,
            params.header_type as u8,
            params.payload_length,
            params.crc_on as u8,
            params.invert_iq as u8,
        ];
        self.write_command(opcode::SET_PACKET_PARAMS, &data)
    }

    /// Set CAD parameters.
    pub fn set_cad_params(
        &mut self,
        num_symbols: CadSymbols,
        det_peak: u8,
        det_min: u8,
        exit_mode: CadExitMode,
        timeout: u32,
    ) -> Result<(), Error<SPI::Error>> {
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
        self.write_command(opcode::SET_CAD_PARAMS, &params)
    }

    /// Set the TX and RX buffer base addresses.
    pub fn set_buffer_base_address(
        &mut self,
        tx_base: u8,
        rx_base: u8,
    ) -> Result<(), Error<SPI::Error>> {
        defmt::debug!(
            "SetBufferBaseAddress tx=0x{:02X} rx=0x{:02X}",
            tx_base,
            rx_base
        );
        self.write_command(opcode::SET_BUFFER_BASE_ADDRESS, &[tx_base, rx_base])
    }

    /// Set the LoRa symbol number timeout (for RX).
    pub fn set_lora_symb_num_timeout(
        &mut self,
        symb_num: u8,
    ) -> Result<(), Error<SPI::Error>> {
        self.write_command(opcode::SET_LORA_SYMB_NUM_TIMEOUT, &[symb_num])
    }

    // -----------------------------------------------------------------------
    // Status and Diagnostics
    // -----------------------------------------------------------------------

    /// Get the device status.
    pub fn get_status(&mut self) -> Result<Status, Error<SPI::Error>> {
        let mut status_byte = [0u8; 1];
        self.spi
            .transaction(&mut [
                Operation::Write(&[opcode::GET_STATUS]),
                Operation::Read(&mut status_byte),
            ])
            .map_err(Error::Spi)?;
        let status = Status::from(status_byte[0]);
        defmt::debug!("GetStatus status={}", status);
        Ok(status)
    }

    /// Get the instantaneous RSSI value (dBm).
    pub fn get_rssi_inst(&mut self) -> Result<i16, Error<SPI::Error>> {
        let mut buf = [0u8; 1];
        self.read_command(opcode::GET_RSSI_INST, &mut buf)?;
        let rssi = -(buf[0] as i16) / 2;
        defmt::debug!("GetRssiInst rssi={}dBm", rssi);
        Ok(rssi)
    }

    /// Get the RX buffer status (payload length and start pointer).
    pub fn get_rx_buffer_status(&mut self) -> Result<RxBufferStatus, Error<SPI::Error>> {
        let mut buf = [0u8; 2];
        self.read_command(opcode::GET_RX_BUFFER_STATUS, &mut buf)?;
        let status = RxBufferStatus {
            payload_length: buf[0],
            buffer_start_pointer: buf[1],
        };
        defmt::debug!("GetRxBufferStatus status={}", status);
        Ok(status)
    }

    /// Get the LoRa packet status.
    pub fn get_lora_packet_status(&mut self) -> Result<LoRaPacketStatus, Error<SPI::Error>> {
        let mut buf = [0u8; 3];
        self.read_command(opcode::GET_PACKET_STATUS, &mut buf)?;
        let status = LoRaPacketStatus {
            rssi_pkt: -(buf[0] as i16) / 2,
            snr_pkt: (buf[1] as i8) / 4,
            signal_rssi_pkt: -(buf[2] as i16) / 2,
        };
        defmt::debug!("GetLoRaPacketStatus status={}", status);
        Ok(status)
    }

    /// Get the device error flags.
    pub fn get_device_errors(&mut self) -> Result<u16, Error<SPI::Error>> {
        let mut buf = [0u8; 2];
        self.read_command(opcode::GET_DEVICE_ERRORS, &mut buf)?;
        let errors = u16::from_be_bytes(buf);
        defmt::debug!("GetDeviceErrors errors=0x{:04X}", errors);
        Ok(errors)
    }

    /// Clear all device errors.
    pub fn clear_device_errors(&mut self) -> Result<(), Error<SPI::Error>> {
        defmt::debug!("ClearDeviceErrors");
        self.write_command(opcode::CLEAR_DEVICE_ERRORS, &[0x00, 0x00])
    }

    // -----------------------------------------------------------------------
    // Convenience (higher-level) methods
    // -----------------------------------------------------------------------

    /// Set the LoRa sync word.
    ///
    /// Common values: `0x3444` for public network, `0x1424` for private network.
    pub fn set_lora_sync_word(&mut self, sync_word: u16) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SetLoRaSyncWord sync_word=0x{:04X}", sync_word);
        self.write_register(
            opcode::REG_LORA_SYNC_WORD_MSB,
            &[(sync_word >> 8) as u8, sync_word as u8],
        )
    }

    /// Send a packet of data using LoRa modulation.
    ///
    /// This writes the payload to the TX buffer, configures the payload length,
    /// and initiates transmission. The caller should check IRQ status for TX_DONE.
    ///
    /// `timeout` is the TX timeout in units of 15.625 µs (0 = no timeout).
    pub fn send_lora(
        &mut self,
        data: &[u8],
        timeout: u32,
    ) -> Result<(), Error<SPI::Error>> {
        defmt::info!("SendLoRa len={} timeout={}", data.len(), timeout);
        self.set_buffer_base_address(0x00, 0x00)?;
        self.write_buffer(0x00, data)?;
        self.set_tx(timeout)?;
        Ok(())
    }

    /// Read a received LoRa packet from the RX buffer.
    ///
    /// Returns the number of bytes read. `buf` must be large enough to hold the
    /// received payload.
    pub fn read_lora_packet(&mut self, buf: &mut [u8]) -> Result<usize, Error<SPI::Error>> {
        let rx_status = self.get_rx_buffer_status()?;
        let len = rx_status.payload_length as usize;
        if len > buf.len() {
            defmt::warn!(
                "RX payload ({} bytes) exceeds buffer ({} bytes)",
                len,
                buf.len()
            );
            return Err(Error::InvalidParameter);
        }
        self.read_buffer(rx_status.buffer_start_pointer, &mut buf[..len])?;
        defmt::info!("ReadLoRaPacket len={}", len);
        Ok(len)
    }

    /// Apply a full configuration to the SX1268.
    ///
    /// This sets up the device from standby by applying all settings from the
    /// provided [`Sx1268Config`] in the correct order:
    ///
    /// 1. Set standby (RC)
    /// 2. Set regulator mode
    /// 3. Set packet type
    /// 4. Set RF frequency
    /// 5. Set PA config
    /// 6. Set TX params (power + ramp)
    /// 7. Set buffer base address
    /// 8. Set LoRa modulation params
    /// 9. Set LoRa packet params
    /// 10. Set LoRa sync word
    /// 11. Set DIO2 as RF switch
    /// 12. Set fallback mode
    pub fn apply_config(&mut self, config: &Sx1268Config) -> Result<(), Error<SPI::Error>> {
        defmt::info!("ApplyConfig config={}", config);
        self.set_standby(StandbyConfig::StbyRc)?;
        self.set_regulator_mode(config.regulator_mode)?;
        self.set_packet_type(config.packet_type)?;
        self.set_rf_frequency(config.frequency_hz)?;
        self.set_pa_config(config.pa_config)?;
        self.set_tx_params(config.tx_power, config.ramp_time)?;
        self.set_buffer_base_address(config.tx_base_address, config.rx_base_address)?;
        self.set_lora_modulation_params(config.lora_modulation)?;
        self.set_lora_packet_params(config.lora_packet)?;
        self.set_lora_sync_word(config.lora_sync_word)?;
        self.set_dio2_as_rf_switch_ctrl(config.dio2_as_rf_switch)?;
        self.set_rx_tx_fallback_mode(config.fallback_mode)?;
        Ok(())
    }
}
