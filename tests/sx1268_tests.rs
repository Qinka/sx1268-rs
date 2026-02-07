#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};
    use sx1268_rs::types::*;
    use sx1268_rs::Sx1268;

    /// Helper: build mock SPI expectations for a write_command call.
    /// write_command does: Write([opcode]), Write(params)
    fn write_cmd_expectations(opcode: u8, params: &[u8]) -> Vec<SpiTransaction<u8>> {
        vec![
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![opcode]),
            SpiTransaction::write_vec(params.to_vec()),
            SpiTransaction::transaction_end(),
        ]
    }

    /// Helper: build mock SPI expectations for a read_command call.
    /// read_command does: Write([opcode, 0x00]), Read([status]), Read(response)
    fn read_cmd_expectations(
        opcode: u8,
        status: u8,
        response: &[u8],
    ) -> Vec<SpiTransaction<u8>> {
        vec![
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![opcode, 0x00]),
            SpiTransaction::read_vec(vec![status]),
            SpiTransaction::read_vec(response.to_vec()),
            SpiTransaction::transaction_end(),
        ]
    }

    #[test]
    fn test_set_standby_rc() {
        let expectations = write_cmd_expectations(0x80, &[0x00]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_standby(StandbyConfig::StbyRc).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_standby_xosc() {
        let expectations = write_cmd_expectations(0x80, &[0x01]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_standby(StandbyConfig::StbyXosc).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_sleep_warm_start() {
        let config = SleepConfig {
            warm_start: true,
            rtc_wakeup: false,
        };
        let expectations = write_cmd_expectations(0x84, &[0x04]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_sleep(config).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_sleep_cold_start_rtc() {
        let config = SleepConfig {
            warm_start: false,
            rtc_wakeup: true,
        };
        let expectations = write_cmd_expectations(0x84, &[0x01]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_sleep(config).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_fs() {
        let expectations = write_cmd_expectations(0xC1, &[]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_fs().unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_tx() {
        // timeout = 0x01_00_00 (65536 * 15.625 µs)
        let expectations = write_cmd_expectations(0x83, &[0x01, 0x00, 0x00]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_tx(0x010000).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_rx_continuous() {
        // 0xFFFFFF = continuous RX
        let expectations = write_cmd_expectations(0x82, &[0xFF, 0xFF, 0xFF]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_rx(0xFFFFFF).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_regulator_mode() {
        let expectations = write_cmd_expectations(0x96, &[0x01]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_regulator_mode(RegulatorMode::DcDcLdo).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_packet_type_lora() {
        let expectations = write_cmd_expectations(0x8A, &[0x01]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_packet_type(PacketType::LoRa).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_rf_frequency() {
        // 868 MHz: freq_reg = 868_000_000 * 2^25 / 32_000_000
        let freq_hz: u32 = 868_000_000;
        let freq_reg = ((freq_hz as u64 * (1u64 << 25)) / 32_000_000) as u32;
        let expected_params = [
            ((freq_reg >> 24) & 0xFF) as u8,
            ((freq_reg >> 16) & 0xFF) as u8,
            ((freq_reg >> 8) & 0xFF) as u8,
            (freq_reg & 0xFF) as u8,
        ];
        let expectations = write_cmd_expectations(0x86, &expected_params);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_rf_frequency(freq_hz).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_pa_config() {
        let config = PaConfig::default();
        let expectations = write_cmd_expectations(0x95, &[0x04, 0x07, 0x00, 0x01]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_pa_config(config).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_tx_params() {
        // +22 dBm, ramp 200 µs
        let expectations = write_cmd_expectations(0x8E, &[22u8, 0x04]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_tx_params(22, RampTime::Ramp200Us).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_buffer_base_address() {
        let expectations = write_cmd_expectations(0x8F, &[0x00, 0x00]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_buffer_base_address(0x00, 0x00).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_lora_modulation_params() {
        let params = LoRaModulationParams {
            sf: LoRaSpreadingFactor::Sf7,
            bw: LoRaBandwidth::Bw125,
            cr: LoRaCodingRate::Cr4_5,
            low_data_rate_optimize: false,
        };
        let expectations = write_cmd_expectations(0x8B, &[0x07, 0x04, 0x01, 0x00]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_lora_modulation_params(params).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_lora_packet_params() {
        let params = LoRaPacketParams {
            preamble_length: 8,
            header_type: LoRaHeaderType::Explicit,
            payload_length: 64,
            crc_on: true,
            invert_iq: false,
        };
        let expectations = write_cmd_expectations(0x8C, &[0x00, 0x08, 0x00, 64, 0x01, 0x00]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_lora_packet_params(params).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_dio_irq_params() {
        let expectations = write_cmd_expectations(
            0x08,
            &[0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00],
        );
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio
            .set_dio_irq_params(0x0003, 0x0003, 0x0000, 0x0000)
            .unwrap();
        radio.release().done();
    }

    #[test]
    fn test_get_irq_status() {
        // Simulate TX_DONE (bit 0) set
        let expectations = read_cmd_expectations(0x12, 0x00, &[0x00, 0x01]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        let irq = radio.get_irq_status().unwrap();
        assert_eq!(irq, 0x0001);
        radio.release().done();
    }

    #[test]
    fn test_clear_irq_status() {
        let expectations = write_cmd_expectations(0x02, &[0x03, 0xFF]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.clear_irq_status(0x03FF).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_dio2_as_rf_switch() {
        let expectations = write_cmd_expectations(0x9D, &[0x01]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_dio2_as_rf_switch_ctrl(true).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_dio3_as_tcxo() {
        let expectations = write_cmd_expectations(0x97, &[0x06, 0x00, 0x01, 0x40]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio
            .set_dio3_as_tcxo_ctrl(TcxoVoltage::Ctrl3v0, 0x000140)
            .unwrap();
        radio.release().done();
    }

    #[test]
    fn test_get_status() {
        // GetStatus: Write([0xC0]), Read([status_byte])
        let expectations = vec![
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0xC0]),
            SpiTransaction::read_vec(vec![0x54]), // chip_mode=Rx(0x05), cmd_status=DataAvailable(0x02)
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        let status = radio.get_status().unwrap();
        assert_eq!(status.chip_mode, ChipMode::Rx);
        assert_eq!(status.command_status, CommandStatus::DataAvailable);
        radio.release().done();
    }

    #[test]
    fn test_get_rssi_inst() {
        // Response: rssi_raw = 100 → rssi = -100/2 = -50 dBm
        let expectations = read_cmd_expectations(0x15, 0x00, &[100]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        let rssi = radio.get_rssi_inst().unwrap();
        assert_eq!(rssi, -50);
        radio.release().done();
    }

    #[test]
    fn test_get_rx_buffer_status() {
        let expectations = read_cmd_expectations(0x13, 0x00, &[32, 0x00]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        let status = radio.get_rx_buffer_status().unwrap();
        assert_eq!(status.payload_length, 32);
        assert_eq!(status.buffer_start_pointer, 0x00);
        radio.release().done();
    }

    #[test]
    fn test_get_lora_packet_status() {
        // rssi_pkt_raw=100 → -50dBm, snr_raw=40 → 10dB, signal_rssi_raw=90 → -45dBm
        let expectations = read_cmd_expectations(0x14, 0x00, &[100, 40, 90]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        let status = radio.get_lora_packet_status().unwrap();
        assert_eq!(status.rssi_pkt, -50);
        assert_eq!(status.snr_pkt, 10);
        assert_eq!(status.signal_rssi_pkt, -45);
        radio.release().done();
    }

    #[test]
    fn test_get_device_errors() {
        let expectations = read_cmd_expectations(0x17, 0x00, &[0x00, 0x00]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        let errors = radio.get_device_errors().unwrap();
        assert_eq!(errors, 0);
        radio.release().done();
    }

    #[test]
    fn test_clear_device_errors() {
        let expectations = write_cmd_expectations(0x07, &[0x00, 0x00]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.clear_device_errors().unwrap();
        radio.release().done();
    }

    #[test]
    fn test_write_buffer() {
        // WriteBuffer: Write([0x0E, offset]), Write(data)
        let expectations = vec![
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x0E, 0x00]),
            SpiTransaction::write_vec(vec![0xDE, 0xAD, 0xBE, 0xEF]),
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.write_buffer(0x00, &[0xDE, 0xAD, 0xBE, 0xEF]).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_read_buffer() {
        // ReadBuffer: Write([0x1E, offset, 0x00]), Read(data)
        let expectations = vec![
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x1E, 0x00, 0x00]),
            SpiTransaction::read_vec(vec![0xCA, 0xFE]),
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        let mut buf = [0u8; 2];
        radio.read_buffer(0x00, &mut buf).unwrap();
        assert_eq!(buf, [0xCA, 0xFE]);
        radio.release().done();
    }

    #[test]
    fn test_write_register() {
        // WriteRegister: Write([0x0D, addr_hi, addr_lo]), Write(data)
        let expectations = vec![
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x0D, 0x07, 0x40]),
            SpiTransaction::write_vec(vec![0x34, 0x44]),
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.write_register(0x0740, &[0x34, 0x44]).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_lora_sync_word() {
        // Uses write_register to address 0x0740
        let expectations = vec![
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x0D, 0x07, 0x40]),
            SpiTransaction::write_vec(vec![0x34, 0x44]),
            SpiTransaction::transaction_end(),
        ];
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_lora_sync_word(0x3444).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_calibrate() {
        let expectations = write_cmd_expectations(0x89, &[0x7F]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.calibrate(CalibrationParams::ALL).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_calibrate_image() {
        let expectations = write_cmd_expectations(0x98, &[0xD7, 0xDB]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.calibrate_image(0xD7, 0xDB).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_rx_tx_fallback_mode() {
        let expectations = write_cmd_expectations(0x93, &[0x20]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio
            .set_rx_tx_fallback_mode(FallbackMode::StbyRc)
            .unwrap();
        radio.release().done();
    }

    #[test]
    fn test_set_cad() {
        let expectations = write_cmd_expectations(0xC5, &[]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.set_cad().unwrap();
        radio.release().done();
    }

    #[test]
    fn test_get_packet_type_lora() {
        let expectations = read_cmd_expectations(0x11, 0x00, &[0x01]);
        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        let pt = radio.get_packet_type().unwrap();
        assert_eq!(pt, PacketType::LoRa);
        radio.release().done();
    }

    #[test]
    fn test_send_lora() {
        let data = [0x48, 0x65, 0x6C, 0x6C, 0x6F]; // "Hello"
        let mut expectations = Vec::new();

        // set_buffer_base_address(0x00, 0x00)
        expectations.extend(write_cmd_expectations(0x8F, &[0x00, 0x00]));

        // write_buffer(0x00, data)
        expectations.push(SpiTransaction::transaction_start());
        expectations.push(SpiTransaction::write_vec(vec![0x0E, 0x00]));
        expectations.push(SpiTransaction::write_vec(data.to_vec()));
        expectations.push(SpiTransaction::transaction_end());

        // set_tx(0)
        expectations.extend(write_cmd_expectations(0x83, &[0x00, 0x00, 0x00]));

        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        radio.send_lora(&data, 0).unwrap();
        radio.release().done();
    }

    #[test]
    fn test_read_lora_packet() {
        let mut expectations = Vec::new();

        // get_rx_buffer_status → payload_length=5, start=0
        expectations.extend(read_cmd_expectations(0x13, 0x00, &[5, 0x00]));

        // read_buffer(0x00, &mut buf[..5])
        expectations.push(SpiTransaction::transaction_start());
        expectations.push(SpiTransaction::write_vec(vec![0x1E, 0x00, 0x00]));
        expectations.push(SpiTransaction::read_vec(vec![0x48, 0x65, 0x6C, 0x6C, 0x6F]));
        expectations.push(SpiTransaction::transaction_end());

        let spi = SpiMock::new(&expectations);
        let mut radio = Sx1268::new(spi);
        let mut buf = [0u8; 64];
        let len = radio.read_lora_packet(&mut buf).unwrap();
        assert_eq!(len, 5);
        assert_eq!(&buf[..5], &[0x48, 0x65, 0x6C, 0x6C, 0x6F]);
        radio.release().done();
    }

    #[test]
    fn test_status_parsing() {
        // 0x54: chip_mode = (5 << 4) | cmd_status = (2 << 1)
        // bits: 0101_0100
        // chip_mode = (0x54 >> 4) & 0x07 = 0x05 = Rx
        // command_status = (0x54 >> 1) & 0x07 = 0x02 = DataAvailable
        let status = Status::from(0x54);
        assert_eq!(status.chip_mode, ChipMode::Rx);
        assert_eq!(status.command_status, CommandStatus::DataAvailable);
    }

    #[test]
    fn test_sleep_config_byte() {
        let config = SleepConfig {
            warm_start: true,
            rtc_wakeup: true,
        };
        assert_eq!(config.to_byte(), 0x05);

        let config = SleepConfig {
            warm_start: false,
            rtc_wakeup: false,
        };
        assert_eq!(config.to_byte(), 0x00);
    }

    #[test]
    fn test_irq_flags() {
        use sx1268_rs::types::irq;
        assert_eq!(irq::TX_DONE, 0x0001);
        assert_eq!(irq::RX_DONE, 0x0002);
        assert_eq!(irq::TIMEOUT, 0x0200);
        assert_eq!(irq::ALL, 0x03FF);
    }
}
