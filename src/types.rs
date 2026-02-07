//! Configuration types and enumerations for the SX1268 driver.

/// Sleep configuration.
#[derive(Clone, Copy, Debug, defmt::Format)]
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

/// Regulator mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum RegulatorMode {
    /// Only LDO.
    Ldo = 0x00,
    /// DC-DC + LDO.
    DcDcLdo = 0x01,
}

/// Packet type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum PacketType {
    /// GFSK modulation.
    Gfsk = 0x00,
    /// LoRa modulation.
    LoRa = 0x01,
}

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

/// LoRa modulation parameters.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct LoRaModulationParams {
    pub sf: LoRaSpreadingFactor,
    pub bw: LoRaBandwidth,
    pub cr: LoRaCodingRate,
    pub low_data_rate_optimize: bool,
}

/// LoRa packet header type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum LoRaHeaderType {
    /// Variable length packet (explicit header).
    Explicit = 0x00,
    /// Fixed length packet (implicit header).
    Implicit = 0x01,
}

/// LoRa packet parameters.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct LoRaPacketParams {
    /// Preamble length in symbols.
    pub preamble_length: u16,
    /// Header type.
    pub header_type: LoRaHeaderType,
    /// Payload length in bytes.
    pub payload_length: u8,
    /// CRC enable.
    pub crc_on: bool,
    /// IQ inversion.
    pub invert_iq: bool,
}

/// PA (Power Amplifier) configuration.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct PaConfig {
    /// PA duty cycle (raw register value).
    pub pa_duty_cycle: u8,
    /// HP max (raw register value).
    pub hp_max: u8,
    /// Device selection: 0x00 = SX1268.
    pub device_sel: u8,
    /// PA LUT (always 0x01).
    pub pa_lut: u8,
}

impl Default for PaConfig {
    fn default() -> Self {
        // Default: +22 dBm on SX1268
        PaConfig {
            pa_duty_cycle: 0x04,
            hp_max: 0x07,
            device_sel: 0x00,
            pa_lut: 0x01,
        }
    }
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

/// Fallback mode after TX/RX.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum FallbackMode {
    StbyRc = 0x20,
    StbyXosc = 0x30,
    Fs = 0x40,
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

/// IRQ flags bitmask.
pub mod irq {
    pub const TX_DONE: u16 = 1 << 0;
    pub const RX_DONE: u16 = 1 << 1;
    pub const PREAMBLE_DETECTED: u16 = 1 << 2;
    pub const SYNC_WORD_VALID: u16 = 1 << 3;
    pub const HEADER_VALID: u16 = 1 << 4;
    pub const HEADER_ERR: u16 = 1 << 5;
    pub const CRC_ERR: u16 = 1 << 6;
    pub const CAD_DONE: u16 = 1 << 7;
    pub const CAD_DETECTED: u16 = 1 << 8;
    pub const TIMEOUT: u16 = 1 << 9;
    pub const ALL: u16 = 0x03FF;
    pub const NONE: u16 = 0x0000;
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
