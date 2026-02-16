// 该文件是 sx1268-rs 项目的一部分。
// src/codes.rs - SPI 命令操作码和寄存器地址
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

//! SX1268 SPI command opcodes and register addresses.
//!
//! All opcodes and register addresses are taken from the
//! *SX1268 Datasheet DS.SX1261-2.W.APP, Rev 2.1* (Semtech, 2022).
//!
//! The SX1268 uses a synchronous SPI interface. Each command starts with
//! an opcode byte followed by zero or more parameter bytes. The chip
//! returns a status byte on every transaction.
//!
//! ## Command categories
//!
//! | Category                  | Opcodes |
//! |---------------------------|---------|
//! | Operational Modes         | `SetSleep`, `SetStandby`, `SetFs`, `SetTx`, `SetRx`, … |
//! | Register / Buffer Access  | `WriteRegister`, `ReadRegister`, `WriteBuffer`, `ReadBuffer` |
//! | DIO & IRQ                 | `SetDioIrqParams`, `GetIrqStatus`, `ClearIrqStatus`, … |
//! | RF / Modulation / Packet  | `SetRfFrequency`, `SetPacketType`, `SetModulationParams`, … |
//! | Status                    | `GetStatus`, `GetRssiInst`, `GetRxBufferStatus`, … |

// ---------------------------------------------------------------------------
// Operational Modes (Datasheet §13.1)
// ---------------------------------------------------------------------------

/// Enter sleep mode. Opcode `0x84`.
///
/// Puts the device into the lowest-power state. Configuration can
/// optionally be retained for a warm-start wakeup.
pub const SET_SLEEP: u8 = 0x84;

/// Enter standby mode. Opcode `0x80`.
///
/// Two sub-modes are available: STDBY_RC (13 MHz RC oscillator) and
/// STDBY_XOSC (32 MHz crystal). Standby is the default idle state.
pub const SET_STANDBY: u8 = 0x80;

/// Enter frequency-synthesis (FS) mode. Opcode `0xC1`.
///
/// The PLL is locked to the configured frequency but no packet
/// transmission or reception is performed. Useful as a low-latency
/// intermediate state before TX/RX.
pub const SET_FS: u8 = 0xC1;

/// Enter transmit mode. Opcode `0x83`.
///
/// A 24-bit timeout value (units of 15.625 µs) controls the maximum
/// transmission duration. `0x000000` means no timeout (single TX).
pub const SET_TX: u8 = 0x83;

/// Enter receive mode. Opcode `0x82`.
///
/// A 24-bit timeout value (units of 15.625 µs) controls the maximum
/// listening window. Special values:
/// - `0x000000` — single RX (no timeout)
/// - `0xFFFFFF` — continuous RX
pub const SET_RX: u8 = 0x82;

/// Stop the RX timer on preamble detect. Opcode `0x9F`.
pub const STOP_TIMER_ON_PREAMBLE: u8 = 0x9F;

/// Enter RX duty-cycle mode. Opcode `0x94`.
///
/// The device alternates between an RX window and a sleep period,
/// useful for low-power listening scenarios.
pub const SET_RX_DUTY_CYCLE: u8 = 0x94;

/// Start Channel Activity Detection. Opcode `0xC5`.
///
/// CAD detects whether LoRa preamble energy is present on the channel
/// before initiating a transmission (Listen-Before-Talk).
pub const SET_CAD: u8 = 0xC5;

/// Transmit a continuous unmodulated carrier (test mode). Opcode `0xD1`.
pub const SET_TX_CONTINUOUS_WAVE: u8 = 0xD1;

/// Transmit an infinite preamble (test mode). Opcode `0xD2`.
pub const SET_TX_INFINITE_PREAMBLE: u8 = 0xD2;

/// Set the voltage regulator mode. Opcode `0x96`.
///
/// `0x00` — LDO only; `0x01` — DC-DC + LDO (more power-efficient).
pub const SET_REGULATOR_MODE: u8 = 0x96;

/// Run internal calibration blocks. Opcode `0x89`.
///
/// A bitmask selects which blocks to calibrate (RC64k, RC13M, PLL,
/// ADC pulse, ADC bulk N/P, image).
pub const CALIBRATE: u8 = 0x89;

/// Calibrate the image-rejection filter. Opcode `0x98`.
///
/// Takes two frequency-range bytes to tune the image rejection for
/// the band in use.
pub const CALIBRATE_IMAGE: u8 = 0x98;

/// Configure the Power Amplifier. Opcode `0x95`.
///
/// Parameters: `paDutyCycle`, `hpMax`, `deviceSel`, `paLut`.
pub const SET_PA_CONFIG: u8 = 0x95;

/// Set the fallback mode after TX or RX completes. Opcode `0x93`.
///
/// The device will automatically transition to STDBY_RC, STDBY_XOSC
/// or FS after a transmission or reception ends.
pub const SET_RX_TX_FALLBACK_MODE: u8 = 0x93;

// ---------------------------------------------------------------------------
// Register and Buffer Access (Datasheet §13.2)
// ---------------------------------------------------------------------------

/// Write to one or more contiguous registers. Opcode `0x0D`.
///
/// Format: `[0x0D, addr_hi, addr_lo, data…]`
pub const WRITE_REGISTER: u8 = 0x0D;

/// Read from one or more contiguous registers. Opcode `0x1D`.
///
/// Format: `[0x1D, addr_hi, addr_lo, NOP] → [data…]`
pub const READ_REGISTER: u8 = 0x1D;

/// Write data into the TX data buffer. Opcode `0x0E`.
///
/// Format: `[0x0E, offset, data…]`
pub const WRITE_BUFFER: u8 = 0x0E;

/// Read data from the RX data buffer. Opcode `0x1E`.
///
/// Format: `[0x1E, offset, NOP] → [data…]`
pub const READ_BUFFER: u8 = 0x1E;

// ---------------------------------------------------------------------------
// DIO and IRQ (Datasheet §13.3)
// ---------------------------------------------------------------------------

/// Configure the IRQ sources routed to DIO1 / DIO2 / DIO3. Opcode `0x08`.
///
/// Parameters are four 16-bit masks (big-endian):
/// `[irq_mask, dio1_mask, dio2_mask, dio3_mask]`.
pub const SET_DIO_IRQ_PARAMS: u8 = 0x08;

/// Read the current pending IRQ flags. Opcode `0x12`.
///
/// Returns a 16-bit bitmask (big-endian) of active interrupt sources.
pub const GET_IRQ_STATUS: u8 = 0x12;

/// Clear specified IRQ flags. Opcode `0x02`.
///
/// Write a 16-bit bitmask (big-endian) to acknowledge the given IRQs.
pub const CLEAR_IRQ_STATUS: u8 = 0x02;

/// Configure DIO2 to automatically control an external RF switch.
/// Opcode `0x9D`.
///
/// When enabled, DIO2 goes high during TX and low otherwise, which
/// can directly drive a typical RF front-end switch.
pub const SET_DIO2_AS_RF_SWITCH_CTRL: u8 = 0x9D;

/// Configure DIO3 to supply the TCXO reference voltage. Opcode `0x97`.
///
/// Parameters: `[voltage, timeout_23..16, timeout_15..8, timeout_7..0]`.
/// The timeout is in units of 15.625 µs and defines how long the chip
/// waits for the TCXO to stabilize before proceeding.
pub const SET_DIO3_AS_TCXO_CTRL: u8 = 0x97;

// ---------------------------------------------------------------------------
// RF, Modulation and Packet (Datasheet §13.4)
// ---------------------------------------------------------------------------

/// Set the RF carrier frequency. Opcode `0x86`.
///
/// The frequency is expressed as a 32-bit PLL step count:
///
/// ```text
/// freq_reg = frequency_hz × 2²⁵ / F_XTAL
/// ```
///
/// where `F_XTAL = 32 MHz`.
pub const SET_RF_FREQUENCY: u8 = 0x86;

/// Set the packet type (GFSK = 0x00, LoRa = 0x01). Opcode `0x8A`.
pub const SET_PACKET_TYPE: u8 = 0x8A;

/// Read the currently configured packet type. Opcode `0x11`.
pub const GET_PACKET_TYPE: u8 = 0x11;

/// Set transmit power and PA ramp time. Opcode `0x8E`.
///
/// Parameters: `[power_dBm, ramp_time]`.
/// Power ranges from −9 to +22 dBm for SX1268 (high-power PA).
pub const SET_TX_PARAMS: u8 = 0x8E;

/// Set LoRa (or GFSK) modulation parameters. Opcode `0x8B`.
///
/// For LoRa the parameters are:
/// `[spreading_factor, bandwidth, coding_rate, low_data_rate_optimize]`.
pub const SET_MODULATION_PARAMS: u8 = 0x8B;

/// Set LoRa (or GFSK) packet parameters. Opcode `0x8C`.
///
/// For LoRa the parameters are:
/// `[preamble_hi, preamble_lo, header_type, payload_len, crc_on, invert_iq]`.
pub const SET_PACKET_PARAMS: u8 = 0x8C;

/// Set Channel Activity Detection parameters. Opcode `0x88`.
pub const SET_CAD_PARAMS: u8 = 0x88;

/// Set the TX / RX buffer base addresses in the 256-byte data buffer.
/// Opcode `0x8F`.
///
/// Parameters: `[tx_base_address, rx_base_address]`.
pub const SET_BUFFER_BASE_ADDRESS: u8 = 0x8F;

/// Set LoRa symbol number timeout for RX. Opcode `0xA0`.
pub const SET_LORA_SYMB_NUM_TIMEOUT: u8 = 0xA0;

// ---------------------------------------------------------------------------
// Status (Datasheet §13.5)
// ---------------------------------------------------------------------------

/// Get the transceiver status byte. Opcode `0xC0`.
///
/// The returned byte encodes both the current chip mode (bits 6:4) and
/// the last command status (bits 3:1).
pub const GET_STATUS: u8 = 0xC0;

/// Get instantaneous RSSI in RX mode. Opcode `0x15`.
///
/// The raw value is an unsigned byte; actual RSSI = −(raw / 2) dBm.
pub const GET_RSSI_INST: u8 = 0x15;

/// Get the RX buffer status. Opcode `0x13`.
///
/// Returns `[payload_length, buffer_start_pointer]`.
pub const GET_RX_BUFFER_STATUS: u8 = 0x13;

/// Get the last LoRa packet status. Opcode `0x14`.
///
/// Returns `[rssi_pkt_raw, snr_pkt_raw, signal_rssi_pkt_raw]`.
pub const GET_PACKET_STATUS: u8 = 0x14;

/// Get the device error flags. Opcode `0x17`.
///
/// Returns a 16-bit bitmask (big-endian) of error conditions.
pub const GET_DEVICE_ERRORS: u8 = 0x17;

/// Clear all device error flags. Opcode `0x07`.
pub const CLEAR_DEVICE_ERRORS: u8 = 0x07;

// ---------------------------------------------------------------------------
// Register Addresses (Datasheet §12)
// ---------------------------------------------------------------------------

/// LoRa sync-word MSB register. Address `0x0740`.
///
/// Together with [`REG_LORA_SYNC_WORD_LSB`] this 16-bit value identifies
/// the LoRa network:
/// - `0x3444` — public (LoRaWAN)
/// - `0x1424` — private
pub const REG_LORA_SYNC_WORD_MSB: u16 = 0x0740;

/// LoRa sync-word LSB register. Address `0x0741`.
pub const REG_LORA_SYNC_WORD_LSB: u16 = 0x0741;

/// RX gain register. Address `0x08AC`.
///
/// Writing `0x96` enables the power-saving RX gain mode, while `0x94`
/// selects the boosted (higher-sensitivity) gain.
pub const REG_RX_GAIN: u16 = 0x08AC;

/// TX clamp configuration register. Address `0x08D8`.
pub const REG_TX_CLAMP_CONFIG: u16 = 0x08D8;

/// Over-Current Protection configuration register. Address `0x08E7`.
pub const REG_OCP_CONFIGURATION: u16 = 0x08E7;

/// TX modulation quality register. Address `0x0889`.
///
/// Bit 2 of this register must be managed as a workaround when using
/// 500 kHz LoRa bandwidth (see Errata, §15.1).
pub const REG_TX_MODULATION: u16 = 0x0889;

/// IQ polarity setup register. Address `0x0736`.
///
/// Bit 2 controls IQ inversion and must be patched after calling
/// `SetPacketParams` (see Errata, §15.4).
pub const REG_IQ_POLARITY: u16 = 0x0736;
