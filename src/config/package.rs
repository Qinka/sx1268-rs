//! Packet format types for the SX1268.
//!
//! The SX1268 supports two packet types — GFSK and LoRa.  This module
//! defines the [`PacketType`] selector and the LoRa-specific packet
//! parameters ([`LoRaPacketParams`]).

/// Packet type selector.
///
/// The SX1268 can operate in either GFSK or LoRa mode. The packet type
/// must be configured **before** setting modulation or packet parameters,
/// because the interpretation of subsequent commands depends on it.
#[derive(Default, Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum PacketType {
  /// GFSK (Gaussian Frequency-Shift Keying) modulation.
  Gfsk = 0x00,
  /// LoRa (Long Range) chirp spread-spectrum modulation.
  #[default]
  LoRa = 0x01,
}

/// LoRa packet header type.
///
/// - **Explicit** (variable-length): the header carries the payload length,
///   coding rate and CRC presence. The receiver can decode any valid packet.
/// - **Implicit** (fixed-length): no header is transmitted. Both TX and RX
///   must agree on payload length, coding rate and CRC setting in advance.
///   This saves a few bytes of airtime.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum LoRaHeaderType {
  /// Variable length packet (explicit header).
  Explicit = 0x00,
  /// Fixed length packet (implicit header).
  Implicit = 0x01,
}

/// LoRa packet parameters.
///
/// These parameters control the over-the-air packet structure and are
/// sent to the chip via the `SetPacketParams` command (opcode `0x8C`).
///
/// # Fields
///
/// | Field             | Description |
/// |-------------------|-------------|
/// | `preamble_length` | Number of preamble symbols (typically 8). |
/// | `header_type`     | Explicit (variable-length) or Implicit (fixed-length). |
/// | `payload_length`  | Maximum (or fixed) payload size in bytes. |
/// | `crc_on`          | Enable CRC-16 for payload integrity checking. |
/// | `invert_iq`       | Invert I/Q signals — must be `true` on one side of a gateway↔node link to improve packet rejection of self-generated echoes. |
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct LoRaPacketParams {
  /// Preamble length in symbols.
  ///
  /// 前导码长度决定了接收端用于同步和载波频率检测的符号数量。
  /// 典型值为 8 个符号；增加前导码长度可改善在弱信号下的同步成功率，
  /// 但会增加空中时间。
  pub(crate) preamble_length: u16,

  /// Header type (explicit or implicit).
  pub(crate) header_type: LoRaHeaderType,

  /// Payload length in bytes (0–255).
  ///
  /// 在显式报头模式下该值为最大有效载荷长度；在隐式报头模式下
  /// 收发双方必须配置相同的固定长度。
  pub(crate) payload_length: u8,

  /// CRC enable.
  ///
  /// 启用后会在有效载荷后附加 16-bit CRC，接收端可据此丢弃损坏的帧。
  pub(crate) crc_on: bool,

  /// IQ inversion.
  ///
  /// LoRa 网络中，网关与终端节点通常使用相反的 IQ 极性，以增强对自
  /// 身发射信号回波的抑制能力。
  pub(crate) invert_iq: bool,
}

impl Default for LoRaPacketParams {
  fn default() -> Self {
    Self {
      preamble_length: 8,
      header_type: LoRaHeaderType::Explicit,
      payload_length: 0,
      crc_on: true,
      invert_iq: false,
    }
  }
}

impl LoRaPacketParams {
  /// Set the preamble length in symbols.
  pub fn with_preamble_length(mut self, preamble_length: u16) -> Self {
    self.preamble_length = preamble_length;
    self
  }

  /// Set the header type (explicit or implicit).
  pub fn with_header_type(mut self, header_type: LoRaHeaderType) -> Self {
    self.header_type = header_type;
    self
  }

  /// Set the payload length in bytes.
  pub fn with_payload_length(mut self, payload_length: u8) -> Self {
    self.payload_length = payload_length;
    self
  }

  /// Enable or disable the 16-bit CRC.
  pub fn with_crc_on(mut self, crc_on: bool) -> Self {
    self.crc_on = crc_on;
    self
  }

  /// Enable or disable IQ signal inversion.
  pub fn with_invert_iq(mut self, invert_iq: bool) -> Self {
    self.invert_iq = invert_iq;
    self
  }
}
