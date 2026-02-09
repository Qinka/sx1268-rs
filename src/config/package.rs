/// Packet type.
#[derive(Default, Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum PacketType {
  /// GFSK modulation.
  Gfsk = 0x00,
  /// LoRa modulation.
  #[default]
  LoRa = 0x01,
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
  pub(crate) preamble_length: u16,
  /// Header type.
  pub(crate) header_type: LoRaHeaderType,
  /// Payload length in bytes.
  pub(crate) payload_length: u8,
  /// CRC enable.
  pub(crate) crc_on: bool,
  /// IQ inversion.
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
  /// setup preamble length in symbols.
  pub fn with_preamble_length(mut self, preamble_length: u16) -> Self {
    self.preamble_length = preamble_length;
    self
  }

  /// setup header type.
  pub fn with_header_type(mut self, header_type: LoRaHeaderType) -> Self {
    self.header_type = header_type;
    self
  }

  /// setup payload length in bytes.
  pub fn with_payload_length(mut self, payload_length: u8) -> Self {
    self.payload_length = payload_length;
    self
  }

  /// setup CRC enable.
  pub fn with_crc_on(mut self, crc_on: bool) -> Self {
    self.crc_on = crc_on;
    self
  }

  /// setup IQ inversion.
  pub fn with_invert_iq(mut self, invert_iq: bool) -> Self {
    self.invert_iq = invert_iq;
    self
  }
}
