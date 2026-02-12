pub trait Control {
  type Status;
  type Error;

  /// write_command sends a command with the given opcode and parameters to the SX1268, returning the resulting status or an error.
  fn write_command(&mut self, opcode: u8, params: &[u8]) -> Result<(), Self::Error>;
  /// read_command sends a command with the given opcode and parameters to the SX1268, then reads back the resulting status or an error.
  fn read_command(&mut self, opcode: u8, params: &mut [u8]) -> Result<Self::Status, Self::Error>;

  /// write register writes a value to the specified register address.
  fn write_register(&mut self, address: u16, data: &[u8]) -> Result<(), Self::Error>;
  /// read register reads a value from the specified register address.
  fn read_register(&mut self, address: u16, data: &mut [u8]) -> Result<(), Self::Error>;

  /// write buffer writes data to the specified buffer address.
  fn write_buffer(&mut self, address: u8, data: &[u8]) -> Result<(), Self::Error>;
  /// read buffer reads data from the specified buffer address.
  fn read_buffer(&mut self, address: u8, data: &mut [u8]) -> Result<(), Self::Error>;

  /// get_status reads the current status of the SX1268, returning it or an error.
  fn get_status(&mut self) -> Result<Self::Status, Self::Error>;

  /// reset performs a hardware reset of the SX1268, returning Ok on success or an error.
  fn reset(&mut self) -> Result<(), Self::Error>;

  /// wakeup wakes the SX1268 from sleep mode, returning Ok on success or an error.
  fn wakeup(&mut self) -> Result<(), Self::Error>;

  /// switch tx: switch the SX1268 to transmit mode, returning Ok on success or an error.
  fn switch_tx(&mut self, timeout: u32) -> Result<(), Self::Error>;

  /// switch rx: switch the SX1268 to receive mode, returning Ok on success or an error.
  fn switch_rx(&mut self, timeout: u32) -> Result<(), Self::Error>;
}
