// 该文件是 sx1268-rs 项目的一部分。
// src/control.rs - 硬件控制接口
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

//! Hardware abstraction trait for communicating with the SX1268.
//!
//! The [`Control`] trait decouples the driver logic in [`crate::sx1268::Sx1268`]
//! from the concrete hardware layer (SPI bus, chip-select, RESET and BUSY
//! GPIOs, etc.).  Users implement this trait for their particular platform
//! and pass the implementation to [`Sx1268::new`](crate::Sx1268::new).
//!
//! All SPI framing details — e.g. the required NOP byte after the opcode
//! for read commands, or the 3-byte address header for register access —
//! are handled *inside* the `Control` implementation so the upper driver
//! layer can work at a purely logical level.

/// Low-level hardware control interface for the SX1268 transceiver.
///
/// Every method maps to a well-defined SPI transaction as described in
/// the SX1268 datasheet §8.  Implementations must honour the chip's
/// timing constraints (e.g. waiting for BUSY to go low after each
/// command).
pub trait Control {
  /// The device status type returned by read operations.
  type Status;
  /// The error type propagated from the underlying bus.
  type Error;

  /// Send a write-command to the SX1268.
  ///
  /// SPI frame: `[opcode, params…]`.
  /// The chip does not return meaningful data; only errors are reported.
  fn write_command(&mut self, opcode: u8, params: &[u8]) -> Result<(), Self::Error>;

  /// Send a read-command to the SX1268 and receive the response.
  ///
  /// SPI frame: `[opcode, NOP] → [status] → [response…]`.
  /// The first response byte contains the device status.
  fn read_command(&mut self, opcode: u8, params: &mut [u8]) -> Result<Self::Status, Self::Error>;

  /// Write to one or more contiguous internal registers.
  ///
  /// SPI frame: `[WRITE_REGISTER(0x0D), addr_hi, addr_lo, data…]`.
  fn write_register(&mut self, address: u16, data: &[u8]) -> Result<(), Self::Error>;

  /// Read from one or more contiguous internal registers.
  ///
  /// SPI frame: `[READ_REGISTER(0x1D), addr_hi, addr_lo, NOP] → [data…]`.
  fn read_register(&mut self, address: u16, data: &mut [u8]) -> Result<(), Self::Error>;

  /// Write payload data into the TX buffer at `address`.
  ///
  /// SPI frame: `[WRITE_BUFFER(0x0E), offset, data…]`.
  fn write_buffer(&mut self, address: u8, data: &[u8]) -> Result<(), Self::Error>;

  /// Read payload data from the RX buffer starting at `address`.
  ///
  /// SPI frame: `[READ_BUFFER(0x1E), offset, NOP] → [data…]`.
  fn read_buffer(&mut self, address: u8, data: &mut [u8]) -> Result<(), Self::Error>;

  /// Read the device status byte.
  ///
  /// SPI frame: `[GET_STATUS(0xC0)] → [status]`.
  ///
  /// The status byte encodes the current chip mode (bits 6:4) and the
  /// last command status (bits 3:1).
  fn get_status(&mut self) -> Result<Self::Status, Self::Error>;

  /// Perform a hardware reset of the SX1268.
  ///
  /// The implementation should pull the NRESET pin low for ≥ 100 µs
  /// and then wait for the chip to become ready (BUSY low).
  fn reset(&mut self) -> Result<(), Self::Error>;

  /// Wake the SX1268 from sleep mode.
  ///
  /// Typically achieved by driving NSS low momentarily while the chip
  /// is in sleep, which triggers the internal wakeup sequence.
  fn wakeup(&mut self) -> Result<(), Self::Error>;

  /// Switch the radio to TX mode with the given timeout.
  ///
  /// This is a platform-level helper that may include additional steps
  /// such as toggling an external RF switch before calling `SetTx`.
  fn switch_tx(&mut self, timeout: u32) -> Result<(), Self::Error>;

  /// Switch the radio to RX mode with the given timeout.
  ///
  /// This is a platform-level helper that may include additional steps
  /// such as toggling an external RF switch before calling `SetRx`.
  fn switch_rx(&mut self, timeout: u32) -> Result<(), Self::Error>;
}
