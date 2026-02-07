//! Error types for the SX1268 driver.

/// Driver error type wrapping SPI errors.
#[derive(Debug, defmt::Format)]
pub enum Error<SPI> {
    /// SPI bus error.
    Spi(SPI),
    /// Invalid or unexpected device status.
    InvalidStatus,
    /// Operation timed out.
    Timeout,
    /// Invalid parameter.
    InvalidParameter,
}

impl<SPI> From<SPI> for Error<SPI> {
    fn from(e: SPI) -> Self {
        Error::Spi(e)
    }
}
