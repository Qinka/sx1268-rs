mod modulation;
mod pa_config;
mod package;
mod types;

pub use modulation::{LoRaModulationParams, LoRaBandwidth, LoRaCodingRate, LoRaSpreadingFactor};
pub use pa_config::PaConfig;
pub use package::{LoRaHeaderType, LoRaPacketParams, PacketType};
pub use types::{
  CadExitMode, CadSymbols, CalibrationParams, FallbackMode, RampTime, RegulatorMode, SleepConfig,
  StandbyConfig, TcxoConfig, TcxoVoltage,
};

const FREQ_MIN_HZ: u32 = 410_000_000;
const FREQ_MAX_HZ: u32 = 510_000_000;
const FREQ_DEFAULT_HZ: u32 = 433_000_000;

///
/// Config Error
///
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum ConfigError {
  InvalidFrequency { freq: u32, max: u32, min: u32 },
}

///
/// SX1268 Configuration
///
#[derive(Debug, Clone, defmt::Format)]
pub struct Sx1268Config {
  /// Package type (GFSK or LoRa).
  pub(super) package_type: PacketType,
  /// Frequency in Hz.
  pub(super) frequency_hz: u32,
  /// PaConfig
  pub(super) pa_config: PaConfig,
  /// TX output power in dBm (−9 to +22 for SX1268).
  pub(super) tx_power: i8,
  /// PA ramp time.
  pub(super) ramp_time: RampTime,
  /// LoRa modulation parameters.
  pub(super) lora_modulation: LoRaModulationParams,
  /// LoRa packet parameters.
  pub(super) lora_packet: LoRaPacketParams,
  /// Regulator mode.
  pub(super) regulator_mode: RegulatorMode,
  /// LoRa sync word (0x3444 = public, 0x1424 = private).
  pub(super) lora_sync_word: u16,
  /// TX buffer base address.
  pub(super) tx_base_address: u8,
  /// RX buffer base address.
  pub(super) rx_base_address: u8,
  /// Enable DIO2 as RF switch control.
  pub(super) dio2_as_rf_switch: bool,
  /// Fallback mode after TX/RX.
  pub(super) fallback_mode: FallbackMode,
  /// Optional TCXO configuration on DIO3.
  pub(super) tcxo: Option<TcxoConfig>,
  /// Calibration parameters to run during configuration.
  pub(super) calibration: CalibrationParams,
}

impl Default for Sx1268Config {
  fn default() -> Self {
    Self {
      // Set default configuration values here
      package_type: PacketType::default(),
      frequency_hz: FREQ_DEFAULT_HZ,
      pa_config: PaConfig::default(),
      tx_power: 0,
      ramp_time: RampTime::Ramp10Us,
      lora_modulation: LoRaModulationParams::default(),
      lora_packet: LoRaPacketParams::default(),
      regulator_mode: RegulatorMode::Ldo,
      lora_sync_word: 0x14,
      tx_base_address: 0x00,
      rx_base_address: 0x00,
      dio2_as_rf_switch: false,
      fallback_mode: FallbackMode::StbyRc,
      tcxo: None,
      calibration: CalibrationParams::ALL,
    }
  }
}

impl Sx1268Config {
  /// configure the package type to GFSK.
  pub fn with_package_gfsk(mut self) -> Self {
    self.package_type = PacketType::Gfsk;
    self
  }

  /// configure the package type to LoRA.
  pub fn with_package_lora(mut self) -> Self {
    self.package_type = PacketType::LoRa;
    self
  }

  /// setup frequency in Hz.
  pub fn with_frequency_hz(mut self, freq: u32) -> Result<Self, ConfigError> {
    if !(FREQ_MIN_HZ..=FREQ_MAX_HZ).contains(&freq) {
      Err(ConfigError::InvalidFrequency {
        freq,
        max: FREQ_MAX_HZ,
        min: FREQ_MIN_HZ,
      })
    } else {
      self.frequency_hz = freq;
      Ok(self)
    }
  }

  /// setup PA configuration.
  pub fn with_pa_config(mut self, pa_config: PaConfig) -> Self {
    self.pa_config = pa_config;
    self
  }

  /// setup TX output power in dBm.
  /// 输出功率定义为在以下范围内以 1 dB 为步长，以 dBm 为单位的功率：
  /// - 如果选择低功率 PA，范围为-17 (0xEF)至+14 (0x0E) dBm
  /// - 如果选择高功率 PA，范围为 9 (0xF7)至+22 (0x16) dBm
  ///
  /// 在高压功放和低压功放之间进行选择是通过命令 SetPaConfig 和参数 deviceSel 完成的。
  ///
  /// 默认情况下，低压功放和+14 dBm 被设置。
  pub fn with_tx_power(mut self, tx_power: i8) -> Self {
    self.tx_power = tx_power;
    self
  }

  /// setup PA ramp time.
  /// PA ramp time 定义了 PA 从关闭到完全导通的上升时间。较长的 ramp time 可以减少谐波，但会增加发送前的延迟。
  /// 可用的 ramp time 选项为 10、20、40、80、200、800、1700 和 3400 微秒。
  /// 默认值为 10 微秒。
  pub fn with_ramp_time(mut self, ramp_time: RampTime) -> Self {
    self.ramp_time = ramp_time;
    self
  }

  /// setup LoRa modulation parameters.
  pub fn with_lora_modulation(mut self, modulation: LoRaModulationParams) -> Self {
    self.lora_modulation = modulation;
    self
  }

  /// setup LoRa packet parameters.
  pub fn with_lora_packet(mut self, packet: LoRaPacketParams) -> Self {
    self.lora_packet = packet;
    self
  }

  /// setup regulator mode.
  /// SX1268 支持两种电压调节模式：仅 LDO 和 DC-DC + LDO。选择适当的调节模式可以优化功耗和性能。
  /// - LDO 模式：在这种模式下，所有电压调节都通过线性稳压器（LDO）完成。这种模式通常在较低功耗应用中使用，因为它提供了更干净的电源，但效率较低。
  /// - DC-DC + LDO 模式：在这种模式下，主电压调节通过一个高效的 DC-DC 转换器完成，而 LDO 仅用于为敏感组件提供稳定的电压。这种模式适用于需要更高效率的应用，尤其是在高功率输出设置下。
  ///
  /// 默认情况下，SX1268 配置为 LDO 模式。
  pub fn with_regulator_mode(mut self, mode: RegulatorMode) -> Self {
    self.regulator_mode = mode;
    self
  }

  /// setup LoRa sync word.
  /// LoRa sync word 用于区分不同的 LoRa 网络。公共 sync word（0x3444）用于公共 LoRa 网络，而私有 sync word（0x1424）用于私有网络。选择正确的 sync word 可以确保设备只与预期的网络通信。
  pub fn with_lora_sync_word(mut self, sync_word: u16) -> Self {
    self.lora_sync_word = sync_word;
    self
  }

  pub fn with_lora_sync_word_public(self) -> Self {
    self.with_lora_sync_word(0x3444)
  }
  pub fn with_lora_sync_word_private(self) -> Self {
    self.with_lora_sync_word(0x1424)
  }

  /// setup TX buffer base address.
  /// TX buffer base address 定义了用于存储要发送的数据的内部缓冲区的起始地址。正确设置这个地址对于确保数据正确加载到发送缓冲区并成功发送至关重要。
  pub fn with_tx_base_address(mut self, address: u8) -> Self {
    self.tx_base_address = address;
    self
  }

  /// setup RX buffer base address.
  /// RX buffer base address 定义了用于存储接收数据的内部缓冲区的起始地址。正确设置这个地址对于确保接收的数据正确存储并可以被处理器访问至关重要。
  pub fn with_rx_base_address(mut self, address: u8) -> Self {
    self.rx_base_address = address;
    self
  }

  /// setup DIO2 as RF switch control.
  /// 启用 DIO2 作为 RF 开关控制允许设备使用 DIO2 引脚来控制外部 RF 开关。这对于使用外部天线切换或需要在发送和接收之间切换的应用非常有用。启用此功能可以简化硬件设计并提高系统的灵活性。
  pub fn with_dio2_as_rf_switch(mut self) -> Self {
    self.dio2_as_rf_switch = true;
    self
  }

  /// setup fallback mode after TX/RX.
  /// Fallback mode 定义了设备在完成发送或接收操作后进入的状态。选择适当的 fallback mode 可以优化功耗和响应时间。
  /// - StbyRc：设备进入使用内部 RC 振荡器的待机模式。这种模式下设备的功耗较低，但唤醒时间较长。
  /// - StbyXosc：设备进入使用外部晶振的待机模式。这种模式下设备的功耗较低，唤醒时间较短，适用于需要快速响应的应用。
  /// - Fs：设备进入频率合成器模式，准备进行发送或接收操作。这种模式下设备的功耗较高，但可以实现最快的响应时间，适用于需要立即发送或接收数据的应用。
  pub fn with_fallback_mode(mut self, mode: FallbackMode) -> Self {
    self.fallback_mode = mode;
    self
  }

  /// setup optional TCXO configuration on DIO3.
  /// 如果使用外部 TCXO 作为时钟源，必须提供 TCXO 配置，包括 TCXO 的电压设置和稳定时间。正确配置 TCXO 可以确保设备在发送和接收操作中具有准确的时钟，从而提高通信的可靠性和性能。
  pub fn with_tcxo_config(mut self, voltage: TcxoVoltage, timeout: u32) -> Self {
    self.tcxo = Some(TcxoConfig { voltage, timeout });
    self
  }

  /// setup calibration parameters to run during configuration.
  /// 校准参数定义了在配置过程中要运行的校准程序。正确设置这些参数可以确保设备在各种环境条件下都能保持最佳性能。
  pub fn with_calibration(mut self, calibration: CalibrationParams) -> Self {
    self.calibration = calibration;
    self
  }

  pub fn get_frequency_hz(&self) -> u32 {
    self.frequency_hz
  }

  pub fn get_frequency_mhz(&self) -> f32 {
    self.frequency_hz as f32 / 1_000_000.0
  }

  pub fn get_power_dbm(&self) -> i8 {
    self.tx_power
  }

  pub fn get_bandwidth_khz(&self) -> u32 {
    self.lora_modulation.bw.as_khz()
  }

  pub fn get_sf(&self) -> u8 {
    self.lora_modulation.sf.as_sf()
  }

  pub fn get_cr_ratio(&self) -> (u8, u8) {
    self.lora_modulation.cr.as_cr()
  }

  pub fn get_preamble_length(&self) -> u16 {
    self.lora_packet.preamble_length
  }

  pub fn get_crc_enabled(&self) -> bool {
    self.lora_packet.crc_on
  }

  pub fn get_header_type(&self) -> LoRaHeaderType {
    self.lora_packet.header_type
  }

  pub fn get_sync_word(&self) -> u16 {
    self.lora_sync_word
  }

  pub fn get_pa_duty_cycle(&self) -> u8 {
    self.pa_config.pa_duty_cycle
  }

  pub fn get_pa_hp_max(&self) -> u8 {
    self.pa_config.hp_max
  }

}
