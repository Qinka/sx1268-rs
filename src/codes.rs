//! SX1268 command opcodes as defined in the datasheet.

// Operational Modes
pub const SET_SLEEP: u8 = 0x84;
pub const SET_STANDBY: u8 = 0x80;
pub const SET_FS: u8 = 0xC1;
pub const SET_TX: u8 = 0x83;
pub const SET_RX: u8 = 0x82;
pub const STOP_TIMER_ON_PREAMBLE: u8 = 0x9F;
pub const SET_RX_DUTY_CYCLE: u8 = 0x94;
pub const SET_CAD: u8 = 0xC5;
pub const SET_TX_CONTINUOUS_WAVE: u8 = 0xD1;
pub const SET_TX_INFINITE_PREAMBLE: u8 = 0xD2;
pub const SET_REGULATOR_MODE: u8 = 0x96;
pub const CALIBRATE: u8 = 0x89;
pub const CALIBRATE_IMAGE: u8 = 0x98;
pub const SET_PA_CONFIG: u8 = 0x95;
pub const SET_RX_TX_FALLBACK_MODE: u8 = 0x93;

// Register and Buffer Access
pub const WRITE_REGISTER: u8 = 0x0D;
pub const READ_REGISTER: u8 = 0x1D;
pub const WRITE_BUFFER: u8 = 0x0E;
pub const READ_BUFFER: u8 = 0x1E;

// DIO and IRQ
pub const SET_DIO_IRQ_PARAMS: u8 = 0x08;
pub const GET_IRQ_STATUS: u8 = 0x12;
pub const CLEAR_IRQ_STATUS: u8 = 0x02;
pub const SET_DIO2_AS_RF_SWITCH_CTRL: u8 = 0x9D;
pub const SET_DIO3_AS_TCXO_CTRL: u8 = 0x97;

// RF, Modulation and Packet
pub const SET_RF_FREQUENCY: u8 = 0x86;
pub const SET_PACKET_TYPE: u8 = 0x8A;
pub const GET_PACKET_TYPE: u8 = 0x11;
pub const SET_TX_PARAMS: u8 = 0x8E;
pub const SET_MODULATION_PARAMS: u8 = 0x8B;
pub const SET_PACKET_PARAMS: u8 = 0x8C;
pub const SET_CAD_PARAMS: u8 = 0x88;
pub const SET_BUFFER_BASE_ADDRESS: u8 = 0x8F;
pub const SET_LORA_SYMB_NUM_TIMEOUT: u8 = 0xA0;

// Status
pub const GET_STATUS: u8 = 0xC0;
pub const GET_RSSI_INST: u8 = 0x15;
pub const GET_RX_BUFFER_STATUS: u8 = 0x13;
pub const GET_PACKET_STATUS: u8 = 0x14;
pub const GET_DEVICE_ERRORS: u8 = 0x17;
pub const CLEAR_DEVICE_ERRORS: u8 = 0x07;

// Register Addresses
pub const REG_LORA_SYNC_WORD_MSB: u16 = 0x0740;
pub const REG_LORA_SYNC_WORD_LSB: u16 = 0x0741;
pub const REG_RX_GAIN: u16 = 0x08AC;
pub const REG_TX_CLAMP_CONFIG: u16 = 0x08D8;
pub const REG_OCP_CONFIGURATION: u16 = 0x08E7;
pub const REG_TX_MODULATION: u16 = 0x0889;
pub const REG_IQ_POLARITY: u16 = 0x0736;