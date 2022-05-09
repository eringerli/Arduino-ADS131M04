#ifndef ADS131M04_h
#define ADS131M04_h

#include "Arduino.h"
#include "SPI.h"

#include <Adafruit_SPIDevice.h>

struct adcOutput {
  uint16_t status;
  int32_t  ch[4];
};

constexpr uint8_t DRDY_STATE_LOGIC_HIGH = 0; // DEFAULS
constexpr uint8_t DRDY_STATE_HI_Z       = 1;

constexpr uint8_t POWER_MODE_VERY_LOW_POWER  = 0;
constexpr uint8_t POWER_MODE_LOW_POWER       = 1;
constexpr uint8_t POWER_MODE_HIGH_RESOLUTION = 2; // DEFAULT

constexpr uint8_t CHANNEL_PGA_1   = 0;
constexpr uint8_t CHANNEL_PGA_2   = 1;
constexpr uint8_t CHANNEL_PGA_4   = 2;
constexpr uint8_t CHANNEL_PGA_8   = 3;
constexpr uint8_t CHANNEL_PGA_16  = 4;
constexpr uint8_t CHANNEL_PGA_32  = 5;
constexpr uint8_t CHANNEL_PGA_64  = 6;
constexpr uint8_t CHANNEL_PGA_128 = 7;

constexpr uint8_t INPUT_CHANNEL_MUX_AIN0P_AIN0N             = 0; // Default
constexpr uint8_t INPUT_CHANNEL_MUX_INPUT_SHORTED           = 1;
constexpr uint8_t INPUT_CHANNEL_MUX_POSITIVE_DC_TEST_SIGNAL = 2;
constexpr uint8_t INPUT_CHANNEL_MUX_NEGATIVE_DC_TEST_SIGNAL = 3;

constexpr uint8_t OSR_128   = 0;
constexpr uint8_t OSR_256   = 1;
constexpr uint8_t OSR_512   = 2;
constexpr uint8_t OSR_1024  = 3; // default
constexpr uint8_t OSR_2018  = 4;
constexpr uint8_t OSR_4096  = 5;
constexpr uint8_t OSR_8192  = 6;
constexpr uint8_t OSR_16384 = 7;

// Commands
constexpr uint16_t CMD_NULL    = 0x0000; // This command gives the STATUS REGISTER
constexpr uint16_t CMD_RESET   = 0x0011;
constexpr uint16_t CMD_STANDBY = 0x0022;
constexpr uint16_t CMD_WAKEUP  = 0x0033;
constexpr uint16_t CMD_LOCK    = 0x0555;
constexpr uint16_t CMD_UNLOCK  = 0x0655;
constexpr uint16_t CMD_READ_REG =
  0xA000; // 101a aaaa annn nnnn   a=adress  n=numero de registros-1
constexpr uint16_t CMD_WRITE_REG = 0x6000;

// Responses
constexpr uint16_t RSP_RESET_OK  = 0xFF24;
constexpr uint16_t RSP_RESET_NOK = 0x0011;

// Registers Read Only
constexpr uint8_t REG_ID     = 0x00;
constexpr uint8_t REG_STATUS = 0x01;

// Registers Global Settings across channels
constexpr uint8_t REG_MODE    = 0x02;
constexpr uint8_t REG_CLOCK   = 0x03;
constexpr uint8_t REG_GAIN    = 0x04;
constexpr uint8_t REG_CFG     = 0x06;
constexpr uint8_t THRSHLD_MSB = 0x07;
constexpr uint8_t THRSHLD_LSB = 0x08;

// Registers Channel 0 Specific
constexpr uint8_t REG_CH0_CFG      = 0x09;
constexpr uint8_t REG_CH0_OCAL_MSB = 0x0A;
constexpr uint8_t REG_CH0_OCAL_LSB = 0x0B;
constexpr uint8_t REG_CH0_GCAL_MSB = 0x0C;
constexpr uint8_t REG_CH0_GCAL_LSB = 0x0D;

// Registers Channel 1 Specific
constexpr uint8_t REG_CH1_CFG      = 0x0E;
constexpr uint8_t REG_CH1_OCAL_MSB = 0x0F;
constexpr uint8_t REG_CH1_OCAL_LSB = 0x10;
constexpr uint8_t REG_CH1_GCAL_MSB = 0x11;
constexpr uint8_t REG_CH1_GCAL_LSB = 0x12;

// Registers Channel 2 Specific
constexpr uint8_t REG_CH2_CFG      = 0x13;
constexpr uint8_t REG_CH2_OCAL_MSB = 0x14;
constexpr uint8_t REG_CH2_OCAL_LSB = 0x15;
constexpr uint8_t REG_CH2_GCAL_MSB = 0x16;
constexpr uint8_t REG_CH2_GCAL_LSB = 0x17;

// Registers Channel 3 Specific
constexpr uint8_t REG_CH3_CFG      = 0x18;
constexpr uint8_t REG_CH3_OCAL_MSB = 0x19;
constexpr uint8_t REG_CH3_OCAL_LSB = 0x1A;
constexpr uint8_t REG_CH3_GCAL_MSB = 0x1B;
constexpr uint8_t REG_CH3_GCAL_LSB = 0x1C;

// Registers MAP CRC
constexpr uint8_t REG_MAP_CRC = 0x3E;

// ------------------------------------------------------------------------------------

// Mask READ_REG
constexpr uint16_t REGMASK_CMD_READ_REG_ADDRESS = 0x1F80;
constexpr uint16_t REGMASK_CMD_READ_REG_BYTES   = 0x007F;

// Mask Register STATUS
constexpr uint16_t REGMASK_STATUS_LOCK     = 0x8000;
constexpr uint16_t REGMASK_STATUS_RESYNC   = 0x4000;
constexpr uint16_t REGMASK_STATUS_REGMAP   = 0x2000;
constexpr uint16_t REGMASK_STATUS_CRC_ERR  = 0x1000;
constexpr uint16_t REGMASK_STATUS_CRC_TYPE = 0x0800;
constexpr uint16_t REGMASK_STATUS_RESET    = 0x0400;
constexpr uint16_t REGMASK_STATUS_WLENGTH  = 0x0300;
constexpr uint16_t REGMASK_STATUS_DRDY3    = 0x0008;
constexpr uint16_t REGMASK_STATUS_DRDY2    = 0x0004;
constexpr uint16_t REGMASK_STATUS_DRDY1    = 0x0002;
constexpr uint16_t REGMASK_STATUS_DRDY0    = 0x0001;

// Mask Register MODE
constexpr uint16_t REGMASK_MODE_REG_CRC_EN = 0x2000;
constexpr uint16_t REGMASK_MODE_RX_CRC_EN  = 0x1000;
constexpr uint16_t REGMASK_MODE_CRC_TYPE   = 0x0800;
constexpr uint16_t REGMASK_MODE_RESET      = 0x0400;
constexpr uint16_t REGMASK_MODE_WLENGTH    = 0x0300;
constexpr uint16_t REGMASK_MODE_TIMEOUT    = 0x0010;
constexpr uint16_t REGMASK_MODE_DRDY_SEL   = 0x000C;
constexpr uint16_t REGMASK_MODE_DRDY_HiZ   = 0x0002;
constexpr uint16_t REGMASK_MODE_DRDY_FMT   = 0x0001;

// Mask Register CLOCK
constexpr uint16_t REGMASK_CLOCK_CH3_EN = 0x0800;
constexpr uint16_t REGMASK_CLOCK_CH2_EN = 0x0400;
constexpr uint16_t REGMASK_CLOCK_CH1_EN = 0x0200;
constexpr uint16_t REGMASK_CLOCK_CH0_EN = 0x0100;
constexpr uint16_t REGMASK_CLOCK_OSR    = 0x001C;
constexpr uint16_t REGMASK_CLOCK_PWR    = 0x0003;

// Mask Register GAIN
constexpr uint16_t REGMASK_GAIN_PGAGAIN3 = 0x7000;
constexpr uint16_t REGMASK_GAIN_PGAGAIN2 = 0x0700;
constexpr uint16_t REGMASK_GAIN_PGAGAIN1 = 0x0070;
constexpr uint16_t REGMASK_GAIN_PGAGAIN0 = 0x0007;

// Mask Register CFG
constexpr uint16_t REGMASK_CFG_GC_DLY   = 0x1E00;
constexpr uint16_t REGMASK_CFG_GC_EN    = 0x0100;
constexpr uint16_t REGMASK_CFG_CD_ALLCH = 0x0080;
constexpr uint16_t REGMASK_CFG_CD_NUM   = 0x0070;
constexpr uint16_t REGMASK_CFG_CD_LEN   = 0x000E;
constexpr uint16_t REGMASK_CFG_CD_EN    = 0x0001;

// Mask Register THRSHLD_LSB
constexpr uint16_t REGMASK_THRSHLD_LSB_CD_TH_LSB = 0xFF00;
constexpr uint16_t REGMASK_THRSHLD_LSB_DCBLOCK   = 0x000F;

// Mask Register CHX_CFG
constexpr uint16_t REGMASK_CHX_CFG_PHASE       = 0xFFC0;
constexpr uint16_t REGMASK_CHX_CFG_DCBLKX_DIS0 = 0x0004;
constexpr uint16_t REGMASK_CHX_CFG_MUX         = 0x0003;

// Mask Register CHX_OCAL_LSB
constexpr uint16_t REGMASK_CHX_OCAL0_LSB = 0xFF00;

// Mask Register CHX_GCAL_LSB
constexpr uint16_t REGMASK_CHX_GCAL0_LSB = 0xFF00;

//   --------------------------------------------------------------------

// Conversion modes
constexpr uint8_t CONVERSION_MODE_CONT   = 0;
constexpr uint8_t CONVERSION_MODE_SINGLE = 1;

// Data Format
constexpr uint8_t DATA_FORMAT_TWO_COMPLEMENT = 0;
constexpr uint8_t DATA_FORMAT_BINARY         = 1;

// Measure Mode
constexpr uint8_t MEASURE_UNIPOLAR = 1;
constexpr uint8_t MEASURE_BIPOLAR  = 0;

// Clock Type
constexpr uint8_t CLOCK_EXTERNAL = 1;
constexpr uint8_t CLOCK_INTERNAL = 0;

// PGA Gain
constexpr uint8_t PGA_GAIN_1   = 0;
constexpr uint8_t PGA_GAIN_2   = 1;
constexpr uint8_t PGA_GAIN_4   = 2;
constexpr uint8_t PGA_GAIN_8   = 3;
constexpr uint8_t PGA_GAIN_16  = 4;
constexpr uint8_t PGA_GAIN_32  = 5;
constexpr uint8_t PGA_GAIN_64  = 6;
constexpr uint8_t PGA_GAIN_128 = 7;

// Input Filter
constexpr uint8_t FILTER_SINC    = 0;
constexpr uint8_t FILTER_FIR     = 2;
constexpr uint8_t FILTER_FIR_IIR = 3;

// Data Mode
constexpr uint8_t DATA_MODE_24BITS = 0;
constexpr uint8_t DATA_MODE_32BITS = 1;

// Data Rate
constexpr uint8_t DATA_RATE_0  = 0;
constexpr uint8_t DATA_RATE_1  = 1;
constexpr uint8_t DATA_RATE_2  = 2;
constexpr uint8_t DATA_RATE_3  = 3;
constexpr uint8_t DATA_RATE_4  = 4;
constexpr uint8_t DATA_RATE_5  = 5;
constexpr uint8_t DATA_RATE_6  = 6;
constexpr uint8_t DATA_RATE_7  = 7;
constexpr uint8_t DATA_RATE_8  = 8;
constexpr uint8_t DATA_RATE_9  = 9;
constexpr uint8_t DATA_RATE_10 = 10;
constexpr uint8_t DATA_RATE_11 = 11;
constexpr uint8_t DATA_RATE_12 = 12;
constexpr uint8_t DATA_RATE_13 = 13;
constexpr uint8_t DATA_RATE_14 = 14;
constexpr uint8_t DATA_RATE_15 = 15;

// Sync Mpdes
constexpr uint8_t SYNC_CONTINUOUS = 1;
constexpr uint8_t SYNC_PULSE      = 0;

// DIO Config Mode
constexpr uint8_t DIO_OUTPUT = 1;
constexpr uint8_t DIO_INPUT  = 0;

constexpr uint8_t  SPI_MASTER_DUMMY   = 0xFF;
constexpr uint16_t SPI_MASTER_DUMMY16 = 0xFFFF;
constexpr uint32_t SPI_MASTER_DUMMY32 = 0xFFFFFFFF;

class ADS131M04 {
public:
  ADS131M04( uint8_t       cspin,
             uint32_t      freq      = 1000000,
             BusIOBitOrder dataOrder = SPI_BITORDER_MSBFIRST,
             uint8_t       dataMode  = SPI_MODE1,
             SPIClass*     spi       = &SPI );

  void      begin();
  void      reset();
  int8_t    isDataReadySoft( byte channel );
  bool      isResetStatus( void );
  bool      isLockSPI( void );
  bool      setDrdyFormat( uint8_t drdyFormat );
  bool      setDrdyStateWhenUnavailable( uint8_t drdyState );
  bool      setPowerMode( uint8_t powerMode );
  bool      setChannelEnable( uint8_t channel, uint16_t enable );
  bool      setChannelPGA( uint8_t channel, uint16_t pga );
  void      setGlobalChop( uint16_t global_chop );
  void      setGlobalChopDelay( uint16_t delay );
  bool      setInputChannelSelection( uint8_t channel, uint8_t input );
  bool      setChannelOffsetCalibration( uint8_t channel, int32_t offset );
  bool      setChannelGainCalibration( uint8_t channel, uint32_t gain );
  bool      setOsr( uint16_t osr );
  adcOutput readADC( void );

private:
  uint8_t  writeRegister( uint8_t address, uint16_t value );
  void     writeRegisterMasked( uint8_t address, uint16_t value, uint16_t mask );
  uint16_t readRegister( uint8_t address );

  int32_t convertAdcValue( uint8_t* value );

  Adafruit_SPIDevice spi_dev;
};
#endif
