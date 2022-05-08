#include "ADS131M04.h"

#include "Arduino.h"
#include "SPI.h"

#define settings SPISettings( 4000000, MSBFIRST, SPI_MODE1 )

ADS131M04::ADS131M04( uint8_t       cs_pin,
                      uint32_t      freq,
                      BusIOBitOrder dataOrder,
                      uint8_t       dataMode,
                      SPIClass*     spi )
    : spi_dev( cs_pin, freq, dataOrder, dataMode, spi ) {}

void
ADS131M04::reset() {
  constexpr uint8_t size         = ( 1 + 4 + 1 ) * 3 * 1;
  uint8_t           buffer[size] = { 0 };

  {
    uint16_t cmd = CMD_RESET;
    buffer[0]    = cmd >> 8;
    buffer[1]    = cmd & 0x00ff;
  }

  spi_dev.write_and_read( buffer, size );
}

uint16_t
ADS131M04::readRegister( uint8_t address ) {
  constexpr uint8_t size         = ( 1 + 4 + 1 ) * 3 * 2;
  uint8_t           buffer[size] = { 0 };

  {
    uint16_t cmd = CMD_READ_REG | ( address << 7 );
    buffer[0]    = cmd >> 8;
    buffer[1]    = cmd & 0x00ff;
  }

  spi_dev.write_and_read( buffer, size );

  return ( buffer[6 * 3 + 0] << 8 ) | ( buffer[6 * 3 + 1] );
}

uint8_t
ADS131M04::writeRegister( uint8_t address, uint16_t value ) {
  constexpr uint8_t size         = ( 1 + 4 + 1 ) * 3 * 2;
  uint8_t           buffer[size] = { 0 };

  {
    uint16_t cmd      = CMD_WRITE_REG | ( address << 7 );
    buffer[0 * 3 + 0] = cmd >> 8;
    buffer[0 * 3 + 1] = cmd & 0x00ff;

    buffer[1 * 3 + 0] = value >> 8;
    buffer[1 * 3 + 1] = value & 0x00ff;
  }

  spi_dev.write_and_read( buffer, size );

  uint16_t res = buffer[6 * 3 + 0] << 8 | buffer[6 * 3 + 1];

  uint8_t addressRcv = ( res & REGMASK_CMD_READ_REG_ADDRESS ) >> 7;
  uint8_t bytesRcv   = ( res & REGMASK_CMD_READ_REG_BYTES );

  if( addressRcv == address ) {
    return bytesRcv + 1;
  }
  return 0;
}

adcOutput
ADS131M04::readADC( void ) {
  constexpr uint8_t size         = ( 1 + 4 + 1 ) * 3 * 1;
  uint8_t           buffer[size] = { 0 };

  spi_dev.write_and_read( buffer, size );

  {
    adcOutput res;

    res.status = ( ( buffer[0] << 8 ) | buffer[1] );
    for( const int i : { 0, 1, 2, 3 } ) {
      res.ch[i] = convertAdcValue( buffer + ( i + 1 ) * 3 );
    }

    return res;
  }
}

void
ADS131M04::writeRegisterMasked( uint8_t address, uint16_t value, uint16_t mask ) {
  uint16_t register_contents = readRegister( address );

  register_contents = ( register_contents & ~mask ) | value;

  writeRegister( address, register_contents );
}

void
ADS131M04::begin() {
  spi_dev.begin();
}

int8_t
ADS131M04::isDataReadySoft( byte channel ) {
  if( channel == 0 ) {
    return ( readRegister( REG_STATUS ) & REGMASK_STATUS_DRDY0 );
  } else if( channel == 1 ) {
    return ( readRegister( REG_STATUS ) & REGMASK_STATUS_DRDY1 );
  } else if( channel == 2 ) {
    return ( readRegister( REG_STATUS ) & REGMASK_STATUS_DRDY2 );
  } else if( channel == 3 ) {
    return ( readRegister( REG_STATUS ) & REGMASK_STATUS_DRDY3 );
  } else {
    return -1;
  }
}

bool
ADS131M04::isResetStatus( void ) {
  return ( readRegister( REG_STATUS ) & REGMASK_STATUS_RESET );
}

bool
ADS131M04::isLockSPI( void ) {
  return ( readRegister( REG_STATUS ) & REGMASK_STATUS_LOCK );
}

bool
ADS131M04::setDrdyFormat( uint8_t drdyFormat ) {
  if( drdyFormat > 1 ) {
    return false;
  } else {
    writeRegisterMasked( REG_MODE, drdyFormat, REGMASK_MODE_DRDY_FMT );
    return true;
  }
}

bool
ADS131M04::setDrdyStateWhenUnavailable( uint8_t drdyState ) {
  if( drdyState > 1 ) {
    return false;
  } else {
    writeRegisterMasked( REG_MODE, drdyState < 1, REGMASK_MODE_DRDY_HiZ );
    return true;
  }
}

bool
ADS131M04::setPowerMode( uint8_t powerMode ) {
  if( powerMode > 3 ) {
    return false;
  } else {
    writeRegisterMasked( REG_CLOCK, powerMode, REGMASK_CLOCK_PWR );
    return true;
  }
}

bool
ADS131M04::setOsr( uint16_t osr ) {
  if( osr > 7 ) {
    return false;
  } else {
    writeRegisterMasked( REG_CLOCK, osr << 2, REGMASK_CLOCK_OSR );
    return true;
  }
}

bool
ADS131M04::setChannelEnable( uint8_t channel, uint16_t enable ) {
  if( channel == 0 ) {
    writeRegisterMasked( REG_CLOCK, enable << 8, REGMASK_CLOCK_CH0_EN );
    return true;
  } else if( channel == 1 ) {
    writeRegisterMasked( REG_CLOCK, enable << 9, REGMASK_CLOCK_CH1_EN );
    return true;
  } else if( channel == 2 ) {
    writeRegisterMasked( REG_CLOCK, enable << 10, REGMASK_CLOCK_CH2_EN );
    return true;
  } else if( channel == 3 ) {
    writeRegisterMasked( REG_CLOCK, enable << 11, REGMASK_CLOCK_CH3_EN );
    return true;
  }

  return false;
}

bool
ADS131M04::setChannelPGA( uint8_t channel, uint16_t pga ) {
  if( channel == 0 ) {
    writeRegisterMasked( REG_GAIN, pga, REGMASK_GAIN_PGAGAIN0 );
    return true;
  } else if( channel == 1 ) {
    writeRegisterMasked( REG_GAIN, pga << 4, REGMASK_GAIN_PGAGAIN1 );
    return true;
  } else if( channel == 2 ) {
    writeRegisterMasked( REG_GAIN, pga << 8, REGMASK_GAIN_PGAGAIN2 );
    return true;
  } else if( channel == 3 ) {
    writeRegisterMasked( REG_GAIN, pga << 12, REGMASK_GAIN_PGAGAIN3 );
    return true;
  }
  return false;
}

void
ADS131M04::setGlobalChop( uint16_t global_chop ) {
  writeRegisterMasked( REG_CFG, global_chop << 8, REGMASK_CFG_GC_EN );
}

void
ADS131M04::setGlobalChopDelay( uint16_t delay ) {
  writeRegisterMasked( REG_CFG, delay << 9, REGMASK_CFG_GC_DLY );
}

bool
ADS131M04::setInputChannelSelection( uint8_t channel, uint8_t input ) {
  if( channel == 0 ) {
    writeRegisterMasked( REG_CH0_CFG, input, REGMASK_CHX_CFG_MUX );
    return true;
  } else if( channel == 1 ) {
    writeRegisterMasked( REG_CH1_CFG, input, REGMASK_CHX_CFG_MUX );
    return true;
  } else if( channel == 2 ) {
    writeRegisterMasked( REG_CH2_CFG, input, REGMASK_CHX_CFG_MUX );
    return true;
  } else if( channel == 3 ) {
    writeRegisterMasked( REG_CH3_CFG, input, REGMASK_CHX_CFG_MUX );
    return true;
  }
  return false;
}

bool
ADS131M04::setChannelOffsetCalibration( uint8_t channel, int32_t offset ) {
  uint16_t MSB = offset >> 8;
  uint8_t  LSB = offset;

  if( channel == 0 ) {
    writeRegisterMasked( REG_CH0_OCAL_MSB, MSB, 0xFFFF );
    writeRegisterMasked( REG_CH0_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB );
    return true;
  } else if( channel == 1 ) {
    writeRegisterMasked( REG_CH1_OCAL_MSB, MSB, 0xFFFF );
    writeRegisterMasked( REG_CH1_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB );
    return true;
  } else if( channel == 2 ) {
    writeRegisterMasked( REG_CH2_OCAL_MSB, MSB, 0xFFFF );
    writeRegisterMasked( REG_CH2_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB );
    return true;
  } else if( channel == 3 ) {
    writeRegisterMasked( REG_CH3_OCAL_MSB, MSB, 0xFFFF );
    writeRegisterMasked( REG_CH3_OCAL_LSB, LSB << 8, REGMASK_CHX_OCAL0_LSB );
    return true;
  }
  return false;
}

bool
ADS131M04::setChannelGainCalibration( uint8_t channel, uint32_t gain ) {
  uint16_t MSB = gain >> 8;
  uint8_t  LSB = gain;

  if( channel == 0 ) {
    writeRegisterMasked( REG_CH0_GCAL_MSB, MSB, 0xFFFF );
    writeRegisterMasked( REG_CH0_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB );
    return true;
  } else if( channel == 1 ) {
    writeRegisterMasked( REG_CH1_GCAL_MSB, MSB, 0xFFFF );
    writeRegisterMasked( REG_CH1_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB );
    return true;
  } else if( channel == 2 ) {
    writeRegisterMasked( REG_CH2_GCAL_MSB, MSB, 0xFFFF );
    writeRegisterMasked( REG_CH2_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB );
    return true;
  } else if( channel == 3 ) {
    writeRegisterMasked( REG_CH3_GCAL_MSB, MSB, 0xFFFF );
    writeRegisterMasked( REG_CH3_GCAL_LSB, LSB << 8, REGMASK_CHX_GCAL0_LSB );
    return true;
  }
  return false;
}

int32_t
ADS131M04::convertAdcValue( uint8_t* value ) {
  int32_t result = *( value ) << 16 | *( value + 1 ) << 8 | *( value + 2 );

  if( result > 0x7FFFFF ) {
    result = ( ( ~( result )&0x00FFFFFF ) + 1 ) * -1;
  }

  return result;
}
