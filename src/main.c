/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <string.h>
#include <stdio.h>
#include <math.h>

#include <ch.h>
#include <hal.h>

#define MS5637_I2C_ADDR 	(0x76)
#define MS56_CMD_RST		(0x1E)
#define MS56_CMD_PROM_READ	(0xA0)
#define MS56_CMD_CONV_P_256	(0x40)
#define MS56_CMD_CONV_T_256	(0x50)
#define MS56_CMD_ADC_READ	(0x00)
#define MS56_PROM_SIZE		(7)

// Oversample rates
#define MS5637_OSR_256  0x00
#define MS5637_OSR_512  0x02
#define MS5637_OSR_1024 0x04
#define MS5637_OSR_2048 0x06
#define MS5637_OSR_4096 0x08

#define SEA_LEVEL_PRESSURE 1013.25f

#define MS2ST(msec)                                                         \
  ((systime_t)(((((uint32_t)(msec)) *                                       \
                 ((uint32_t)CH_CFG_ST_FREQUENCY) - 1UL) / 1000UL) + 1UL))

#define SEND_MSG(ser_p, msg) send_cfg((ser_p), (msg), sizeof((msg)));

typedef struct __attribute__((packed)) {
  uint8_t sync1;
  uint8_t sync2;
  uint8_t class;
  uint8_t id;
  uint16_t length;
  uint16_t status;
  float pressure;
  float temperature;
  uint8_t checksum_a;
  uint8_t checksum_b;
} ubx_msg_pressure;

/*****************************************************************************
 * Global variables
 *****************************************************************************/

static I2CConfig i2cfg1 =
  {
   STM32_TIMINGR_PRESC(5U)  | /* 48MHz/6 = 8MHz I2CCLK.           */
   STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
   STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U),
   // Just scale bits 28..31 if not 8MHz clock
   0,              // CR1
   0,              // CR2
  };

SerialConfig uart1_config;
SerialConfig uart2_config;
uint8_t found_valid_message;
ubx_msg_pressure baro_msg;
uint8_t out_buffer[40];
uint16_t MS56_PROMData[MS56_PROM_SIZE];
// Baud 57600
static const uint8_t ubx_57kbps[28] =
  {
   0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00,
   0x00, 0xE1, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE, 0xC9
  };
static const uint8_t nmea_gga_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F
  };
static const uint8_t nmea_gll_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11
  };
static const uint8_t nmea_gsa_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13
  };
static const uint8_t nmea_gsv_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15
  };
static const uint8_t nmea_rmc_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17
  };
static const uint8_t nmea_vtg_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19
  };
static const uint8_t ubx_posllh_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47
  };
static const uint8_t ubx_status_msg[] = 
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49
  };
static const uint8_t ubx_sol_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F
  };
static const uint8_t ubx_velned_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67
  };
static const uint8_t ubx_timeutc_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x21, 0x01, 0x2D, 0x85
  };
static const uint8_t ubx_pvt_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51
  };
static const uint8_t ubx_svinfo_msg[] =
  {
   0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x01, 0x3C, 0xA3
  };
static const uint8_t ubx_rate_200ms[] =
  {
   0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
  };
static const uint8_t ubx_cfg_gnns[] =
  {
   0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x20, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00,
   0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00, 0x01, 0x00,
   0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00,
   0x01, 0x03, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x05, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00,
   0x01, 0x01, 0x55, 0x47
  };

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
// The length of the string is returned, and it is not null terminated.
uint8_t itoa(int32_t x, uint8_t *str, uint8_t n, uint8_t d) {
  uint8_t len = 0;
  uint8_t i;
  uint8_t j;

  // Don't create an empty string
  if (n == 0) {
    return 0;
  }

  // Insert the negative sign if necessary.
  if (x < 0) {
    x = -x;
    str[0] = '-';
    ++str;
    --n;
    len = 1;
  }

  // The number length can't be longer than the buffer.
  if (d > n) {
    d = n;
  }

  // Create the string backward in the buffer
  for (i = 0; (i < n) && x; ++i, x = x / 10) {
    str[i] = x % 10;
  }

  // Pad with zeros if requested.
  for ( ; i < d; ++i) {
    str[i] = 0;
  }

  // Add the current number of digets to the length;
  len += i;

  // Reverse the string and convert to ASCII
  for (j = 0; i > j; ++j, --i) {
    uint8_t tmp = str[i - 1];
    str[i - 1] = '0' + str[j];
    str[j] = '0' + tmp;
  }

  // Return the length of the string created.
  return len;
}

// Converts a floating point number to string.
uint8_t ftoa(float x, uint8_t *str, uint8_t n, uint8_t afterpoint) {
  uint8_t len = 0;
  uint8_t i;
  uint8_t j;

  // Don't create an empty string
  if (n == 0) {
    return 0;
  }

  // Insert the negative sign if necessary.
  if (x < 0) {
    x = -x;
    str[0] = '-';
    ++str;
    --n;
    len = 1;
  }

  // Append the "0." if the number is < 1.0.
  if (x < 1.0) {
    if (n < 2) {
      return 0;
    }
    str[0] = '0';
    str[1] = '.';
    str += 2;
    n -= 2;
    len += 2;
  }

  // Turn the floating point value into fixed point.
  for (i = 0; i < afterpoint; ++i) {
    x *= 10.0;
  }

  // Create the string backward in the buffer
  for (i = 0; (i < n) && (x > 1); ++i) {
    // Add the decimal point when necessary.
    if ((afterpoint != 0) && (i == afterpoint)) {
      str[i] = '.';
      continue;
    }
    str[i] = '0' + ((uint32_t)(x) % 10);
    x = x / 10;
  }

  // Add the current number of digets to the length;
  len += i;

  // Reverse the string;
  for (j = 0; i > j; ++j, --i) {
    uint8_t tmp = str[i - 1];
    str[i - 1] = str[j];
    str[j] = tmp;
  }

  // Return the length of the string created.
  return len;
}

void ubx_checksum(uint8_t *msg) {
  uint16_t length = ((uint16_t*)msg)[2];
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  uint16_t i;
  for (i = 2; i < length + 6; ++i) {
    ck_a = ck_a + msg[i];
    ck_b = ck_b + ck_a;
  }
  msg[length + 6] = ck_a;
  msg[length + 7] = ck_b;
}

int nmea_checksum(const char *s) {
  int c = 0;
  while(*s)
    c ^= *s++;
  return c;
}

void send_cfg(SerialDriver *sd, const uint8_t *msg, uint8_t len) {
  sdWrite(sd, msg, len);
  chThdSleepMilliseconds(100);
}

int8_t MS5637_Setup(I2CDriver *i2cp) {
  msg_t status;
  uint8_t data[2];
  uint8_t i;

  // sensor needs at most 15ms after power-up
  chThdSleepMilliseconds(100);

  /* Send reset sequence */
  data[0] = 0x1E;
  status = i2cMasterTransmit(i2cp, MS5637_I2C_ADDR, data, 1, 0, 0);
  if (status != MSG_OK) {
    return -2;
  }
  chThdSleepMilliseconds(100);

  /* Read PROM containing calibration data */
  for (i=0; i<MS56_PROM_SIZE; i++) {

    /* Send PROM read command */
    data[0] = MS56_CMD_PROM_READ + (i<<1);
    status = i2cMasterTransmit(i2cp, MS5637_I2C_ADDR, data, 1, 0, 0);
    if (status != MSG_OK) {
      return -4;
    }

    /* Read 16-bit data */
    status = i2cMasterReceive(i2cp, MS5637_I2C_ADDR, data, 2);
    if (status != MSG_OK) {
      return -8;
    }
    MS56_PROMData[i] = ((uint16_t)data[0] << 8) + data[1];
  }

  return 0;
}

int8_t MS5637_ReadData(I2CDriver *i2cp, ubx_msg_pressure *msg) {
  msg_t status;
  uint8_t data[3];

  // Send temperature conversion command
  data[0] = MS56_CMD_CONV_T_256 + 6;
  status = i2cMasterTransmit(i2cp, MS5637_I2C_ADDR, data, 1, 0, 0);
  if (status != MSG_OK) {
    return -1;
  }

  // Wait for conversion - depends on oversampling ratio
  chThdSleepMilliseconds(6);

  /* Read ADC result */
  data[0] = MS56_CMD_ADC_READ;
  status = i2cMasterTransmit(i2cp, MS5637_I2C_ADDR, data, 1, 0, 0);
  if (status != MSG_OK) {
    return -3;
  }

  status = i2cMasterReceive(i2cp, MS5637_I2C_ADDR, data, 3);
  if (status != MSG_OK) {
    return -4;
  }

  uint32_t d2 = ((uint32_t)data[0] << 16) + (data[1] << 8) + data[2];

  /* Send pressure conversion command */
  data[0] = MS56_CMD_CONV_P_256 + 6;
  status = i2cMasterTransmit(i2cp, MS5637_I2C_ADDR, data, 1, 0, 0);
  if (status != MSG_OK) {
    return -5;
  }

  /* Wait for conversion - depends on oversampling ratio */
  chThdSleepMilliseconds(6);

  /* Read ADC result */
  data[0] = MS56_CMD_ADC_READ;
  status = i2cMasterTransmit(i2cp, MS5637_I2C_ADDR, data, 1, 0, 0);
  if (status != MSG_OK) {
    return -7;
  }

  status = i2cMasterReceive(i2cp, MS5637_I2C_ADDR, data, 3);
  if (status != MSG_OK) {
    return -8;
  }

  uint32_t d1 = ((uint32_t)data[0] << 16) + (data[1] << 8) + data[2];

  // Calculate the final temperature and pressure values
  int64_t dT = d2 - (((uint64_t)MS56_PROMData[5]) << 8);
  int32_t t = 2000 + ((dT * (int64_t)MS56_PROMData[6]) >> 23);

  int64_t off  = ((int64_t)MS56_PROMData[2] << 17) + ((dT * (int64_t)MS56_PROMData[4]) >> 6);
  int64_t sens = (((int64_t)MS56_PROMData[1]) << 16) + ((dT * (int64_t)MS56_PROMData[3]) >> 7);

  // Second order temperature compensation
  if (t < 2000) {
    double square = pow (dT,2);
    double t2 = square / (1L << 31);
    square = pow (t-2000,2);
    double off2  = square * 5 / 2;
    double sens2 = square * 5 / 4;
    if (t < 15) {
      square = pow(t+1500,2);
      off2  += square * 7;
      sens2 += square * 11 / 2;
    }

    t    -= t2;
    off  -= off2;
    sens -= sens2;
  }

  int64_t p0 = ((sens * (int64_t)d1) >> 21);
  int32_t p = (int32_t)((p0 - off) >> 15);
  msg->temperature = (float)t / 100.0f;
  msg->pressure = (float)p / 100.0f;;

  return 0;
}

static void set_baud(SerialDriver *sd, SerialConfig *cfg, uint32_t speed) {
  // Stop the serial driver.
  sdStop(sd);
  // Reinitialize the driver
  sdInit();
  // Wait a bit...
  chThdSleepMilliseconds(100);
  // Restart the driver with a new baud rate.
  cfg->speed = speed;
  sdStart(sd, cfg);
  // Wait a bit more...
  chThdSleepMilliseconds(1000);
}

/*
 * Sensor read thread
 */
static THD_WORKING_AREA(waSensorThread, 150);
static __attribute__((noreturn)) THD_FUNCTION(SensorThread, arg) {
  (void)arg;
  chRegSetThreadName("Sensor");
  baro_msg.sync1 = 0xB5;
  baro_msg.sync2 = 0x62;
  baro_msg.class = 01;
  baro_msg.id = 99;
  baro_msg.length = 10;
  while (1) {
    chThdSleepMilliseconds(200);
    int status = MS5637_ReadData(&I2CD1, &baro_msg);
    baro_msg.status = ((0 == status) ? 1 : 0);
    ubx_checksum((uint8_t*)&baro_msg);
  }
}

/*
 * UART1 recieve thread
 */
static THD_WORKING_AREA(waUSART1Thread, 128);
static __attribute__((noreturn)) THD_FUNCTION(USART1Thread, arg) {
  (void)arg;
  chRegSetThreadName("UART1");
  uint8_t buf[32];
  while (true) {
    uint8_t len = sdReadTimeout(&SD1, buf, 32, TIME_MS2I(10));
    if (len > 0) {
      chnWrite(&SD2, buf, len);
    }
  }
}

/*
 * UART2 recieve thread
 */
static THD_WORKING_AREA(waUSART2Thread, 256);
static __attribute__((noreturn)) THD_FUNCTION(USART2Thread, arg) {
  (void)arg;
  chRegSetThreadName("UART2");
  uint8_t NMEA_msg_len = 0;
  uint8_t UBX_msg_len = 0;
  uint8_t UBX_payload_len = 0;
  uint8_t parsing_UBX = 0;
  uint16_t recv_gap_counter = 0;
  while (true) {

    // Patialy parse NMEA/UBX messages to ensure we are getting valid data
    // And to allow insertion of our own messages.
    uint8_t val;
    if (sdReadTimeout(&SD2, &val, 1, TIME_MS2I(1))) {
      recv_gap_counter = 0;

      // Write the byte to the output channel.
      chnPutTimeout(&SD1, val, TIME_INFINITE);

      // Are we parsing an NMEA message?
      if (NMEA_msg_len) {
	++NMEA_msg_len;
	parsing_UBX = 0;
	// Is this message too long, or are we missing the start delminter?
	if ((NMEA_msg_len > 82) || ((NMEA_msg_len == 7) && (val != ','))) {
	  NMEA_msg_len = 0;
	} else if (val == '\n') {
	  // We recieved the end of a frame.
	  NMEA_msg_len = 0;
	  found_valid_message = 1;
	}
      } else if (UBX_msg_len) {
	// Are we parsing a UBX message?
	parsing_UBX = 1;
	++UBX_msg_len;

	if (UBX_msg_len == 2) {
	  // Is this the second sync char?
	  if (val == 0x62) {
	  } else {
	    UBX_msg_len = 0;
	  }
	} else if (UBX_msg_len == 5) {
	  // This should be the high byte of the length field.
	  // It's little-endian, so this is the least significant byte.
	  UBX_payload_len = val;
	} else if (UBX_msg_len == 6) {
	  // None of the messages that we are interested in are longer than 255 bytes.
	  // If this value is non-zero, we must have a parsing error.
	  if (val) {
	    UBX_msg_len = 0;
	  }
	} else if (UBX_msg_len == (UBX_payload_len + 8)) {
	  UBX_msg_len = 0;
	  found_valid_message = 1;
	}
      } else if (val == '$') {
	// Start of NMEA message.
	NMEA_msg_len = 1;
      } if (val == 0xB5) {
	// Start of UBX message.
	UBX_msg_len = 1;
      }

      // Turn on the LED when we're parsing a message.
      if ((UBX_msg_len == 0) && (NMEA_msg_len == 0)) {
	palClearLine(LINE_LED_BLUE);
      } else {
	palSetLine(LINE_LED_BLUE);
      }
    } else {
      ++recv_gap_counter;
    }

    if (recv_gap_counter == 20) {
      // Write the baro message
      if (parsing_UBX || 1) {
	chnWrite(&SD1, (uint8_t*)&baro_msg, sizeof(baro_msg));
      } else {
	uint8_t idx = 0;
	// Write the NMEA header.
	strncpy((char*)out_buffer + idx, "$GPBAR,1,", 40 - idx);
	idx += 9;
	// Write the temperature in degrees
	idx += ftoa(baro_msg.temperature, out_buffer + idx, 40 - idx, 2);
	out_buffer[idx++] = ',';
	// Write the pressure in milibar
	idx += ftoa(baro_msg.pressure, out_buffer + idx, 40 - idx, 2);
	out_buffer[idx++] = '\r';
	out_buffer[idx++] = '\n';
	chnWrite(&SD1, out_buffer, idx);
      }
    }
  }
}


/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 1 (output) and 2 (GPS)
   */
  uart1_config.speed = 115200;
  uart1_config.cr1 = 0;
  uart1_config.cr2 = USART_CR2_STOP1_BITS;
  uart1_config.cr3 = 0;
  sdStart(&SD1, &uart1_config);
  uart2_config.speed = 9600;
  uart2_config.cr1 = 0;
  uart2_config.cr2 = USART_CR2_STOP1_BITS;
  uart2_config.cr3 = 0;
  sdStart(&SD2, &uart2_config);

  /*
   * Initialize I2C1 for reading the sensors.
   */
  i2cStart(&I2CD1, &i2cfg1);

  /*
   * Initialize the ms5637 pressure/temperature sensor
   */
  int8_t status = MS5637_Setup(&I2CD1);
  if (status < 0) {
    while (1) {
      palSetLine(LINE_LED_BLUE);
      chThdSleepMilliseconds(1000);
      palClearLine(LINE_LED_BLUE);
      chThdSleepMilliseconds(1000 * -status);
    }
  }

  // Initialize to 9600 baud, which is the default baud rate of the GPS
  set_baud(&SD2, &uart2_config, 9600);
  chThdSleepMilliseconds(200);
  // Send the command to switch to 57600
  SEND_MSG(&SD2, ubx_57kbps);
  // Switch our baud rate to 57600
  set_baud(&SD2, &uart2_config, 57600);
 
  // Turn off all NMEA messages.
  SEND_MSG(&SD2, nmea_gga_msg);
  SEND_MSG(&SD2, nmea_gll_msg);
  SEND_MSG(&SD2, nmea_gsa_msg);
  SEND_MSG(&SD2, nmea_gsv_msg);
  SEND_MSG(&SD2, nmea_rmc_msg);
  SEND_MSG(&SD2, nmea_vtg_msg);

  // Turn on a selection of UBX messages.
  SEND_MSG(&SD2, ubx_posllh_msg);
  SEND_MSG(&SD2, ubx_status_msg);
  SEND_MSG(&SD2, ubx_sol_msg);
  SEND_MSG(&SD2, ubx_velned_msg);
  SEND_MSG(&SD2, ubx_timeutc_msg);
  SEND_MSG(&SD2, ubx_pvt_msg);
  SEND_MSG(&SD2, ubx_svinfo_msg);

  // Configure the rate to 5Hz
  SEND_MSG(&SD2, ubx_rate_200ms);

  // Configure the satallites that we will follow
  SEND_MSG(&SD2, ubx_cfg_gnns);

  /*
   * Create the threads.
   */
  chThdCreateStatic(waUSART1Thread, sizeof(waUSART1Thread), NORMALPRIO, USART1Thread, NULL);
  chThdCreateStatic(waUSART2Thread, sizeof(waUSART2Thread), NORMALPRIO, USART2Thread, NULL);
  chThdCreateStatic(waSensorThread, sizeof(waSensorThread), NORMALPRIO, SensorThread, NULL);

  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state.
   */
  while (true) {
    chThdSleepMilliseconds(500);
  }
}
