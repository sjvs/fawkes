
/***************************************************************************
 *  roomba_500.cpp - Roomba Open Interface implementation for 500 series
 *
 *  Created: Sat Jan 01 19:37:13 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "roomba_500.h"

#include <core/exceptions/system.h>

#include <cstring>
#include <cstdlib>
#include <cmath>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <netinet/in.h>

using namespace fawkes;

/// @cond SELFEXPLAINING
const unsigned char Roomba500::BUTTON_CLEAN	=   1;
const unsigned char Roomba500::BUTTON_SPOT	=   2;
const unsigned char Roomba500::BUTTON_DOCK	=   4;
const unsigned char Roomba500::BUTTON_MINUTE	=   8;
const unsigned char Roomba500::BUTTON_HOUR	=  16;
const unsigned char Roomba500::BUTTON_DAY	=  32;
const unsigned char Roomba500::BUTTON_SCHEDULE	=  64;
const unsigned char Roomba500::BUTTON_CLOCK	= 128;

const unsigned char Roomba500::WHEEL_DROP_LEFT  =   8;
const unsigned char Roomba500::WHEEL_DROP_RIGHT =   4;
const unsigned char Roomba500::BUMP_LEFT        =   2;
const unsigned char Roomba500::BUMP_RIGHT       =   1;

const unsigned char Roomba500::OVERCURRENT_WHEEL_LEFT	= 16;
const unsigned char Roomba500::OVERCURRENT_WHEEL_RIGHT	=  8;
const unsigned char Roomba500::OVERCURRENT_MAIN_BRUSH	=  4;
const unsigned char Roomba500::OVERCURRENT_SIDE_BRUSH	=  1;

const unsigned char Roomba500::CHARGING_SOURCE_HOME_BASE = 2;
const unsigned char Roomba500::CHARGING_SOURCE_INTERNAL  = 1;

const unsigned char Roomba500::BUMPER_LEFT		=  1;
const unsigned char Roomba500::BUMPER_FRONT_LEFT	=  2;
const unsigned char Roomba500::BUMPER_CENTER_LEFT	=  4;
const unsigned char Roomba500::BUMPER_CENTER_RIGHT	=  8;
const unsigned char Roomba500::BUMPER_FRONT_RIGHT	= 16;
const unsigned char Roomba500::BUMPER_RIGHT		= 32;

const unsigned char Roomba500::LED_DEBRIS		=  1;
const unsigned char Roomba500::LED_SPOT			=  2;
const unsigned char Roomba500::LED_DOCK			=  4;
const unsigned char Roomba500::LED_CHECK_ROBOT		=  8;

const unsigned char Roomba500::WEEKDAY_LED_SUN		=  1;
const unsigned char Roomba500::WEEKDAY_LED_MON		=  2;
const unsigned char Roomba500::WEEKDAY_LED_TUE		=  4;
const unsigned char Roomba500::WEEKDAY_LED_WED		=  8;
const unsigned char Roomba500::WEEKDAY_LED_THU		= 16;
const unsigned char Roomba500::WEEKDAY_LED_FRI		= 32;
const unsigned char Roomba500::WEEKDAY_LED_SAT		= 64;

const unsigned char Roomba500::SCHEDULING_LED_COLON		=  1;
const unsigned char Roomba500::SCHEDULING_LED_PM		=  2;
const unsigned char Roomba500::SCHEDULING_LED_AM		=  4;
const unsigned char Roomba500::SCHEDULING_LED_CLOCK		=  8;
const unsigned char Roomba500::SCHEDULING_LED_SCHEDULE		= 16;

const unsigned char Roomba500::DIGIT_LED_NORTH                  =  1;
const unsigned char Roomba500::DIGIT_LED_NORTH_WEST             = 32;
const unsigned char Roomba500::DIGIT_LED_NORTH_EAST             =  2;
const unsigned char Roomba500::DIGIT_LED_CENTER                 = 64;
const unsigned char Roomba500::DIGIT_LED_SOUTH_WEST             = 16;
const unsigned char Roomba500::DIGIT_LED_SOUTH_EAST             =  4;
const unsigned char Roomba500::DIGIT_LED_SOUTH                  =  8;

const unsigned char Roomba500::MOTOR_SIDE_BRUSH			=  1;
const unsigned char Roomba500::MOTOR_VACUUM			=  2;
const unsigned char Roomba500::MOTOR_MAIN_BRUSHES		=  4;
const unsigned char Roomba500::MOTOR_SIDE_BRUSH_BACKWARD	=  8;
const unsigned char Roomba500::MOTOR_MAIN_BRUSHES_BACKWARD	= 16;

const unsigned char Roomba500::CHARGER_HOME_BASE	= 2;
const unsigned char Roomba500::CHARGER_INTERNAL		= 1;

const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_0 = 26;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_1 = 10;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_2 = 6;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_3 = 10;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_4 = 14;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_5 = 12;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_6 = 52;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_ALL = 80;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_101 = 28;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_106 = 12;
const unsigned short int Roomba500::SENSPACK_SIZE_GROUP_107 = 9;
const unsigned short int Roomba500::SENSPACK_SIZE_BUMPS_DROPS = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_WALL = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_LEFT = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_FRONT_LEFT = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_FRONT_RIGHT = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_RIGHT = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_VIRTUAL_WALL = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_WHEEL_OVERCURRENTS = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_DIRT_DETECT = 1;	
const unsigned short int Roomba500::SENSPACK_SIZE_IR_CHAR_OMNI = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_IR_CHAR_LEFT = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_IR_CHAR_RIGHT = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_BUTTONS = 1;	
const unsigned short int Roomba500::SENSPACK_SIZE_DISTANCE = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_ANGLE = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CHARGING_STATE = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_VOLTAGE = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CURRENT = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_TEMPERATURE = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_BATTERY_CHARGE = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_BATTERY_CAPACITY = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_WALL_SIGNAL = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_LEFT_SIGNAL = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_FRONT_LEFT_SIGNAL = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_FRONT_RIGHT_SIGNAL= 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CLIFF_RIGHT_SIGNAL = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_CHARGE_SOURCES = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_OI_MODE = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_SONG_NUMBER = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_SONG_PLAYING = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_STREAM_PACKETS = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_REQ_VELOCITY = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_REQ_RADIUS = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_REQ_RIGHT_VELOCITY = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_REQ_LEFT_VELOCITY = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_RIGHT_ENCODER = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LEFT_ENCODER = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER = 1;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_LEFT = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_FRONT_LEFT = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_CENTER_LEFT = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_CENTER_RIGHT = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_FRONT_RIGHT = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LIGHT_BUMPER_RIGHT = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_LEFT_MOTOR_CURRENT = 2;	
const unsigned short int Roomba500::SENSPACK_SIZE_RIGHT_MOTOR_CURRENT = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_BRUSH_MOTOR_CURRENT = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_SIDE_BRUSH_MOTOR_CURRENT = 2;
const unsigned short int Roomba500::SENSPACK_SIZE_STASIS = 1;

const float Roomba500::DIAMETER          = 0.33;
const float Roomba500::BUMPER_X_OFFSET   = 0.05;
const float Roomba500::AXLE_LENGTH       = 0.235;

const float Roomba500::MAX_LIN_VEL_MM_S  = 500;
const float Roomba500::MAX_ANG_VEL_RAD_S = 2;
const float Roomba500::MAX_RADIUS_MM     = 2000;

const unsigned short int Roomba500::MAX_ENCODER_COUNT = 65535;
const short int Roomba500::MAX_PWM           = 255;

const unsigned short int Roomba500::STREAM_INTERVAL_MS = 15;
const unsigned short int Roomba500::MODE_CHANGE_WAIT_MS = 20;

const unsigned char Roomba500::CHECKSUM_SIZE = 1;

/// @endcond

/// @cond INTERNALS
#pragma pack(push,1)

typedef struct {
  int16_t velocity;
  int16_t radius;
} DriveParams;

typedef struct {
  int16_t left_velocity;
  int16_t right_velocity;
} DriveDirectParams;

typedef struct {
  int16_t left_pwm;
  int16_t right_pwm;
} DrivePWMParams;

typedef struct {
  uint8_t num_packets;
  uint8_t packet_id;
} StreamOnePacketParams;

#pragma pack(pop)
/// @endcond


/** @class Roomba500 "roomba_500.h"
 * Roomba 500 series communication class.
 * This class implements the serial communication with Roomba robots of the
 * 500 series.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param device_file device file for serial connection
 */
Roomba500::Roomba500(const char *device_file)
{
  __mode = MODE_OFF;
  __fd = -1;
  __packet_id = SENSPACK_GROUP_ALL;
  __sensors_enabled = false;
  __device_file = strdup(device_file);

  try {
    open();
  } catch (Exception &e) {
    free(__device_file);
    throw;
  }
}


/** Destructor. */
Roomba500::~Roomba500()
{
  close();
  free(__device_file);
}


/** Open serial port. */
void
Roomba500::open() {

  struct termios param;

  __fd = ::open(__device_file, O_NOCTTY | O_RDWR);
  if (__fd == -1) {
    throw CouldNotOpenFileException(__device_file, errno, "Cannot open device file");
  }

  if (tcgetattr(__fd, &param) == -1) {
    Exception e(errno, "Getting the port parameters failed");
    ::close(__fd);
    __fd = -1;
    throw e;
  }

  cfsetospeed(&param, B115200);
  cfsetispeed(&param, B115200);

  param.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG | IEXTEN);
  param.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK | PARMRK);

  // turn off hardware flow control
  param.c_iflag &= ~(IXON | IXOFF | IXANY);
  param.c_cflag &= ~CRTSCTS;

  param.c_cflag |= (CREAD | CLOCAL);
    
  // number of data bits: 8
  param.c_cflag &= ~CS5 & ~CS6 & ~CS7 & ~CS8;
  param.c_cflag |= CS8;
    
  // parity: none
  param.c_cflag &= ~(PARENB | PARODD);
  param.c_iflag &= ~(INPCK | ISTRIP);
    
  // stop bits: 1
  param.c_cflag &= ~CSTOPB;

  //enable raw output
  param.c_oflag &= ~OPOST;

  param.c_cc[VMIN]  = 1;
  param.c_cc[VTIME] = 0;
    
  tcflush(__fd, TCIOFLUSH);

  if (tcsetattr(__fd, TCSANOW, &param) != 0) {
    Exception e(errno, "Setting the port parameters failed");
    ::close(__fd);
    __fd = -1;
    throw e;
  }

  send(OPCODE_START);
  usleep(MODE_CHANGE_WAIT_MS * 1000);
  __mode = MODE_PASSIVE;
}


/** Close serial connection. */
void
Roomba500::close()
{
  if (__fd >= 0) {
    ::close(__fd);
    __fd = -1;
  }
  __mode = MODE_OFF;
}

/** Send instruction packet.
 * @param opcode Opcode of command
 * @param params parameter bytes of the message
 * @param plength length of the params array
 */
void
Roomba500::send(Roomba500::OpCode opcode,
		const void *params, const size_t plength)
{
  // Byte 0 and 1 must be 0xFF
  __obuffer[0] = opcode;
  __obuffer_length = 1;

  if (params && (plength > 0)) {
    if (plength > (sizeof(__obuffer) - __obuffer_length)) {
      throw Exception("Parameters for command %i too long, maximum length is %zu",
		      opcode, (sizeof(__obuffer) - __obuffer_length));
    }
    unsigned char *pbytes = (unsigned char *)params;
    for (size_t i = 0; i < plength; ++i) {
      __obuffer[1+i] = pbytes[i];
    }
    __obuffer_length += plength;
  }

  int written = write(__fd, __obuffer, __obuffer_length);

  /*
  printf("Wrote %i of %i bytes:\n", written, __obuffer_length);
  for (int i = 0; i < __obuffer_length; ++i) {
    printf("%2u %s", __obuffer[i], i == written ? "| " : "");
  }
  printf("\n");
  */

  if ( written < 0 ) {
    throw Exception(errno, "Failed to write Roomba 500 packet %i", opcode);
  } else if (written < __obuffer_length) {
    throw Exception("Failed to write Roomba 500 packet %i, "
		    "only %d of %d bytes sent",
		    opcode, written, __obuffer_length);
  }
}


/** Receive a packet.
 * @param index index in ibuffer to fill
 * @param num_bytes number of bytes to read
 * @param timeout_ms maximum wait time in miliseconds, 0 to wait indefinitely.
 */
void
Roomba500::recv(size_t index, size_t num_bytes, unsigned int timeout_ms)
{
  timeval timeout = {0, timeout_ms  * 1000};

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(__fd, &read_fds);

  int rv = 0;
  rv = select(__fd + 1, &read_fds, NULL, NULL, (timeout_ms > 0) ? &timeout : NULL);

  if ( rv == -1 ) {
   throw Exception(errno, "Roomba500::recv(): select on file descriptor failed");
  } else if ( rv == 0 ) {
    throw TimeoutException("Timeout while waiting for incoming Roomba data");
  }

  __ibuffer_length = 0;

  // get octets one by one
  int bytes_read = 0;
  while (bytes_read < (int)num_bytes) {
    int rv = read(__fd, &__ibuffer[index] +bytes_read, num_bytes -bytes_read);
    if (rv == -1) {
      throw Exception(errno, "Roomba500::recv(): read failed");
    }
    bytes_read += rv;
  }

  if (bytes_read < (int)num_bytes) {
    throw Exception("Roomba500::recv(): failed to read packet data");
  }

  __ibuffer_length = index + num_bytes;
}


/** Check if data is available.
 * @return true if data is available and can be read, false otherwise
 */
bool
Roomba500::is_data_available()
{
  if (!__sensors_enabled) {
    throw Exception("Roomba 500 sensors have not been enabled.");
  }

  timeval timeout = {0, 0};

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(__fd, &read_fds);

  int rv = 0;
  rv = select(__fd + 1, &read_fds, NULL, NULL, &timeout);

  return (rv > 0);
}


/** Read sensor values.
 * Enable sensors before using enable_sensors(). This method will block until new
 * data has been read. You can call is_data_available() to check if this method
 * will block or not.
 */
void
Roomba500::read_sensors()
{
  if (!__sensors_enabled) {
    throw Exception("Roomba 500 sensors have not been enabled.");
  }

  bool done = false;
  unsigned int skipped = 0;
  while (!done) {
    __ibuffer_length = 0;

    recv(__ibuffer_length, 1);
    if (__ibuffer[0] != 19) {
      ++skipped;
      continue;
    }

    recv(__ibuffer_length, 1);
    if (__ibuffer[1] != __packet_length + 1) {
      ++skipped;
      continue;
    }

    recv(__ibuffer_length, 1);
    if (__ibuffer[2] != __packet_id) {
      ++skipped;
      continue;
    }

    recv(__ibuffer_length, __packet_length);

    recv(__ibuffer_length++, 1);

    unsigned int sum = 0;
    for (int i = 0; i < __ibuffer_length; ++i) {
      sum += __ibuffer[i];
    }

    if ((sum & 0xFF) != 0) {
      __sensor_packet_received = false;
    } else {
      memcpy(&__sensor_packet, &__ibuffer[3], sizeof(SensorPacketGroupAll));
      __sensor_packet_received = true;
    }

    done = true;
  }

  /*
  printf("Read %u bytes: ", __ibuffer_length);
  for (int i = 0; i < __ibuffer_length; ++i) {
    printf("%2u ", __ibuffer[i]);
  }
  printf(" (skipped %u)\n", skipped);
  */
}

/** Enable sensor data stream.
 * For simplicity and efficiency only the single SENSPACK_GROUP_ALL packet can
 * be streamed at this time. Make sure that the used connection is fast enough.
 * 56700 bit/sec should suffice, but 115200 is strongly recommended. If using
 * RooTooth make sure to use it in fast mode.
 */
void
Roomba500::enable_sensors()
{
  assert_connected();

  StreamOnePacketParams sp;
  sp.num_packets = 1;
  sp.packet_id = SENSPACK_GROUP_ALL;

  send(OPCODE_STREAM, &sp, sizeof(StreamOnePacketParams));

  __packet_id = SENSPACK_GROUP_ALL;
  __packet_reply_id = 19;
  __packet_length = get_packet_size(SENSPACK_GROUP_ALL);
  __sensors_enabled = true;
  __sensor_packet_received = false;
}


/** Disable sensor data stream. */
void
Roomba500::disable_sensors()
{
  assert_connected();

  if ( ! __sensors_enabled) {
    throw Exception("Roomba 500 sensors have not been enabled.");
  }

  unsigned char streamstate = STREAM_DISABLE;

  send(OPCODE_PAUSE_RESUME_STREAM, &streamstate, 1);

  __sensors_enabled = false;
  __sensor_packet_received = false;
}


/** Query sensor once..
 * For simplicity and efficiency only the single SENSPACK_GROUP_ALL packet can
 * be streamed at this time.
 */
void
Roomba500::query_sensors()
{
  assert_connected();

  unsigned char p = SENSPACK_GROUP_ALL;

  send(OPCODE_QUERY, &p, 1);

  __packet_id = SENSPACK_GROUP_ALL;
  __packet_reply_id = 0;
  __packet_length = get_packet_size(SENSPACK_GROUP_ALL);
  __sensor_packet_received = true;

  recv(0, __packet_length, 10);

  memcpy(&__sensor_packet, __ibuffer, sizeof(SensorPacketGroupAll));
}


/** Get latest sensor packet.
 * @return sensor packet
 */
const Roomba500::SensorPacketGroupAll &
Roomba500::get_sensor_packet() const
{
  if (! __sensor_packet_received) {
    throw Exception("No valid data received, yet.");
  }

  return __sensor_packet;
}

/** Set control mode.
 * @param mode the mode can be either MODE_FULL or MODE_SAFE (recommended).
 * MODE_OFF and MODE_PASSIVE cannot be set explicitly. To enter MODE_PASSIVE
 * issue a command which transitions mode, like clean() or seek_dock().
 * @exception Exception thrown if an invalid mode is passed
 */
void
Roomba500::set_mode(Roomba500::Mode mode)
{
  if (mode == MODE_PASSIVE) {
    send(OPCODE_START);
  } if (mode == MODE_SAFE) {
    send(OPCODE_SAFE);
  } else if (mode == MODE_FULL) {
    send(OPCODE_FULL);
  } else if (mode == MODE_OFF) {
    throw Exception("Mode OFF cannot be set, use power_down() instead");
  }

  usleep(MODE_CHANGE_WAIT_MS * 1000);
  __mode = mode;
}


/** Start normal cleaning operation.
 * Transitions to passive mode.
 */
void
Roomba500::clean()
{
  send(OPCODE_CLEAN);

  __mode = MODE_PASSIVE;
}


/** Start spot cleaning operation.
 * Transitions to passive mode.
 */
void
Roomba500::clean_spot()
{
  send(OPCODE_SPOT);

  __mode = MODE_PASSIVE;
}


/** Seek for the home base and dock.
 * Transitions to passive mode.
 */
void
Roomba500::seek_dock()
{
  send(OPCODE_SEEK_DOCK);

  __mode = MODE_PASSIVE;
}


/** Powers down the Roomba.
 * Transitions to passive mode.
 */
void
Roomba500::power_down()
{
  send(OPCODE_POWER);

  __mode = MODE_PASSIVE;
}


/** Stop moption of the Roomba.
 * Available only in safe or full mode.
 */
void
Roomba500::stop()
{
  assert_control();
  drive_pwm(0, 0);
}



/** Drive Roomba straight.
 * Available only in safe or full mode.
 * @param velocity_m_per_sec desired velocity in m/sec
 */
void
Roomba500::drive_straight(float velocity_m_per_sec)
{
  assert_control();

  int mm_per_sec = (int)roundf(velocity_m_per_sec * 1000.);
  if (mm_per_sec < -MAX_LIN_VEL_MM_S)  mm_per_sec = -MAX_LIN_VEL_MM_S;
  if (mm_per_sec >  MAX_LIN_VEL_MM_S)  mm_per_sec =  MAX_LIN_VEL_MM_S;

  DriveParams dp;
  dp.velocity = htons(mm_per_sec);
  dp.radius   = htons(0x8000);

  send(OPCODE_DRIVE, &dp, sizeof(DriveParams));
}

/** Turn robot on the spot.
 * Available only in safe or full mode.
 * @param direction turning direction
 */
void
Roomba500::drive_turn(Roomba500::TurnDirection direction)
{
  assert_control();

  DriveParams dp;
  dp.velocity = htons(0);
  dp.radius   = (direction == TURN_CLOCKWISE) ? -1 : 1;

  send(OPCODE_DRIVE, &dp, sizeof(DriveParams));
}


/** Drive Roomba on an arc.
 * Available only in safe or full mode.
 * @param velocity_m_per_sec desired velocity in m/sec
 * @param radius_m desired radius of arc in m
 */
void
Roomba500::drive_arc(float velocity_m_per_sec, float radius_m)
{
  assert_control();

  int velo_mm_per_sec = (int)roundf(velocity_m_per_sec * 1000.);
  if (velo_mm_per_sec < -MAX_LIN_VEL_MM_S)  velo_mm_per_sec = -MAX_LIN_VEL_MM_S;
  if (velo_mm_per_sec >  MAX_LIN_VEL_MM_S)  velo_mm_per_sec =  MAX_LIN_VEL_MM_S;

  int radius_mm = (int)roundf(radius_m * 1000.);
  if (radius_mm < -MAX_RADIUS_MM)  radius_mm = -MAX_RADIUS_MM;
  if (radius_mm >  MAX_RADIUS_MM)  radius_mm =  MAX_RADIUS_MM;

  DriveParams dp;
  dp.velocity = htons(velo_mm_per_sec);
  dp.radius   = htons(radius_mm);

  send(OPCODE_DRIVE, &dp, sizeof(DriveParams));  
}


/** Directly control wheel velocities.
 * Available only in safe or full mode.
 * @param left_velo_m_per_sec velocity of left wheel in m/sec
 * @param right_velo_m_per_sec velocity of right wheel in m/sec
 */
void
Roomba500::drive_direct(float left_velo_m_per_sec, float right_velo_m_per_sec)
{
  assert_control();

  int left_mm_per_sec = (int)roundf(left_velo_m_per_sec * 1000.);
  if (left_mm_per_sec < -MAX_LIN_VEL_MM_S)  left_mm_per_sec = -MAX_LIN_VEL_MM_S;
  if (left_mm_per_sec >  MAX_LIN_VEL_MM_S)  left_mm_per_sec =  MAX_LIN_VEL_MM_S;

  int right_mm_per_sec = (int)roundf(right_velo_m_per_sec * 1000.);
  if (right_mm_per_sec < -MAX_LIN_VEL_MM_S)  right_mm_per_sec = -MAX_LIN_VEL_MM_S;
  if (right_mm_per_sec >  MAX_LIN_VEL_MM_S)  right_mm_per_sec =  MAX_LIN_VEL_MM_S;

  DriveDirectParams dp;
  dp.left_velocity = htons(left_mm_per_sec);
  dp.right_velocity = htons(right_mm_per_sec);

  send(OPCODE_DRIVE, &dp, sizeof(DriveDirectParams));  
}


/** Directly control wheel velocities via PWM.
 * Available only in safe or full mode.
 * @param left_wheel_pwm PWM parameter for left wheel
 * @param right_wheel_pwm PWM parameter for right wheel
 */
void
Roomba500::drive_pwm(short int left_wheel_pwm, short int right_wheel_pwm)
{
  assert_control();

  if (left_wheel_pwm < -MAX_PWM)  left_wheel_pwm = -MAX_PWM;
  if (left_wheel_pwm >  MAX_PWM)  left_wheel_pwm =  MAX_PWM;

  if (right_wheel_pwm < -MAX_PWM)  right_wheel_pwm = -MAX_PWM;
  if (right_wheel_pwm >  MAX_PWM)  right_wheel_pwm =  MAX_PWM;

  DrivePWMParams dp;
  dp.left_pwm  = htons(left_wheel_pwm);
  dp.right_pwm = htons(right_wheel_pwm);

  send(OPCODE_DRIVE, &dp, sizeof(DrivePWMParams));  
}


/** Disable all brushes (and vacuuming).
 * Available only in safe or full mode.
 */
void
Roomba500::disable_brushes()
{
  assert_control();

  unsigned char param = 0;

  send(OPCODE_MOTORS, &param, 1);
}


/** Enable brushes (and vacuum).
 * Available only in safe or full mode.
 * @param main true to enable main brushes
 * @param side true to enable side brush
 * @param vacuum true to enable vacuuming
 * @param main_backward true to enable backward operation of main brushes
 * @param side_backward true to enable backward operation of side brush
 */
void
Roomba500::enable_brushes(bool main, bool side, bool vacuum,
			  bool main_backward, bool side_backward)
{
  assert_control();

  unsigned char param = 0;
  if (main)           param |= MOTOR_MAIN_BRUSHES;
  if (side)           param |= MOTOR_SIDE_BRUSH;
  if (vacuum)         param |= MOTOR_VACUUM;
  if (main_backward)  param |= MOTOR_MAIN_BRUSHES_BACKWARD;
  if (side_backward)  param |= MOTOR_SIDE_BRUSH_BACKWARD;

  send(OPCODE_MOTORS, &param, 1);
}


/** Set LED status of main LEDs.
 * Available only in safe or full mode.
 * @param debris true to enable debris LED, false to disable
 * @param spot true to enable spot LED, false to disable
 * @param dock true to enable dock LED, false to disable
 * @param check_robot true to enable check_robot LED, false to disable
 * @param clean_color color of clean button LED from green (0) to red (255)
 * @param clean_intensity intensity of clean button LED from off (0) to
 * full intensity (255)
 */
void
Roomba500::set_leds(bool debris, bool spot, bool dock, bool check_robot,
		    char clean_color, char clean_intensity)
{
  assert_control();

  unsigned char param[3] = {0, clean_color, clean_intensity};
  if (debris)       param[0] |= LED_DEBRIS;
  if (spot)         param[0] |= LED_SPOT;
  if (dock)         param[0] |= LED_DOCK;
  if (check_robot)  param[0] |= LED_CHECK_ROBOT;

  send(OPCODE_LEDS, &param, 3);
}


/** Get size of packet.
 * @param packet ID of packet to query size for
 * @return size of packet
 * @exception Exception thrown for unknown packet IDs.
 */
unsigned short int
Roomba500::get_packet_size(Roomba500::SensorPacketID packet)
{
  switch (packet) {
  case SENSPACK_BUMPS_DROPS:		return SENSPACK_SIZE_BUMPS_DROPS;
  case SENSPACK_WALL:			return SENSPACK_SIZE_WALL;
  case SENSPACK_CLIFF_LEFT:		return SENSPACK_SIZE_CLIFF_LEFT;
  case SENSPACK_CLIFF_FRONT_LEFT:	return SENSPACK_SIZE_CLIFF_FRONT_LEFT;
  case SENSPACK_CLIFF_FRONT_RIGHT:	return SENSPACK_SIZE_CLIFF_FRONT_RIGHT;
  case SENSPACK_CLIFF_RIGHT:		return SENSPACK_SIZE_CLIFF_RIGHT;
  case SENSPACK_VIRTUAL_WALL:		return SENSPACK_SIZE_VIRTUAL_WALL;
  case SENSPACK_WHEEL_OVERCURRENTS:	return SENSPACK_SIZE_WHEEL_OVERCURRENTS;
  case SENSPACK_DIRT_DETECT:		return SENSPACK_SIZE_DIRT_DETECT;
  case SENSPACK_IR_CHAR_OMNI:		return SENSPACK_SIZE_IR_CHAR_OMNI;
  case SENSPACK_BUTTONS:		return SENSPACK_SIZE_BUTTONS;
  case SENSPACK_DISTANCE:		return SENSPACK_SIZE_DISTANCE;
  case SENSPACK_ANGLE:			return SENSPACK_SIZE_ANGLE;
  case SENSPACK_CHARGING_STATE:		return SENSPACK_SIZE_CHARGING_STATE;
  case SENSPACK_VOLTAGE:		return SENSPACK_SIZE_VOLTAGE;
  case SENSPACK_CURRENT:		return SENSPACK_SIZE_CURRENT;
  case SENSPACK_TEMPERATURE:		return SENSPACK_SIZE_TEMPERATURE;
  case SENSPACK_BATTERY_CHARGE:		return SENSPACK_SIZE_BATTERY_CHARGE;
  case SENSPACK_BATTERY_CAPACITY:	return SENSPACK_SIZE_BATTERY_CAPACITY;
  case SENSPACK_WALL_SIGNAL:		return SENSPACK_SIZE_WALL_SIGNAL;
  case SENSPACK_CLIFF_LEFT_SIGNAL:	return SENSPACK_SIZE_CLIFF_LEFT_SIGNAL;
  case SENSPACK_CLIFF_FRONT_LEFT_SIGNAL:
    return SENSPACK_SIZE_CLIFF_FRONT_LEFT_SIGNAL;
  case SENSPACK_CLIFF_FRONT_RIGHT_SIGNAL:
    return SENSPACK_SIZE_CLIFF_FRONT_RIGHT_SIGNAL;
  case SENSPACK_CLIFF_RIGHT_SIGNAL:	return SENSPACK_SIZE_CLIFF_RIGHT_SIGNAL;
  case SENSPACK_CHARGE_SOURCES:		return SENSPACK_SIZE_CHARGE_SOURCES;
  case SENSPACK_OI_MODE:		return SENSPACK_SIZE_OI_MODE;
  case SENSPACK_SONG_NUMBER:		return SENSPACK_SIZE_SONG_NUMBER;
  case SENSPACK_SONG_PLAYING:		return SENSPACK_SIZE_SONG_PLAYING;
  case SENSPACK_STREAM_PACKETS:		return SENSPACK_SIZE_STREAM_PACKETS;
  case SENSPACK_REQ_VELOCITY:		return SENSPACK_SIZE_REQ_VELOCITY;
  case SENSPACK_REQ_RADIUS:		return SENSPACK_SIZE_REQ_RADIUS;
  case SENSPACK_REQ_RIGHT_VELOCITY:	return SENSPACK_SIZE_REQ_RIGHT_VELOCITY;
  case SENSPACK_REQ_LEFT_VELOCITY:	return SENSPACK_SIZE_REQ_LEFT_VELOCITY;
  case SENSPACK_RIGHT_ENCODER:		return SENSPACK_SIZE_RIGHT_ENCODER;
  case SENSPACK_LEFT_ENCODER:		return SENSPACK_SIZE_LEFT_ENCODER;
  case SENSPACK_LIGHT_BUMPER:		return SENSPACK_SIZE_LIGHT_BUMPER;
  case SENSPACK_LIGHT_BUMPER_LEFT:	return SENSPACK_SIZE_LIGHT_BUMPER_LEFT;
  case SENSPACK_LIGHT_BUMPER_FRONT_LEFT:
    return SENSPACK_SIZE_LIGHT_BUMPER_FRONT_LEFT;
  case SENSPACK_LIGHT_BUMPER_CENTER_LEFT:
    return SENSPACK_SIZE_LIGHT_BUMPER_CENTER_LEFT;
  case SENSPACK_LIGHT_BUMPER_CENTER_RIGHT:
    return SENSPACK_SIZE_LIGHT_BUMPER_CENTER_RIGHT;
  case SENSPACK_LIGHT_BUMPER_FRONT_RIGHT:
    return SENSPACK_SIZE_LIGHT_BUMPER_FRONT_RIGHT;
  case SENSPACK_LIGHT_BUMPER_RIGHT:
    return SENSPACK_SIZE_LIGHT_BUMPER_RIGHT;
  case SENSPACK_IR_CHAR_LEFT:		return SENSPACK_SIZE_IR_CHAR_LEFT;
  case SENSPACK_IR_CHAR_RIGHT:		return SENSPACK_SIZE_IR_CHAR_RIGHT;
  case SENSPACK_LEFT_MOTOR_CURRENT:	return SENSPACK_SIZE_LEFT_MOTOR_CURRENT;
  case SENSPACK_RIGHT_MOTOR_CURRENT:	return SENSPACK_SIZE_RIGHT_MOTOR_CURRENT;
  case SENSPACK_BRUSH_MOTOR_CURRENT:	return SENSPACK_SIZE_BRUSH_MOTOR_CURRENT;
  case SENSPACK_SIDE_BRUSH_MOTOR_CURRENT:
    return SENSPACK_SIZE_SIDE_BRUSH_MOTOR_CURRENT;
  case SENSPACK_STASIS:			return SENSPACK_SIZE_STASIS;
  case SENSPACK_GROUP_0:		return SENSPACK_SIZE_GROUP_0;
  case SENSPACK_GROUP_1:		return SENSPACK_SIZE_GROUP_1;
  case SENSPACK_GROUP_2:		return SENSPACK_SIZE_GROUP_2;
  case SENSPACK_GROUP_3:		return SENSPACK_SIZE_GROUP_3;
  case SENSPACK_GROUP_4:		return SENSPACK_SIZE_GROUP_4;
  case SENSPACK_GROUP_5:		return SENSPACK_SIZE_GROUP_5;
  case SENSPACK_GROUP_6:		return SENSPACK_SIZE_GROUP_6;
  case SENSPACK_GROUP_ALL:		return SENSPACK_SIZE_GROUP_ALL;
  case SENSPACK_GROUP_101:		return SENSPACK_SIZE_GROUP_101;
  case SENSPACK_GROUP_106:		return SENSPACK_SIZE_GROUP_106;
  case SENSPACK_GROUP_107:		return SENSPACK_SIZE_GROUP_107;
  default:
    throw Exception("Roomba500:get_packet_size(): unknown packet ID %i", packet);
  }
}
