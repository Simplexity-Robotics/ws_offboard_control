// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/SensorGps.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__SENSOR_GPS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__SENSOR_GPS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FIX_TYPE_NONE'.
enum
{
  px4_msgs__msg__SensorGps__FIX_TYPE_NONE = 1
};

/// Constant 'FIX_TYPE_2D'.
enum
{
  px4_msgs__msg__SensorGps__FIX_TYPE_2D = 2
};

/// Constant 'FIX_TYPE_3D'.
enum
{
  px4_msgs__msg__SensorGps__FIX_TYPE_3D = 3
};

/// Constant 'FIX_TYPE_RTCM_CODE_DIFFERENTIAL'.
enum
{
  px4_msgs__msg__SensorGps__FIX_TYPE_RTCM_CODE_DIFFERENTIAL = 4
};

/// Constant 'FIX_TYPE_RTK_FLOAT'.
enum
{
  px4_msgs__msg__SensorGps__FIX_TYPE_RTK_FLOAT = 5
};

/// Constant 'FIX_TYPE_RTK_FIXED'.
enum
{
  px4_msgs__msg__SensorGps__FIX_TYPE_RTK_FIXED = 6
};

/// Constant 'FIX_TYPE_EXTRAPOLATED'.
enum
{
  px4_msgs__msg__SensorGps__FIX_TYPE_EXTRAPOLATED = 8
};

/// Constant 'JAMMING_STATE_UNKNOWN'.
enum
{
  px4_msgs__msg__SensorGps__JAMMING_STATE_UNKNOWN = 0
};

/// Constant 'JAMMING_STATE_OK'.
enum
{
  px4_msgs__msg__SensorGps__JAMMING_STATE_OK = 1
};

/// Constant 'JAMMING_STATE_WARNING'.
enum
{
  px4_msgs__msg__SensorGps__JAMMING_STATE_WARNING = 2
};

/// Constant 'JAMMING_STATE_CRITICAL'.
enum
{
  px4_msgs__msg__SensorGps__JAMMING_STATE_CRITICAL = 3
};

/// Constant 'SPOOFING_STATE_UNKNOWN'.
enum
{
  px4_msgs__msg__SensorGps__SPOOFING_STATE_UNKNOWN = 0
};

/// Constant 'SPOOFING_STATE_NONE'.
enum
{
  px4_msgs__msg__SensorGps__SPOOFING_STATE_NONE = 1
};

/// Constant 'SPOOFING_STATE_INDICATED'.
enum
{
  px4_msgs__msg__SensorGps__SPOOFING_STATE_INDICATED = 2
};

/// Constant 'SPOOFING_STATE_MULTIPLE'.
enum
{
  px4_msgs__msg__SensorGps__SPOOFING_STATE_MULTIPLE = 3
};

/// Constant 'RTCM_MSG_USED_UNKNOWN'.
enum
{
  px4_msgs__msg__SensorGps__RTCM_MSG_USED_UNKNOWN = 0
};

/// Constant 'RTCM_MSG_USED_NOT_USED'.
enum
{
  px4_msgs__msg__SensorGps__RTCM_MSG_USED_NOT_USED = 1
};

/// Constant 'RTCM_MSG_USED_USED'.
enum
{
  px4_msgs__msg__SensorGps__RTCM_MSG_USED_USED = 2
};

// Struct defined in msg/SensorGps in the package px4_msgs.
typedef struct px4_msgs__msg__SensorGps
{
  uint64_t timestamp;
  uint64_t timestamp_sample;
  uint32_t device_id;
  double latitude_deg;
  double longitude_deg;
  double altitude_msl_m;
  double altitude_ellipsoid_m;
  float s_variance_m_s;
  float c_variance_rad;
  uint8_t fix_type;
  float eph;
  float epv;
  float hdop;
  float vdop;
  int32_t noise_per_ms;
  uint16_t automatic_gain_control;
  uint8_t jamming_state;
  int32_t jamming_indicator;
  uint8_t spoofing_state;
  float vel_m_s;
  float vel_n_m_s;
  float vel_e_m_s;
  float vel_d_m_s;
  float cog_rad;
  bool vel_ned_valid;
  int32_t timestamp_time_relative;
  uint64_t time_utc_usec;
  uint8_t satellites_used;
  float heading;
  float heading_offset;
  float heading_accuracy;
  float rtcm_injection_rate;
  uint8_t selected_rtcm_instance;
  bool rtcm_crc_failed;
  uint8_t rtcm_msg_used;
} px4_msgs__msg__SensorGps;

// Struct for a sequence of px4_msgs__msg__SensorGps.
typedef struct px4_msgs__msg__SensorGps__Sequence
{
  px4_msgs__msg__SensorGps * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__SensorGps__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__SENSOR_GPS__STRUCT_H_
