// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from px4_msgs:msg/LandingGearWheel.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "px4_msgs/msg/detail/landing_gear_wheel__rosidl_typesupport_introspection_c.h"
#include "px4_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "px4_msgs/msg/detail/landing_gear_wheel__functions.h"
#include "px4_msgs/msg/detail/landing_gear_wheel__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  px4_msgs__msg__LandingGearWheel__init(message_memory);
}

void LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_fini_function(void * message_memory)
{
  px4_msgs__msg__LandingGearWheel__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_message_member_array[2] = {
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__LandingGearWheel, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "normalized_wheel_setpoint",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__LandingGearWheel, normalized_wheel_setpoint),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_message_members = {
  "px4_msgs__msg",  // message namespace
  "LandingGearWheel",  // message name
  2,  // number of fields
  sizeof(px4_msgs__msg__LandingGearWheel),
  LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_message_member_array,  // message members
  LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_init_function,  // function to initialize message memory (memory has to be allocated)
  LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_message_type_support_handle = {
  0,
  &LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_px4_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, px4_msgs, msg, LandingGearWheel)() {
  if (!LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_message_type_support_handle.typesupport_identifier) {
    LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &LandingGearWheel__rosidl_typesupport_introspection_c__LandingGearWheel_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
