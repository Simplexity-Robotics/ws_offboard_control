// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from px4_msgs:msg/FuelTankStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/fuel_tank_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "px4_msgs/msg/detail/fuel_tank_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace px4_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
cdr_serialize(
  const px4_msgs::msg::FuelTankStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: timestamp
  cdr << ros_message.timestamp;
  // Member: maximum_fuel_capacity
  cdr << ros_message.maximum_fuel_capacity;
  // Member: consumed_fuel
  cdr << ros_message.consumed_fuel;
  // Member: fuel_consumption_rate
  cdr << ros_message.fuel_consumption_rate;
  // Member: percent_remaining
  cdr << ros_message.percent_remaining;
  // Member: remaining_fuel
  cdr << ros_message.remaining_fuel;
  // Member: fuel_tank_id
  cdr << ros_message.fuel_tank_id;
  // Member: fuel_type
  cdr << ros_message.fuel_type;
  // Member: temperature
  cdr << ros_message.temperature;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs::msg::FuelTankStatus & ros_message)
{
  // Member: timestamp
  cdr >> ros_message.timestamp;

  // Member: maximum_fuel_capacity
  cdr >> ros_message.maximum_fuel_capacity;

  // Member: consumed_fuel
  cdr >> ros_message.consumed_fuel;

  // Member: fuel_consumption_rate
  cdr >> ros_message.fuel_consumption_rate;

  // Member: percent_remaining
  cdr >> ros_message.percent_remaining;

  // Member: remaining_fuel
  cdr >> ros_message.remaining_fuel;

  // Member: fuel_tank_id
  cdr >> ros_message.fuel_tank_id;

  // Member: fuel_type
  cdr >> ros_message.fuel_type;

  // Member: temperature
  cdr >> ros_message.temperature;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
get_serialized_size(
  const px4_msgs::msg::FuelTankStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: timestamp
  {
    size_t item_size = sizeof(ros_message.timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: maximum_fuel_capacity
  {
    size_t item_size = sizeof(ros_message.maximum_fuel_capacity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: consumed_fuel
  {
    size_t item_size = sizeof(ros_message.consumed_fuel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fuel_consumption_rate
  {
    size_t item_size = sizeof(ros_message.fuel_consumption_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: percent_remaining
  {
    size_t item_size = sizeof(ros_message.percent_remaining);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: remaining_fuel
  {
    size_t item_size = sizeof(ros_message.remaining_fuel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fuel_tank_id
  {
    size_t item_size = sizeof(ros_message.fuel_tank_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fuel_type
  {
    size_t item_size = sizeof(ros_message.fuel_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: temperature
  {
    size_t item_size = sizeof(ros_message.temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
max_serialized_size_FuelTankStatus(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: maximum_fuel_capacity
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: consumed_fuel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: fuel_consumption_rate
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: percent_remaining
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: remaining_fuel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: fuel_tank_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: fuel_type
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _FuelTankStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::FuelTankStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _FuelTankStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<px4_msgs::msg::FuelTankStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _FuelTankStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::FuelTankStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _FuelTankStatus__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_FuelTankStatus(full_bounded, 0);
}

static message_type_support_callbacks_t _FuelTankStatus__callbacks = {
  "px4_msgs::msg",
  "FuelTankStatus",
  _FuelTankStatus__cdr_serialize,
  _FuelTankStatus__cdr_deserialize,
  _FuelTankStatus__get_serialized_size,
  _FuelTankStatus__max_serialized_size
};

static rosidl_message_type_support_t _FuelTankStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FuelTankStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace px4_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_px4_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<px4_msgs::msg::FuelTankStatus>()
{
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_FuelTankStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, px4_msgs, msg, FuelTankStatus)() {
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_FuelTankStatus__handle;
}

#ifdef __cplusplus
}
#endif
