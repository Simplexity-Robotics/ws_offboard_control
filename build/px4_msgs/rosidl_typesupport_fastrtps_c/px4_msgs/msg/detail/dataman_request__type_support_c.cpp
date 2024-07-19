// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/DatamanRequest.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/dataman_request__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/dataman_request__struct.h"
#include "px4_msgs/msg/detail/dataman_request__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _DatamanRequest__ros_msg_type = px4_msgs__msg__DatamanRequest;

static bool _DatamanRequest__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _DatamanRequest__ros_msg_type * ros_message = static_cast<const _DatamanRequest__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: client_id
  {
    cdr << ros_message->client_id;
  }

  // Field name: request_type
  {
    cdr << ros_message->request_type;
  }

  // Field name: item
  {
    cdr << ros_message->item;
  }

  // Field name: index
  {
    cdr << ros_message->index;
  }

  // Field name: data
  {
    size_t size = 56;
    auto array_ptr = ros_message->data;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: data_length
  {
    cdr << ros_message->data_length;
  }

  return true;
}

static bool _DatamanRequest__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _DatamanRequest__ros_msg_type * ros_message = static_cast<_DatamanRequest__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: client_id
  {
    cdr >> ros_message->client_id;
  }

  // Field name: request_type
  {
    cdr >> ros_message->request_type;
  }

  // Field name: item
  {
    cdr >> ros_message->item;
  }

  // Field name: index
  {
    cdr >> ros_message->index;
  }

  // Field name: data
  {
    size_t size = 56;
    auto array_ptr = ros_message->data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: data_length
  {
    cdr >> ros_message->data_length;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__DatamanRequest(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DatamanRequest__ros_msg_type * ros_message = static_cast<const _DatamanRequest__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name client_id
  {
    size_t item_size = sizeof(ros_message->client_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name request_type
  {
    size_t item_size = sizeof(ros_message->request_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name item
  {
    size_t item_size = sizeof(ros_message->item);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name index
  {
    size_t item_size = sizeof(ros_message->index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name data
  {
    size_t array_size = 56;
    auto array_ptr = ros_message->data;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name data_length
  {
    size_t item_size = sizeof(ros_message->data_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _DatamanRequest__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__DatamanRequest(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__DatamanRequest(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: client_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: request_type
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: item
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: index
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: data
  {
    size_t array_size = 56;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: data_length
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _DatamanRequest__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_px4_msgs__msg__DatamanRequest(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_DatamanRequest = {
  "px4_msgs::msg",
  "DatamanRequest",
  _DatamanRequest__cdr_serialize,
  _DatamanRequest__cdr_deserialize,
  _DatamanRequest__get_serialized_size,
  _DatamanRequest__max_serialized_size
};

static rosidl_message_type_support_t _DatamanRequest__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DatamanRequest,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, DatamanRequest)() {
  return &_DatamanRequest__type_support;
}

#if defined(__cplusplus)
}
#endif
