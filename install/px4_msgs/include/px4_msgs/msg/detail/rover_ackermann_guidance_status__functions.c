// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/RoverAckermannGuidanceStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/rover_ackermann_guidance_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
px4_msgs__msg__RoverAckermannGuidanceStatus__init(px4_msgs__msg__RoverAckermannGuidanceStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // actual_speed
  // desired_speed
  // lookahead_distance
  // heading_error
  // pid_throttle_integral
  // crosstrack_error
  return true;
}

void
px4_msgs__msg__RoverAckermannGuidanceStatus__fini(px4_msgs__msg__RoverAckermannGuidanceStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // actual_speed
  // desired_speed
  // lookahead_distance
  // heading_error
  // pid_throttle_integral
  // crosstrack_error
}

bool
px4_msgs__msg__RoverAckermannGuidanceStatus__are_equal(const px4_msgs__msg__RoverAckermannGuidanceStatus * lhs, const px4_msgs__msg__RoverAckermannGuidanceStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // actual_speed
  if (lhs->actual_speed != rhs->actual_speed) {
    return false;
  }
  // desired_speed
  if (lhs->desired_speed != rhs->desired_speed) {
    return false;
  }
  // lookahead_distance
  if (lhs->lookahead_distance != rhs->lookahead_distance) {
    return false;
  }
  // heading_error
  if (lhs->heading_error != rhs->heading_error) {
    return false;
  }
  // pid_throttle_integral
  if (lhs->pid_throttle_integral != rhs->pid_throttle_integral) {
    return false;
  }
  // crosstrack_error
  if (lhs->crosstrack_error != rhs->crosstrack_error) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__RoverAckermannGuidanceStatus__copy(
  const px4_msgs__msg__RoverAckermannGuidanceStatus * input,
  px4_msgs__msg__RoverAckermannGuidanceStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // actual_speed
  output->actual_speed = input->actual_speed;
  // desired_speed
  output->desired_speed = input->desired_speed;
  // lookahead_distance
  output->lookahead_distance = input->lookahead_distance;
  // heading_error
  output->heading_error = input->heading_error;
  // pid_throttle_integral
  output->pid_throttle_integral = input->pid_throttle_integral;
  // crosstrack_error
  output->crosstrack_error = input->crosstrack_error;
  return true;
}

px4_msgs__msg__RoverAckermannGuidanceStatus *
px4_msgs__msg__RoverAckermannGuidanceStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverAckermannGuidanceStatus * msg = (px4_msgs__msg__RoverAckermannGuidanceStatus *)allocator.allocate(sizeof(px4_msgs__msg__RoverAckermannGuidanceStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__RoverAckermannGuidanceStatus));
  bool success = px4_msgs__msg__RoverAckermannGuidanceStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__RoverAckermannGuidanceStatus__destroy(px4_msgs__msg__RoverAckermannGuidanceStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    px4_msgs__msg__RoverAckermannGuidanceStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence__init(px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverAckermannGuidanceStatus * data = NULL;

  if (size) {
    data = (px4_msgs__msg__RoverAckermannGuidanceStatus *)allocator.zero_allocate(size, sizeof(px4_msgs__msg__RoverAckermannGuidanceStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__RoverAckermannGuidanceStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__RoverAckermannGuidanceStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence__fini(px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__RoverAckermannGuidanceStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence *
px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence * array = (px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence *)allocator.allocate(sizeof(px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence__destroy(px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence__are_equal(const px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence * lhs, const px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__RoverAckermannGuidanceStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence__copy(
  const px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence * input,
  px4_msgs__msg__RoverAckermannGuidanceStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__RoverAckermannGuidanceStatus);
    px4_msgs__msg__RoverAckermannGuidanceStatus * data =
      (px4_msgs__msg__RoverAckermannGuidanceStatus *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__RoverAckermannGuidanceStatus__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__RoverAckermannGuidanceStatus__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!px4_msgs__msg__RoverAckermannGuidanceStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
