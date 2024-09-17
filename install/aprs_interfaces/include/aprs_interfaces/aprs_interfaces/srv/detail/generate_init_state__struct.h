// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from aprs_interfaces:srv/GenerateInitState.idl
// generated code does not contain a copyright notice

#ifndef APRS_INTERFACES__SRV__DETAIL__GENERATE_INIT_STATE__STRUCT_H_
#define APRS_INTERFACES__SRV__DETAIL__GENERATE_INIT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GenerateInitState in the package aprs_interfaces.
typedef struct aprs_interfaces__srv__GenerateInitState_Request
{
  uint8_t structure_needs_at_least_one_member;
} aprs_interfaces__srv__GenerateInitState_Request;

// Struct for a sequence of aprs_interfaces__srv__GenerateInitState_Request.
typedef struct aprs_interfaces__srv__GenerateInitState_Request__Sequence
{
  aprs_interfaces__srv__GenerateInitState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aprs_interfaces__srv__GenerateInitState_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GenerateInitState in the package aprs_interfaces.
typedef struct aprs_interfaces__srv__GenerateInitState_Response
{
  bool success;
  rosidl_runtime_c__String status;
} aprs_interfaces__srv__GenerateInitState_Response;

// Struct for a sequence of aprs_interfaces__srv__GenerateInitState_Response.
typedef struct aprs_interfaces__srv__GenerateInitState_Response__Sequence
{
  aprs_interfaces__srv__GenerateInitState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aprs_interfaces__srv__GenerateInitState_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  aprs_interfaces__srv__GenerateInitState_Event__request__MAX_SIZE = 1
};
// response
enum
{
  aprs_interfaces__srv__GenerateInitState_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GenerateInitState in the package aprs_interfaces.
typedef struct aprs_interfaces__srv__GenerateInitState_Event
{
  service_msgs__msg__ServiceEventInfo info;
  aprs_interfaces__srv__GenerateInitState_Request__Sequence request;
  aprs_interfaces__srv__GenerateInitState_Response__Sequence response;
} aprs_interfaces__srv__GenerateInitState_Event;

// Struct for a sequence of aprs_interfaces__srv__GenerateInitState_Event.
typedef struct aprs_interfaces__srv__GenerateInitState_Event__Sequence
{
  aprs_interfaces__srv__GenerateInitState_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} aprs_interfaces__srv__GenerateInitState_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // APRS_INTERFACES__SRV__DETAIL__GENERATE_INIT_STATE__STRUCT_H_
