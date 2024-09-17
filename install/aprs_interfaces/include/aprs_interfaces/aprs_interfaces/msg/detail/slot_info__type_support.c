// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from aprs_interfaces:msg/SlotInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "aprs_interfaces/msg/detail/slot_info__rosidl_typesupport_introspection_c.h"
#include "aprs_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "aprs_interfaces/msg/detail/slot_info__functions.h"
#include "aprs_interfaces/msg/detail/slot_info__struct.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aprs_interfaces__msg__SlotInfo__init(message_memory);
}

void aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_fini_function(void * message_memory)
{
  aprs_interfaces__msg__SlotInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_message_member_array[3] = {
  {
    "occupied",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aprs_interfaces__msg__SlotInfo, occupied),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aprs_interfaces__msg__SlotInfo, size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aprs_interfaces__msg__SlotInfo, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_message_members = {
  "aprs_interfaces__msg",  // message namespace
  "SlotInfo",  // message name
  3,  // number of fields
  sizeof(aprs_interfaces__msg__SlotInfo),
  aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_message_member_array,  // message members
  aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_message_type_support_handle = {
  0,
  &aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_message_members,
  get_message_typesupport_handle_function,
  &aprs_interfaces__msg__SlotInfo__get_type_hash,
  &aprs_interfaces__msg__SlotInfo__get_type_description,
  &aprs_interfaces__msg__SlotInfo__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aprs_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aprs_interfaces, msg, SlotInfo)() {
  if (!aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_message_type_support_handle.typesupport_identifier) {
    aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &aprs_interfaces__msg__SlotInfo__rosidl_typesupport_introspection_c__SlotInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
