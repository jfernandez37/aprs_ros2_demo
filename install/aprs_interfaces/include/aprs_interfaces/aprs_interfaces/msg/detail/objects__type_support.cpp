// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from aprs_interfaces:msg/Objects.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "aprs_interfaces/msg/detail/objects__functions.h"
#include "aprs_interfaces/msg/detail/objects__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace aprs_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Objects_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) aprs_interfaces::msg::Objects(_init);
}

void Objects_fini_function(void * message_memory)
{
  auto typed_message = static_cast<aprs_interfaces::msg::Objects *>(message_memory);
  typed_message->~Objects();
}

size_t size_function__Objects__objects(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<aprs_interfaces::msg::Object> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Objects__objects(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<aprs_interfaces::msg::Object> *>(untyped_member);
  return &member[index];
}

void * get_function__Objects__objects(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<aprs_interfaces::msg::Object> *>(untyped_member);
  return &member[index];
}

void fetch_function__Objects__objects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const aprs_interfaces::msg::Object *>(
    get_const_function__Objects__objects(untyped_member, index));
  auto & value = *reinterpret_cast<aprs_interfaces::msg::Object *>(untyped_value);
  value = item;
}

void assign_function__Objects__objects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<aprs_interfaces::msg::Object *>(
    get_function__Objects__objects(untyped_member, index));
  const auto & value = *reinterpret_cast<const aprs_interfaces::msg::Object *>(untyped_value);
  item = value;
}

void resize_function__Objects__objects(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<aprs_interfaces::msg::Object> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Objects_message_member_array[1] = {
  {
    "objects",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<aprs_interfaces::msg::Object>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aprs_interfaces::msg::Objects, objects),  // bytes offset in struct
    nullptr,  // default value
    size_function__Objects__objects,  // size() function pointer
    get_const_function__Objects__objects,  // get_const(index) function pointer
    get_function__Objects__objects,  // get(index) function pointer
    fetch_function__Objects__objects,  // fetch(index, &value) function pointer
    assign_function__Objects__objects,  // assign(index, value) function pointer
    resize_function__Objects__objects  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Objects_message_members = {
  "aprs_interfaces::msg",  // message namespace
  "Objects",  // message name
  1,  // number of fields
  sizeof(aprs_interfaces::msg::Objects),
  Objects_message_member_array,  // message members
  Objects_init_function,  // function to initialize message memory (memory has to be allocated)
  Objects_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Objects_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Objects_message_members,
  get_message_typesupport_handle_function,
  &aprs_interfaces__msg__Objects__get_type_hash,
  &aprs_interfaces__msg__Objects__get_type_description,
  &aprs_interfaces__msg__Objects__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace aprs_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<aprs_interfaces::msg::Objects>()
{
  return &::aprs_interfaces::msg::rosidl_typesupport_introspection_cpp::Objects_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, aprs_interfaces, msg, Objects)() {
  return &::aprs_interfaces::msg::rosidl_typesupport_introspection_cpp::Objects_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
