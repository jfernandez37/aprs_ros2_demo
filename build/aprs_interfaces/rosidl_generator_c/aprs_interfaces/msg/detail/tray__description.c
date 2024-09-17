// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from aprs_interfaces:msg/Tray.idl
// generated code does not contain a copyright notice

#include "aprs_interfaces/msg/detail/tray__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_aprs_interfaces
const rosidl_type_hash_t *
aprs_interfaces__msg__Tray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x10, 0xe6, 0x38, 0x7d, 0xb3, 0xbd, 0x0f, 0xa4,
      0x9b, 0x9f, 0xc5, 0x45, 0xae, 0xb5, 0xa5, 0x95,
      0x1b, 0xc6, 0xa5, 0xed, 0xd1, 0xe6, 0xed, 0xc3,
      0x07, 0x87, 0xa2, 0x29, 0x15, 0x2a, 0xfc, 0xf3,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "aprs_interfaces/msg/detail/slot_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t aprs_interfaces__msg__SlotInfo__EXPECTED_HASH = {1, {
    0x32, 0x52, 0x8f, 0xef, 0xec, 0x41, 0x9a, 0x8f,
    0xe3, 0xa1, 0xf5, 0xdb, 0x80, 0x30, 0x50, 0x50,
    0xe6, 0xa5, 0xb3, 0x2e, 0x90, 0x62, 0xcf, 0x24,
    0x5d, 0x1c, 0x11, 0xec, 0xcb, 0x79, 0xcd, 0xe2,
  }};
#endif

static char aprs_interfaces__msg__Tray__TYPE_NAME[] = "aprs_interfaces/msg/Tray";
static char aprs_interfaces__msg__SlotInfo__TYPE_NAME[] = "aprs_interfaces/msg/SlotInfo";

// Define type names, field names, and default values
static char aprs_interfaces__msg__Tray__FIELD_NAME__identifier[] = "identifier";
static char aprs_interfaces__msg__Tray__FIELD_NAME__name[] = "name";
static char aprs_interfaces__msg__Tray__FIELD_NAME__slots[] = "slots";

static rosidl_runtime_c__type_description__Field aprs_interfaces__msg__Tray__FIELDS[] = {
  {
    {aprs_interfaces__msg__Tray__FIELD_NAME__identifier, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {aprs_interfaces__msg__Tray__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {aprs_interfaces__msg__Tray__FIELD_NAME__slots, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {aprs_interfaces__msg__SlotInfo__TYPE_NAME, 28, 28},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription aprs_interfaces__msg__Tray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {aprs_interfaces__msg__SlotInfo__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
aprs_interfaces__msg__Tray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {aprs_interfaces__msg__Tray__TYPE_NAME, 24, 24},
      {aprs_interfaces__msg__Tray__FIELDS, 3, 3},
    },
    {aprs_interfaces__msg__Tray__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&aprs_interfaces__msg__SlotInfo__EXPECTED_HASH, aprs_interfaces__msg__SlotInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = aprs_interfaces__msg__SlotInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint8 identifier # 13: SMALL_GEAR_TRAY, 14: MEDIUM_GEAR_TRAY, 15: LARGE_GEAR_TRAY 16: M2L1_KIT_TRAY, 17: S2L2_KIT_TRAY\n"
  "string name\n"
  "aprs_interfaces/SlotInfo[] slots\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
aprs_interfaces__msg__Tray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {aprs_interfaces__msg__Tray__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 165, 165},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
aprs_interfaces__msg__Tray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *aprs_interfaces__msg__Tray__get_individual_type_description_source(NULL),
    sources[1] = *aprs_interfaces__msg__SlotInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
