#pragma once

/*
 * Charge and preconditioning (departure) schedules, as reported by the car in
 * CarServer.VehicleData.charge_schedule_state (field 15) and
 * preconditioning_schedule_state (field 16).
 *
 * The nanopb structs keep the schedule lists and names as callbacks (unbounded
 * repeated/string fields), so a plain pb_decode drops them. These types hold a
 * bounded copy extracted from the raw CarServer.Response bytes instead.
 */

#include <pb.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace TeslaBLE {

// Upper bounds keep memory use predictable on small targets.
static constexpr size_t MAX_SCHEDULES = 16;
static constexpr size_t MAX_SCHEDULE_NAME_LENGTH = 32;

// days_of_week is a bitmask: bit 0 = Sunday ... bit 6 = Saturday
// (same encoding as vehicle-command's charge/precondition schedules).
struct ChargeScheduleEntry {
  uint64_t id = 0;  // creation time, epoch seconds; stable identifier
  std::string name;
  int32_t days_of_week = 0;
  bool start_enabled = false;
  int32_t start_time = 0;  // minutes after midnight, local car time
  bool end_enabled = false;
  int32_t end_time = 0;  // minutes after midnight, local car time
  bool one_time = false;
  bool enabled = false;
  float latitude = 0.0f;
  float longitude = 0.0f;
};

struct PreconditionScheduleEntry {
  uint64_t id = 0;
  std::string name;
  int32_t days_of_week = 0;
  int32_t precondition_time = 0;  // departure time, minutes after midnight
  bool one_time = false;
  bool enabled = false;
  float latitude = 0.0f;
  float longitude = 0.0f;
};

struct ChargeScheduleList {
  std::vector<ChargeScheduleEntry> schedules;
  bool truncated = false;  // more than MAX_SCHEDULES entries were present
  bool has_window = false;
  ChargeScheduleEntry window;  // the active / next charge window
  bool has_charge_buffer = false;
  int32_t charge_buffer = 0;
  bool has_max_num_schedules = false;
  uint32_t max_num_schedules = 0;
  bool has_next_schedule = false;
  bool next_schedule = false;
  bool has_show_schedule_complete_state = false;
  bool show_schedule_complete_state = false;
};

struct PreconditionScheduleList {
  std::vector<PreconditionScheduleEntry> schedules;
  bool truncated = false;
  bool has_window = false;
  PreconditionScheduleEntry window;  // the active / next departure
  bool has_max_num_schedules = false;
  uint32_t max_num_schedules = 0;
  bool has_next_schedule = false;
  bool next_schedule = false;
};

/**
 * Read CarServer.Response.vehicleData.charge_schedule_state from an encoded
 * (plaintext) CarServer.Response. Returns false if the response holds no
 * charge schedule state or it is malformed.
 */
bool extract_charge_schedule_state(const pb_byte_t *response, size_t length, ChargeScheduleList *output);

/**
 * Same for vehicleData.preconditioning_schedule_state.
 */
bool extract_preconditioning_schedule_state(const pb_byte_t *response, size_t length, PreconditionScheduleList *output);

}  // namespace TeslaBLE
