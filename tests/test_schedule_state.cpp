#include <gtest/gtest.h>

#include "schedule_state.h"

#include "car_server.pb.h"
#include "common.pb.h"
#include "vehicle.pb.h"

#include <pb_encode.h>

#include <string>
#include <vector>

using namespace TeslaBLE;

namespace {

void append_varint(std::vector<pb_byte_t> &out, uint64_t value) {
  do {
    pb_byte_t byte = value & 0x7F;
    value >>= 7;
    if (value != 0) {
      byte |= 0x80;
    }
    out.push_back(byte);
  } while (value != 0);
}

std::vector<pb_byte_t> length_delimited(uint32_t field, const std::vector<pb_byte_t> &body) {
  std::vector<pb_byte_t> out;
  append_varint(out, (static_cast<uint64_t>(field) << 3) | PB_WT_STRING);
  append_varint(out, body.size());
  out.insert(out.end(), body.begin(), body.end());
  return out;
}

std::vector<pb_byte_t> varint_field(uint32_t field, uint64_t value) {
  std::vector<pb_byte_t> out;
  append_varint(out, (static_cast<uint64_t>(field) << 3) | PB_WT_VARINT);
  append_varint(out, value);
  return out;
}

void append(std::vector<pb_byte_t> &out, const std::vector<pb_byte_t> &more) {
  out.insert(out.end(), more.begin(), more.end());
}

bool encode_name(pb_ostream_t *stream, const pb_field_t *field, void *const *arg) {
  const auto *name = static_cast<const std::string *>(*arg);
  return pb_encode_tag_for_field(stream, field) &&
         pb_encode_string(stream, reinterpret_cast<const pb_byte_t *>(name->data()), name->size());
}

// Encodes a schedule with the generated nanopb descriptors, so the test
// follows the real field numbers rather than hand-written bytes.
template<typename PbType>
std::vector<pb_byte_t> encode(PbType pb, const pb_msgdesc_t *fields, const std::string &name) {
  pb.name.funcs.encode = &encode_name;
  pb.name.arg = const_cast<std::string *>(&name);
  std::vector<pb_byte_t> buffer(256);
  pb_ostream_t stream = pb_ostream_from_buffer(buffer.data(), buffer.size());
  EXPECT_TRUE(pb_encode(&stream, fields, &pb)) << PB_GET_ERROR(&stream);
  buffer.resize(stream.bytes_written);
  return buffer;
}

std::vector<pb_byte_t> charge_schedule(uint64_t id, const std::string &name, int32_t days, int32_t start, int32_t end,
                                       bool enabled) {
  CarServer_ChargeSchedule pb = CarServer_ChargeSchedule_init_default;
  pb.id = id;
  pb.days_of_week = days;
  pb.start_enabled = start >= 0;
  pb.start_time = start >= 0 ? start : 0;
  pb.end_enabled = end >= 0;
  pb.end_time = end >= 0 ? end : 0;
  pb.enabled = enabled;
  pb.latitude = 53.16f;
  pb.longitude = 6.79f;
  return encode(pb, CarServer_ChargeSchedule_fields, name);
}

std::vector<pb_byte_t> precondition_schedule(uint64_t id, const std::string &name, int32_t days, int32_t time,
                                             bool one_time) {
  CarServer_PreconditionSchedule pb = CarServer_PreconditionSchedule_init_default;
  pb.id = id;
  pb.days_of_week = days;
  pb.precondition_time = time;
  pb.one_time = one_time;
  pb.enabled = true;
  return encode(pb, CarServer_PreconditionSchedule_fields, name);
}

std::vector<pb_byte_t> response_with(uint32_t vehicle_data_field, const std::vector<pb_byte_t> &state) {
  return length_delimited(CarServer_Response_vehicleData_tag, length_delimited(vehicle_data_field, state));
}

}  // namespace

TEST(ScheduleStateTest, ExtractsChargeSchedulesWindowAndScalars) {
  std::vector<pb_byte_t> state;
  append(state, length_delimited(CarServer_ChargeScheduleState_charge_schedules_tag,
                                 charge_schedule(1700000000, "Home night", 62, 23 * 60, 7 * 60, true)));
  append(state, length_delimited(CarServer_ChargeScheduleState_charge_schedules_tag,
                                 charge_schedule(1700000001, "Weekend", 65, -1, 9 * 60 + 30, false)));
  append(state, length_delimited(CarServer_ChargeScheduleState_charge_schedule_window_tag,
                                 charge_schedule(1700000000, "Home night", 62, 23 * 60, 7 * 60, true)));
  append(state, varint_field(CarServer_ChargeScheduleState_charge_buffer_tag, 15));
  append(state, varint_field(CarServer_ChargeScheduleState_max_num_charge_schedules_tag, 10));
  append(state, varint_field(CarServer_ChargeScheduleState_next_schedule_tag, 1));
  auto response = response_with(CarServer_VehicleData_charge_schedule_state_tag, state);

  ChargeScheduleList list;
  ASSERT_TRUE(extract_charge_schedule_state(response.data(), response.size(), &list));
  ASSERT_EQ(list.schedules.size(), 2u);
  EXPECT_FALSE(list.truncated);

  const auto &first = list.schedules[0];
  EXPECT_EQ(first.id, 1700000000u);
  EXPECT_EQ(first.name, "Home night");
  EXPECT_EQ(first.days_of_week, 62);
  EXPECT_TRUE(first.start_enabled);
  EXPECT_EQ(first.start_time, 23 * 60);
  EXPECT_TRUE(first.end_enabled);
  EXPECT_EQ(first.end_time, 7 * 60);
  EXPECT_TRUE(first.enabled);
  EXPECT_FLOAT_EQ(first.latitude, 53.16f);

  const auto &second = list.schedules[1];
  EXPECT_EQ(second.name, "Weekend");
  EXPECT_FALSE(second.start_enabled);
  EXPECT_EQ(second.end_time, 9 * 60 + 30);
  EXPECT_FALSE(second.enabled);

  EXPECT_TRUE(list.has_window);
  EXPECT_EQ(list.window.name, "Home night");
  EXPECT_EQ(list.window.start_time, 23 * 60);
  EXPECT_TRUE(list.has_charge_buffer);
  EXPECT_EQ(list.charge_buffer, 15);
  EXPECT_TRUE(list.has_max_num_schedules);
  EXPECT_EQ(list.max_num_schedules, 10u);
  EXPECT_TRUE(list.has_next_schedule);
  EXPECT_TRUE(list.next_schedule);
  EXPECT_FALSE(list.has_show_schedule_complete_state);
}

TEST(ScheduleStateTest, ExtractsEmptyChargeScheduleState) {
  auto response = response_with(CarServer_VehicleData_charge_schedule_state_tag, {});
  ChargeScheduleList list;
  ASSERT_TRUE(extract_charge_schedule_state(response.data(), response.size(), &list));
  EXPECT_TRUE(list.schedules.empty());
  EXPECT_FALSE(list.has_window);
}

TEST(ScheduleStateTest, ExtractsPreconditionSchedules) {
  std::vector<pb_byte_t> state;
  append(state, length_delimited(CarServer_PreconditioningScheduleState_precondition_schedules_tag,
                                 precondition_schedule(1700000100, "Work", 62, 7 * 60 + 45, false)));
  append(state, length_delimited(CarServer_PreconditioningScheduleState_preconditioning_schedule_window_tag,
                                 precondition_schedule(1700000100, "Work", 62, 7 * 60 + 45, false)));
  append(state, varint_field(CarServer_PreconditioningScheduleState_max_num_precondition_schedules_tag, 10));
  auto response = response_with(CarServer_VehicleData_preconditioning_schedule_state_tag, state);

  PreconditionScheduleList list;
  ASSERT_TRUE(extract_preconditioning_schedule_state(response.data(), response.size(), &list));
  ASSERT_EQ(list.schedules.size(), 1u);
  EXPECT_EQ(list.schedules[0].name, "Work");
  EXPECT_EQ(list.schedules[0].precondition_time, 7 * 60 + 45);
  EXPECT_TRUE(list.schedules[0].enabled);
  EXPECT_TRUE(list.has_window);
  EXPECT_EQ(list.window.id, 1700000100u);
  EXPECT_EQ(list.max_num_schedules, 10u);
}

TEST(ScheduleStateTest, TruncatesLongNamesAndListsToBounds) {
  std::vector<pb_byte_t> state;
  const std::string long_name(MAX_SCHEDULE_NAME_LENGTH + 10, 'n');
  for (size_t i = 0; i < MAX_SCHEDULES + 3; ++i) {
    append(state, length_delimited(CarServer_ChargeScheduleState_charge_schedules_tag,
                                   charge_schedule(i + 1, long_name, 127, 60, 120, true)));
  }
  auto response = response_with(CarServer_VehicleData_charge_schedule_state_tag, state);

  ChargeScheduleList list;
  ASSERT_TRUE(extract_charge_schedule_state(response.data(), response.size(), &list));
  EXPECT_EQ(list.schedules.size(), MAX_SCHEDULES);
  EXPECT_TRUE(list.truncated);
  EXPECT_EQ(list.schedules[0].name.size(), MAX_SCHEDULE_NAME_LENGTH);
}

TEST(ScheduleStateTest, RejectsMissingOrMalformed) {
  ChargeScheduleList charge;
  PreconditionScheduleList precondition;
  EXPECT_FALSE(extract_charge_schedule_state(nullptr, 0, &charge));
  EXPECT_FALSE(extract_preconditioning_schedule_state(nullptr, 0, &precondition));

  // A charge schedule response is not a preconditioning response and vice versa
  auto response = response_with(CarServer_VehicleData_charge_schedule_state_tag, {});
  EXPECT_FALSE(extract_preconditioning_schedule_state(response.data(), response.size(), &precondition));

  std::vector<pb_byte_t> state =
      length_delimited(CarServer_ChargeScheduleState_charge_schedules_tag, charge_schedule(1, "Cut", 1, 60, 120, true));
  auto full = response_with(CarServer_VehicleData_charge_schedule_state_tag, state);
  for (size_t cut = 0; cut < full.size(); ++cut) {
    // Never reads past the end; partial input yields false or a partial-free result
    extract_charge_schedule_state(full.data(), cut, &charge);
  }
  // Corrupt the inner schedule length so the list item overruns its parent
  auto corrupt = full;
  corrupt[corrupt.size() - state.size() + 1] = 0x7F;
  EXPECT_FALSE(extract_charge_schedule_state(corrupt.data(), corrupt.size(), &charge));
}
