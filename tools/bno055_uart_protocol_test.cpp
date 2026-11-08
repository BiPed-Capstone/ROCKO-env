// Host-side self-test for rocko_env/Bno055UartProtocol.hpp (the BNO055 UART framing for finding
// C-1). No ROS, no serial port, no sensor: it runs on a laptop or in the container.
//
// Build and run (from the repo root; one line, because a backslash ending a // comment line makes
// GCC warn under -Wall):
//   g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sensors/icm20948/include tools/bno055_uart_protocol_test.cpp -o /tmp/bno055_uart_protocol_test && /tmp/bno055_uart_protocol_test
// Exit status 0 and the line "all Bno055UartProtocol tests passed" mean success. Each failed check
// prints "FAIL <line>: <expression>" and the run exits 1 at the end.
//
// What a pass proves: the header builds and parses frames, and decodes the 12-byte burst, the way
// its own comments say. What it cannot prove: that those comments match the BNO055 datasheet (they
// are marked UNVERIFIED), anything about timing, termios, the serial port, the real sensor, which
// axis is the robot's pitch, or the robot. No plugin uses the header yet.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "rocko_env/Bno055UartProtocol.hpp"

namespace u = rocko_env::bno055_uart;

namespace
{
int g_failures = 0;
int g_checks = 0;

#define CHECK(expr) \
  do { \
    ++g_checks; \
    if (!(expr)) { \
      ++g_failures; \
      std::printf("FAIL %d: %s\n", __LINE__, #expr); \
    } \
  } while (0)

// Little-endian bytes of a raw int16 count, as the sensor sends them.
void put_le(std::vector<std::uint8_t> & out, int raw)
{
  const unsigned bits = static_cast<unsigned>(raw) & 0xFFFFu;
  out.push_back(static_cast<std::uint8_t>(bits & 0xFFu));
  out.push_back(static_cast<std::uint8_t>(bits >> 8));
}

// A good reply to the burst read: 0xBB 0x0C, then gyro X, Y, Z and heading, roll, pitch as raw
// counts (16 per deg/s, 16 per deg). The values are chosen so the low and high byte always differ,
// which a byte-order mistake cannot survive.
std::vector<std::uint8_t> good_burst_reply(
  int gx, int gy, int gz, int heading, int roll, int pitch)
{
  std::vector<std::uint8_t> f = {u::kReadOkHeader, u::kBurstLength};
  for (int raw : {gx, gy, gz, heading, roll, pitch}) {
    put_le(f, raw);
  }
  return f;
}

// The raw counts used by the good-reply tests, and what they must decode to (all exact in binary).
constexpr int kGx = 24;       // +1.5 deg/s       bytes 0x18 0x00
constexpr int kGy = -33;      // -2.0625 deg/s    bytes 0xDF 0xFF
constexpr int kGz = -32768;   // -2048 deg/s      bytes 0x00 0x80 (most negative int16)
constexpr int kHeading = 5759;  // 359.9375 deg   bytes 0x7F 0x16
constexpr int kRoll = -1440;  // -90.0 deg        bytes 0x60 0xFA
constexpr int kPitch = -196;  // -12.25 deg       bytes 0x3C 0xFF

bool all_nan_and_invalid(const u::GyroEulerSnapshot & s)
{
  return !s.valid && std::isnan(s.gyro_x_deg_s) && std::isnan(s.gyro_y_deg_s) &&
         std::isnan(s.gyro_z_deg_s) && std::isnan(s.heading_deg) && std::isnan(s.roll_deg) &&
         std::isnan(s.pitch_deg);
}

void test_read_request_bytes()
{
  std::array<std::uint8_t, u::kReadRequestSize> req{};
  CHECK(u::make_read_request(u::kBurstFirstRegister, u::kBurstLength, req));
  CHECK(req[0] == 0xAA && req[1] == 0x01 && req[2] == 0x14 && req[3] == 0x0C);

  std::array<std::uint8_t, u::kReadRequestSize> untouched = {9, 9, 9, 9};
  CHECK(!u::make_read_request(0x14, 0, untouched));  // a read of 0 bytes is refused
  CHECK(untouched[0] == 9 && untouched[3] == 9);
}

void test_write_request_bytes()
{
  const std::uint8_t value[] = {0x08};  // one data byte; its meaning is irrelevant here
  std::uint8_t out[8] = {0};
  CHECK(u::make_write_request(0x3D, value, 1, out, sizeof(out)) == 5);
  CHECK(out[0] == 0xAA && out[1] == 0x00 && out[2] == 0x3D && out[3] == 0x01 && out[4] == 0x08);

  const std::uint8_t two[] = {0x12, 0x34};
  std::uint8_t small[5] = {7, 7, 7, 7, 7};
  CHECK(u::make_write_request(0x55, two, 2, small, sizeof(small)) == 0);  // needs 6 bytes
  CHECK(small[0] == 7);                                                  // nothing written
  CHECK(u::make_write_request(0x55, two, 0, out, sizeof(out)) == 0);
  CHECK(u::make_write_request(0x55, nullptr, 1, out, sizeof(out)) == 0);
}

void test_good_reply_decodes_to_known_values()
{
  const auto f = good_burst_reply(kGx, kGy, kGz, kHeading, kRoll, kPitch);
  CHECK(f.size() == 14);
  CHECK(f[2] == 0x18 && f[3] == 0x00);  // low byte first
  CHECK(f[4] == 0xDF && f[5] == 0xFF);

  const u::BurstResult r = u::parse_gyro_euler_burst_reply(f.data(), f.size());
  CHECK(r.status == u::ReplyStatus::kOk);
  CHECK(r.device_status == 0);
  CHECK(r.snapshot.valid);
  CHECK(r.snapshot.gyro_x_deg_s == 1.5);
  CHECK(r.snapshot.gyro_y_deg_s == -2.0625);
  CHECK(r.snapshot.gyro_z_deg_s == -2048.0);
  CHECK(r.snapshot.heading_deg == 359.9375);
  CHECK(r.snapshot.roll_deg == -90.0);
  CHECK(r.snapshot.pitch_deg == -12.25);
}

void test_positive_and_extreme_values()
{
  // +180 deg pitch = 2880 (0x40 0x0B); +2047.9375 deg/s = 32767 (0xFF 0x7F); -1/16 = -1 (0xFF 0xFF).
  const auto f = good_burst_reply(32767, -1, 1, 0x0102, 16, 2880);
  const u::BurstResult r = u::parse_gyro_euler_burst_reply(f.data(), f.size());
  CHECK(r.status == u::ReplyStatus::kOk);
  CHECK(r.snapshot.gyro_x_deg_s == 2047.9375);
  CHECK(r.snapshot.gyro_y_deg_s == -0.0625);
  CHECK(r.snapshot.gyro_z_deg_s == 0.0625);
  CHECK(r.snapshot.heading_deg == 258.0 / 16.0);  // 0x0102 = 258: catches swapped bytes (0x0201)
  CHECK(r.snapshot.roll_deg == 1.0);
  CHECK(r.snapshot.pitch_deg == 180.0);

  CHECK(u::int16_from_le(0xFF, 0x7F) == 32767);
  CHECK(u::int16_from_le(0x00, 0x80) == -32768);
  CHECK(u::int16_from_le(0xFF, 0xFF) == -1);
  CHECK(u::int16_from_le(0x01, 0x00) == 1);
}

void test_each_device_error_status()
{
  // Every status in the enum, plus one value the enum does not name: all are failed reads.
  const std::uint8_t statuses[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x42};
  for (std::uint8_t st : statuses) {
    const std::uint8_t f[] = {u::kStatusHeader, st};
    const u::BurstResult r = u::parse_gyro_euler_burst_reply(f, sizeof(f));
    CHECK(r.status == u::ReplyStatus::kDeviceError);
    CHECK(r.device_status == st);
    CHECK(all_nan_and_invalid(r.snapshot));
  }
  CHECK(static_cast<int>(u::DeviceStatus::kReadFail) == 0x02);
  CHECK(static_cast<int>(u::DeviceStatus::kReceiveCharacterTimeout) == 0x0A);
}

void test_truncated_and_oversized_frames()
{
  const auto good = good_burst_reply(kGx, kGy, kGz, kHeading, kRoll, kPitch);
  for (std::size_t n = 0; n < good.size(); ++n) {  // every truncation, including empty
    const u::BurstResult r = u::parse_gyro_euler_burst_reply(good.data(), n);
    CHECK(r.status == u::ReplyStatus::kTooShort);
    CHECK(all_nan_and_invalid(r.snapshot));
  }
  auto longer = good;
  longer.push_back(0xAA);  // one byte of the next frame read by mistake
  const u::BurstResult r = u::parse_gyro_euler_burst_reply(longer.data(), longer.size());
  CHECK(r.status == u::ReplyStatus::kTooLong);
  CHECK(all_nan_and_invalid(r.snapshot));

  const std::uint8_t error_plus_one[] = {u::kStatusHeader, 0x02, 0x00};
  const u::BurstResult e = u::parse_gyro_euler_burst_reply(error_plus_one, 3);
  CHECK(e.status == u::ReplyStatus::kTooLong);
  CHECK(all_nan_and_invalid(e.snapshot));

  const u::BurstResult null_buf = u::parse_gyro_euler_burst_reply(nullptr, 14);
  CHECK(null_buf.status == u::ReplyStatus::kTooShort);
  CHECK(all_nan_and_invalid(null_buf.snapshot));
}

void test_wrong_header()
{
  auto f = good_burst_reply(kGx, kGy, kGz, kHeading, kRoll, kPitch);
  for (std::uint8_t first : {std::uint8_t{0x00}, std::uint8_t{0xAA}, std::uint8_t{0xBC},
      std::uint8_t{0xFF}})
  {
    f[0] = first;
    const u::BurstResult r = u::parse_gyro_euler_burst_reply(f.data(), f.size());
    CHECK(r.status == u::ReplyStatus::kBadHeader);
    CHECK(all_nan_and_invalid(r.snapshot));
  }
  // Exhaustive: of all 256 first bytes on an otherwise good frame, only 0xBB is accepted.
  int accepted = 0;
  for (int first = 0; first < 256; ++first) {
    f[0] = static_cast<std::uint8_t>(first);
    const u::BurstResult r = u::parse_gyro_euler_burst_reply(f.data(), f.size());
    if (r.status == u::ReplyStatus::kOk) {
      ++accepted;
      CHECK(first == 0xBB);
    } else {
      CHECK(all_nan_and_invalid(r.snapshot));
    }
  }
  CHECK(accepted == 1);
}

void test_length_mismatch()
{
  // The sensor says 11 or 13 bytes follow when 12 were asked for: refused even if the byte count
  // agrees with the length byte.
  for (std::uint8_t said : {std::uint8_t{11}, std::uint8_t{13}, std::uint8_t{0}}) {
    std::vector<std::uint8_t> f = {u::kReadOkHeader, said};
    f.resize(2 + said, 0x11);
    const u::BurstResult r = u::parse_gyro_euler_burst_reply(f.data(), f.size());
    CHECK(r.status == u::ReplyStatus::kLengthMismatch);
    CHECK(all_nan_and_invalid(r.snapshot));
  }
}

void test_reply_frame_size()
{
  CHECK(u::reply_frame_size(0xBB, 12) == 14);
  CHECK(u::reply_frame_size(0xEE, 12) == 2);
  CHECK(u::reply_frame_size(0xAA, 12) == 0);
  CHECK(u::reply_frame_size(0x00, 12) == 0);
}

void test_decode_needs_exactly_twelve_bytes()
{
  const std::uint8_t bytes[13] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
  CHECK(all_nan_and_invalid(u::decode_gyro_euler(bytes, 11)));
  CHECK(all_nan_and_invalid(u::decode_gyro_euler(bytes, 13)));
  CHECK(all_nan_and_invalid(u::decode_gyro_euler(nullptr, 12)));
  CHECK(u::decode_gyro_euler(bytes, 12).valid);
  CHECK(all_nan_and_invalid(u::GyroEulerSnapshot{}));  // the default is NaN, never 0.0
}

void test_write_reply()
{
  const std::uint8_t ok[] = {0xEE, 0x01};
  CHECK(u::parse_write_reply(ok, 2).status == u::ReplyStatus::kOk);
  const std::uint8_t fail[] = {0xEE, 0x03};
  const u::ReadReply f = u::parse_write_reply(fail, 2);
  CHECK(f.status == u::ReplyStatus::kDeviceError && f.device_status == 0x03);
  const std::uint8_t bad[] = {0xBB, 0x01};
  CHECK(u::parse_write_reply(bad, 2).status == u::ReplyStatus::kBadHeader);
  CHECK(u::parse_write_reply(ok, 1).status == u::ReplyStatus::kTooShort);
  CHECK(u::parse_write_reply(ok, 0).status == u::ReplyStatus::kTooShort);
  const std::uint8_t longer[] = {0xEE, 0x01, 0x00};
  CHECK(u::parse_write_reply(longer, 3).status == u::ReplyStatus::kTooLong);
}
}  // namespace

int main()
{
  test_read_request_bytes();
  test_write_request_bytes();
  test_good_reply_decodes_to_known_values();
  test_positive_and_extreme_values();
  test_each_device_error_status();
  test_truncated_and_oversized_frames();
  test_wrong_header();
  test_length_mismatch();
  test_reply_frame_size();
  test_decode_needs_exactly_twelve_bytes();
  test_write_reply();
  if (g_failures != 0) {
    std::printf("%d of %d Bno055UartProtocol checks FAILED\n", g_failures, g_checks);
    return 1;
  }
  std::printf("all Bno055UartProtocol tests passed (%d checks)\n", g_checks);
  return 0;
}
