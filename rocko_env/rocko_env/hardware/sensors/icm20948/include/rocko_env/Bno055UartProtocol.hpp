#ifndef ROCKO_ENV_BNO055_UART_PROTOCOL
#define ROCKO_ENV_BNO055_UART_PROTOCOL

// The byte framing of the BNO055's UART register protocol, and the decoding of one 12-byte read of
// its gyro and Euler registers into ONE snapshot. Finding C-1 (docs/claude-analysis/
// balance-loop-findings.md) moves the IMU read out of a Python node and into the C++ plugin, over
// UART; this header is the part of that work that can be checked without a robot.
//
// STATUS: NOT USED BY ANY PLUGIN. Nothing in the 100 Hz loop includes this file. Only the host test
// tools/bno055_uart_protocol_test.cpp compiles it. Wiring it into ICM20948.cpp needs a Pi, a BNO055
// in UART mode and a wiring change: docs/proposals/0010-c1-imu-uart-and-encoders-in-cpp.md.
//
// What it is: pure functions on byte buffers the caller owns. No I/O, no termios, no blocking, no
// logging, no allocation, standard library only, so it is safe to call from read() once the plugin
// does its own bounded serial I/O, and a laptop can test it.
//
// Where the facts come from. The frame layout, status codes, register addresses and scale factors
// below are as the author of this file remembers them from the Bosch BNO055 datasheet (the UART
// protocol section and the register map) and Adafruit's BNO055 drivers. The datasheet was NOT open
// while this was written, so every one of them is marked UNVERIFIED against the datasheet: check
// each against the datasheet revision you have before wiring this in. The host test checks that the
// code does what this file says, not that this file matches the sensor.
//
// The protocol, in short (UNVERIFIED against datasheet — check):
//   read request:   0xAA 0x01 <register> <length>             host -> sensor
//   read reply OK:  0xBB <length> <length data bytes>          sensor -> host
//   write request:  0xAA 0x00 <register> <length> <data...>   host -> sensor
//   status reply:   0xEE <status>                              sensor -> host; the answer to every
//                                                              write (0x01 = success), and the
//                                                              answer to a read that failed
// Multi-byte values are little-endian (low byte at the lower register), signed two's complement.

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace rocko_env
{
namespace bno055_uart
{

// ---- Frame bytes (UNVERIFIED against datasheet — check) ----
constexpr std::uint8_t kStartByte = 0xAA;      // first byte of every request
constexpr std::uint8_t kWriteCommand = 0x00;   // second byte of a write request
constexpr std::uint8_t kReadCommand = 0x01;    // second byte of a read request
constexpr std::uint8_t kReadOkHeader = 0xBB;   // first byte of a successful read reply
constexpr std::uint8_t kStatusHeader = 0xEE;   // first byte of a status (ack or error) reply

constexpr std::size_t kReadRequestSize = 4;    // [bytes] 0xAA 0x01 reg len
constexpr std::size_t kWriteHeaderSize = 4;    // [bytes] 0xAA 0x00 reg len, then the data
constexpr std::size_t kReplyHeaderSize = 2;    // [bytes] 0xBB len, or 0xEE status (a whole frame)

// The status byte after 0xEE (UNVERIFIED against datasheet — check the table in its UART section).
// Only kWriteSuccess means something worked; a read never gets it on success (a good read starts
// 0xBB). The plugin should count every other value as a failed read.
enum class DeviceStatus : std::uint8_t
{
  kWriteSuccess = 0x01,
  kReadFail = 0x02,
  kWriteFail = 0x03,
  kRegmapInvalidAddress = 0x04,
  kRegmapWriteDisabled = 0x05,
  kWrongStartByte = 0x06,
  kBusOverRunError = 0x07,  // Adafruit's driver retries on this one; whether to retry is the plugin's call
  kMaxLengthError = 0x08,
  kMinLengthError = 0x09,
  kReceiveCharacterTimeout = 0x0A,
};

// ---- Registers (page 0) and units (UNVERIFIED against datasheet — check) ----
// The 12 bytes from 0x14 to 0x1F are three gyro rates then three Euler angles, each an int16:
constexpr std::uint8_t kRegGyroXLsb = 0x14;        // GYR_DATA_X, 0x14-0x15
constexpr std::uint8_t kRegGyroYLsb = 0x16;        // GYR_DATA_Y, 0x16-0x17
constexpr std::uint8_t kRegGyroZLsb = 0x18;        // GYR_DATA_Z, 0x18-0x19
constexpr std::uint8_t kRegEulerHeadingLsb = 0x1A;  // EUL_Heading, 0x1A-0x1B
constexpr std::uint8_t kRegEulerRollLsb = 0x1C;     // EUL_Roll, 0x1C-0x1D
constexpr std::uint8_t kRegEulerPitchLsb = 0x1E;    // EUL_Pitch, 0x1E-0x1F
// The one read per control cycle (findings H-3: one snapshot; H-6: the gyro rate with it).
constexpr std::uint8_t kBurstFirstRegister = kRegGyroXLsb;
constexpr std::uint8_t kBurstLength = 12;  // [bytes] 0x14..0x1F inclusive
static_assert(kRegEulerPitchLsb + 2 - kBurstFirstRegister == kBurstLength,
  "the burst must end exactly after EUL_Pitch's high byte");

// Scale factors with the unit-selection register UNIT_SEL (0x3B) at its reset default, which is
// assumed and not checked here: gyro in deg/s and Euler angles in degrees. If UNIT_SEL selects rad/s
// or radians the factors are different (900 LSB per unit, UNVERIFIED), so the plugin must set or read
// UNIT_SEL at configure time rather than trust the default.
constexpr double kGyroLsbPerDegPerSec = 16.0;  // [LSB / (deg/s)]
constexpr double kEulerLsbPerDeg = 16.0;       // [LSB / deg]

// ---- Results ----

// Why a reply was or was not accepted. Every value except kOk is a failed read.
enum class ReplyStatus
{
  kOk,              // a well-formed reply of the expected kind and length
  kDeviceError,     // the sensor answered 0xEE <status>; the status is in device_status
  kTooShort,        // fewer bytes than the frame needs (a truncated reply, or nothing at all)
  kBadHeader,       // the first byte is neither 0xBB nor 0xEE: garbage, or out of step with the stream
  kLengthMismatch,  // 0xBB, but its length byte is not the length that was asked for
  kTooLong,         // more bytes than the frame holds: the caller read into the next frame, or noise
};

// One sample of the sensor, all six values from the same 12-byte read, in the SENSOR's own axes.
// UNVERIFIED on this robot: which of these is the robot's pitch and pitch rate, and with which sign.
// Today's node exports EUL_Pitch (+7.6 deg offset) as pitch, and gyro axis 0 (X) with sign +1 as the
// pitch rate, both guesses (ICM20948.py, pitch_rate_axis / pitch_rate_sign). Choosing the field, the
// sign and the offset is the plugin's job, not this header's.
// On any failure ALL SIX are NaN and valid is false: never 0.0, which would read as "upright and
// still" (findings C-2, M-5).
struct GyroEulerSnapshot
{
  bool valid = false;
  double gyro_x_deg_s = std::numeric_limits<double>::quiet_NaN();  // [deg/s] about sensor X
  double gyro_y_deg_s = std::numeric_limits<double>::quiet_NaN();  // [deg/s] about sensor Y
  double gyro_z_deg_s = std::numeric_limits<double>::quiet_NaN();  // [deg/s] about sensor Z
  double heading_deg = std::numeric_limits<double>::quiet_NaN();   // [deg] EUL_Heading
  double roll_deg = std::numeric_limits<double>::quiet_NaN();      // [deg] EUL_Roll
  double pitch_deg = std::numeric_limits<double>::quiet_NaN();     // [deg] EUL_Pitch
};

struct ReadReply
{
  ReplyStatus status = ReplyStatus::kTooShort;
  std::uint8_t device_status = 0;     // the byte after 0xEE when status is kDeviceError, else 0
  const std::uint8_t * data = nullptr;  // points INTO the caller's buffer; set only when kOk
  std::size_t length = 0;             // [bytes] of data; set only when kOk
};

struct BurstResult
{
  ReplyStatus status = ReplyStatus::kTooShort;
  std::uint8_t device_status = 0;  // as in ReadReply
  GyroEulerSnapshot snapshot;      // valid, and finite, only when status is kOk
};

// ---- Building requests ----

// The 4 bytes that ask for `length` bytes starting at `register_address`.
// Returns false (and leaves `out` alone) for length 0, which asks for nothing.
inline bool make_read_request(
  std::uint8_t register_address, std::uint8_t length,
  std::array<std::uint8_t, kReadRequestSize> & out)
{
  if (length == 0) {
    return false;
  }
  out = {kStartByte, kReadCommand, register_address, length};
  return true;
}

// Writes the bytes of a write request (header plus `length` data bytes) into `out`, which has room
// for `out_capacity` bytes. Returns the number of bytes to send, or 0 (nothing written) if length is
// 0, data is null, or `out` is too small. For configuration (OPR_MODE, UNIT_SEL) at configure time.
inline std::size_t make_write_request(
  std::uint8_t register_address, const std::uint8_t * data, std::uint8_t length,
  std::uint8_t * out, std::size_t out_capacity)
{
  const std::size_t frame_size = kWriteHeaderSize + length;
  if (length == 0 || data == nullptr || out == nullptr || out_capacity < frame_size) {
    return 0;
  }
  out[0] = kStartByte;
  out[1] = kWriteCommand;
  out[2] = register_address;
  out[3] = length;
  for (std::size_t i = 0; i < length; ++i) {
    out[kWriteHeaderSize + i] = data[i];
  }
  return frame_size;
}

// ---- Parsing replies ----

// How many bytes the whole reply will be, judged from its first byte, so a non-blocking reader knows
// when to stop: 2 for a status reply (0xEE), 2 + expected_length for a read reply (0xBB), and 0 for
// any other first byte (a bad header: stop and count a failed read).
inline std::size_t reply_frame_size(std::uint8_t first_byte, std::uint8_t expected_length)
{
  if (first_byte == kStatusHeader) {
    return kReplyHeaderSize;
  }
  if (first_byte == kReadOkHeader) {
    return kReplyHeaderSize + expected_length;
  }
  return 0;
}

// Checks the reply to a read of `expected_length` bytes. `buffer` holds the `size` bytes received.
// The checks run in this order: empty; bad header; a status reply (exactly 2 bytes); the length byte;
// the byte count.
inline ReadReply parse_read_reply(
  const std::uint8_t * buffer, std::size_t size, std::uint8_t expected_length)
{
  ReadReply reply;
  if (buffer == nullptr || size == 0) {
    reply.status = ReplyStatus::kTooShort;
    return reply;
  }
  const std::size_t frame_size = reply_frame_size(buffer[0], expected_length);
  if (frame_size == 0) {
    reply.status = ReplyStatus::kBadHeader;
    return reply;
  }
  if (size < kReplyHeaderSize) {  // even a status reply needs its second byte
    reply.status = ReplyStatus::kTooShort;
    return reply;
  }
  if (buffer[0] == kStatusHeader) {
    reply.status = (size == kReplyHeaderSize) ? ReplyStatus::kDeviceError : ReplyStatus::kTooLong;
    reply.device_status = buffer[1];
    return reply;
  }
  // 0xBB: the length byte must be what we asked for, then the byte count must match it.
  if (buffer[1] != expected_length) {
    reply.status = ReplyStatus::kLengthMismatch;
    return reply;
  }
  if (size < frame_size) {
    reply.status = ReplyStatus::kTooShort;
    return reply;
  }
  if (size > frame_size) {
    reply.status = ReplyStatus::kTooLong;
    return reply;
  }
  reply.status = ReplyStatus::kOk;
  reply.data = buffer + kReplyHeaderSize;
  reply.length = expected_length;
  return reply;
}

// Checks the reply to a write. Only 0xEE 0x01 (kWriteSuccess) is kOk; any other status is
// kDeviceError with device_status set; anything else fails as in parse_read_reply.
inline ReadReply parse_write_reply(const std::uint8_t * buffer, std::size_t size)
{
  ReadReply reply;
  if (buffer == nullptr || size == 0) {
    reply.status = ReplyStatus::kTooShort;
    return reply;
  }
  if (buffer[0] != kStatusHeader) {
    reply.status = ReplyStatus::kBadHeader;
    return reply;
  }
  if (size < kReplyHeaderSize) {
    reply.status = ReplyStatus::kTooShort;
    return reply;
  }
  if (size > kReplyHeaderSize) {
    reply.status = ReplyStatus::kTooLong;
    return reply;
  }
  reply.device_status = buffer[1];
  reply.status = (buffer[1] == static_cast<std::uint8_t>(DeviceStatus::kWriteSuccess)) ?
    ReplyStatus::kOk : ReplyStatus::kDeviceError;
  return reply;
}

// ---- Decoding ----

// A signed 16-bit value stored low byte first. Done with int arithmetic rather than a cast to
// int16_t, whose result for values above 32767 is implementation-defined before C++20.
inline int int16_from_le(std::uint8_t low, std::uint8_t high)
{
  const int raw = static_cast<int>(low) | (static_cast<int>(high) << 8);  // 0 .. 65535
  return (raw >= 32768) ? raw - 65536 : raw;                             // -32768 .. 32767
}

// The raw int16 whose low byte is at `register_lsb`, out of the 12 burst bytes in `data`. Its offset
// inside the burst is its register minus the first register (0x14). Caller checks the size.
inline int burst_value(const std::uint8_t * data, std::uint8_t register_lsb)
{
  const std::size_t i = register_lsb - kBurstFirstRegister;
  return int16_from_le(data[i], data[i + 1]);
}

// Decodes the 12 data bytes of a read starting at kBurstFirstRegister. `size` must be exactly
// kBurstLength; anything else gives the all-NaN, invalid snapshot.
inline GyroEulerSnapshot decode_gyro_euler(const std::uint8_t * data, std::size_t size)
{
  GyroEulerSnapshot s;  // all NaN, valid = false
  if (data == nullptr || size != kBurstLength) {
    return s;
  }
  s.gyro_x_deg_s = burst_value(data, kRegGyroXLsb) / kGyroLsbPerDegPerSec;
  s.gyro_y_deg_s = burst_value(data, kRegGyroYLsb) / kGyroLsbPerDegPerSec;
  s.gyro_z_deg_s = burst_value(data, kRegGyroZLsb) / kGyroLsbPerDegPerSec;
  s.heading_deg = burst_value(data, kRegEulerHeadingLsb) / kEulerLsbPerDeg;
  s.roll_deg = burst_value(data, kRegEulerRollLsb) / kEulerLsbPerDeg;
  s.pitch_deg = burst_value(data, kRegEulerPitchLsb) / kEulerLsbPerDeg;
  s.valid = true;
  return s;
}

// The whole step for one control cycle: check the reply to the burst read (0xAA 0x01 0x14 0x0C) and,
// only if it is well formed, decode it. On any failure the snapshot stays all NaN and invalid.
// This does NOT judge whether the numbers are plausible (range limits, a frozen sensor, finding
// H-9): that stays in the plugin, which already rejects |pitch| or |roll| above 180 deg.
inline BurstResult parse_gyro_euler_burst_reply(const std::uint8_t * buffer, std::size_t size)
{
  BurstResult result;
  const ReadReply reply = parse_read_reply(buffer, size, kBurstLength);
  result.status = reply.status;
  result.device_status = reply.device_status;
  if (reply.status == ReplyStatus::kOk) {
    result.snapshot = decode_gyro_euler(reply.data, reply.length);
  }
  return result;
}

}  // namespace bno055_uart
}  // namespace rocko_env

#endif  // ROCKO_ENV_BNO055_UART_PROTOCOL
