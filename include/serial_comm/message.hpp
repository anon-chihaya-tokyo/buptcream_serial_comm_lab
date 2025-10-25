#ifndef MESSAGE_H
#define MESSAGE_H
#include <cstdint>
#include <type_traits>

namespace serial_comm {
struct Quaternion {
  double w;
  double x;
  double y;
  double z;
};

struct Vector3 {
  double x;
  double y;
  double z;
};

struct ImuMessage {
  Quaternion quaternion;
  Vector3 angular_velocity;
  Vector3 linear_acceleration;
};

constexpr char SERIAL_MSG_HEAD = '+';
constexpr char SERIAL_MSG_TAIL = '*';

struct SerialMessage {
  uint8_t head;
  uint32_t id;
  ImuMessage data;
  uint16_t crc16;
  uint8_t tail;

  SerialMessage() {
    head = SERIAL_MSG_HEAD;
    id = 0;
    data = {};
    crc16 = 0;
    tail = SERIAL_MSG_TAIL;
  }
};

template <typename T>
constexpr std::size_t packed_sizeof() {
  if constexpr (std::is_arithmetic_v<T>) {
    return sizeof(T);
  } else if constexpr (std::is_same_v<T, Quaternion>) {
    return packed_sizeof<double>() * 4;
  } else if constexpr (std::is_same_v<T, Vector3>) {
    return packed_sizeof<double>() * 3;
  } else if constexpr (std::is_same_v<T, ImuMessage>) {
    return packed_sizeof<Quaternion>() + packed_sizeof<Vector3>() * 2;
  } else if constexpr (std::is_same_v<T, SerialMessage>) {
    return packed_sizeof<uint8_t>() + packed_sizeof<uint32_t>() + packed_sizeof<ImuMessage>() + packed_sizeof<uint16_t>()
        + packed_sizeof<uint8_t>();
  }
  return 0;
}

constexpr std::size_t SERIAL_MSG_SIZE = packed_sizeof<SerialMessage>();
constexpr std::size_t CRC_FIELD_SIZE = 85;
}  // namespace serial_comm

#endif /* MESSAGE_H */
