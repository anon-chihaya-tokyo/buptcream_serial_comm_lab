#ifndef SERIAL_SENDER_HPP
#define SERIAL_SENDER_HPP

#include <array>
#include <boost/asio.hpp>
#include <boost/crc.hpp>
#include <random>
#include <string>

#include "serial_comm/message.hpp"

struct SenderArgs {
  uint32_t send_frequency;
  std::string serial_port_name;

  SenderArgs();
};

template <typename T>
inline void put_field(uint8_t *buf_ptr, size_t &offset, T field);

class SerialSender {
  using SerialBuffer = std::array<uint8_t, serial_comm::SERIAL_MSG_SIZE>;

private:
  boost::asio::io_context ioc_;
  boost::asio::serial_port serial_;
  serial_comm::SerialMessage msg_;
  SerialBuffer buffer_;
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> dist_;
  boost::crc_optimal<16, 0x1021, 0x0000, 0x0000, false, false> crc16_;
  double send_period_sec_;

  void put_buf();
  void do_send();

public:
  SerialSender(const SenderArgs &args);
  SerialSender(const SerialSender &) = delete;
  SerialSender &operator=(const SerialSender &) = delete;

  void start();
};

#endif  // SERIAL_SENDER_HPP
