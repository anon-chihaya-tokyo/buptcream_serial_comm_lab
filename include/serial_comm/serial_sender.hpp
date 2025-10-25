#include "serial_comm/serial_sender.hpp"

#include <spdlog/spdlog.h>

#include <chrono>
#include <cstring>
#include <thread>
#include<bits/stdc++.h>
#include "CLI11/CLI11.hpp"

using namespace std::chrono_literals;
using namespace serial_comm;

SenderArgs::SenderArgs() : send_frequency(100), serial_port_name("/dev/ttyUSB0") {
}

template <typename T>
inline void put_field(uint8_t *buf_ptr, size_t &offset, T field) {
  std::memcpy(buf_ptr + offset, &field, sizeof(T));
  offset += sizeof(T);
}

SerialSender::SerialSender(const SenderArgs &args)
    : ioc_(),
      serial_(ioc_, args.serial_port_name),
      msg_(),
      buffer_(),
      rd_(),
      gen_(rd_()),
      dist_(0.0, 1.0),
      crc16_(),
      send_period_sec_(1.0 / std::max<uint32_t>(args.send_frequency, 1)) {
  serial_.set_option(boost::asio::serial_port::baud_rate(115200));
  serial_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
  serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
  serial_.set_option(boost::asio::serial_port::character_size(8));
}

void SerialSender::put_buf() {
  msg_.id += 1;
  msg_.data.quaternion.w = dist_(gen_);
  msg_.data.quaternion.x = dist_(gen_);
  msg_.data.quaternion.y = dist_(gen_);
  msg_.data.quaternion.z = dist_(gen_);
  msg_.data.angular_velocity.x = dist_(gen_);
  msg_.data.angular_velocity.y = dist_(gen_);
  msg_.data.angular_velocity.z = dist_(gen_);
  msg_.data.linear_acceleration.x = dist_(gen_);
  msg_.data.linear_acceleration.y = dist_(gen_);
  msg_.data.linear_acceleration.z = dist_(gen_);

  size_t offset = 0;
  put_field(buffer_.data(), offset, msg_.head);
  put_field(buffer_.data(), offset, msg_.id);
  put_field(buffer_.data(), offset, msg_.data.quaternion.w);
  put_field(buffer_.data(), offset, msg_.data.quaternion.x);
  put_field(buffer_.data(), offset, msg_.data.quaternion.y);
  put_field(buffer_.data(), offset, msg_.data.quaternion.z);
  put_field(buffer_.data(), offset, msg_.data.angular_velocity.x);
  put_field(buffer_.data(), offset, msg_.data.angular_velocity.y);
  put_field(buffer_.data(), offset, msg_.data.angular_velocity.z);
  put_field(buffer_.data(), offset, msg_.data.linear_acceleration.x);
  put_field(buffer_.data(), offset, msg_.data.linear_acceleration.y);
  put_field(buffer_.data(), offset, msg_.data.linear_acceleration.z);

  crc16_.process_bytes(buffer_.data(), CRC_FIELD_SIZE);
  msg_.crc16 = crc16_.checksum();
  crc16_.reset();

  put_field(buffer_.data(), offset, msg_.crc16);
  put_field(buffer_.data(), offset, msg_.tail);
}

void SerialSender::do_send() {
  put_buf();
  async_write(serial_, boost::asio::buffer(buffer_), [this](const boost::system::error_code &ec, std::size_t /*bytes_transferred*/) {
    if (!ec) {
      auto us = static_cast<int64_t>(send_period_sec_ * 1'000'000.0);
      if (us < 1) us = 1;
      std::this_thread::sleep_for(std::chrono::microseconds(us));
      do_send();
    } else {
      spdlog::error(std::string("send error: ") + ec.message());
    }
  });
}

void SerialSender::start() {
  do_send();
  ioc_.run();
}

int main(int argc, char *argv[]) {
  CLI::App app{"serial_sender description"};
  argv = app.ensure_utf8(argv);

  SenderArgs args{};
  app.add_option("-f,--send_frequency", args.send_frequency, "Set send frequency(Hz)");
  app.add_option("-n,--serial_port_name", args.serial_port_name, "Set serial port name");
  CLI11_PARSE(app, argc, argv);

  auto sender = std::make_unique<SerialSender>(args);
  sender->start();
  return 0;
}
