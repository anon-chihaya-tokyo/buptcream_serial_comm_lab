#ifndef SERIAL_RECEIVER_HPP
#define SERIAL_RECEIVER_HPP


#include <functional> 
#include <string>     
#include <vector>     
#include <cstdint>    
#include <memory>     
#include <thread>  
#include<bits/stdc++.h>   

#include <boost/asio/io_context.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/system/error_code.hpp>

#include "serial_comm/message.hpp" 

class SerialReceiver {
public:
  // 定义回调函数类型，当接收到完整消息时调用
  using CallbackType = std::function<void(const serial_comm::SerialMessage&)>;

  /**
   * @brief 构造函数
   * @param port 串口设备路径 (例如 "/dev/ttyUSB0")
   * @throws std::runtime_error 如果串口打开或配置失败
   */
  SerialReceiver(const std::string& port);

  /**
   * @brief 析构函数：安全关闭 IO 线程和串口
   */
  ~SerialReceiver();

  /**
   * @brief 注册用于处理完整消息的回调函数
   * @param callback 当接收到完整 SerialMessage 时调用的函数
   */
  void registerCallback(CallbackType callback);

  /**
   * @brief 启动 IO 线程和异步读取循环
   */
  void start(); 

  /**
   * @brief 状态机：处理接收到的单个字节
   * @param data 从串口读取的字节 (由内部的异步读取循环调用)
   */
  void processIncomingData(uint8_t data);

private:
  /**
   * @brief (私有) 启动一次异步读取操作
   */
  void do_read();

  // --- 状态机 ---
  enum class State
  {
    WaitingForHead,
    ReadingBody
  };
  CallbackType data_callback_;      // 存储回调函数
  State currentState = State::WaitingForHead; // 当前状态
  std::vector<uint8_t> buffer;     // 状态机使用的帧缓冲区

  // --- 3. Boost.asio 和线程成员 ---
  boost::asio::io_context io_ctx_;         // Boost.asio 的核心 IO 上下文
  boost::asio::serial_port serial_port_;     // 串口对象
  std::vector<uint8_t> read_buffer_;     // asio 读取用的单字节缓冲区
  std::thread io_thread_;                // 运行 io_ctx_.run() 的线程
};

#endif  // SERIAL_RECEIVER_HPP

