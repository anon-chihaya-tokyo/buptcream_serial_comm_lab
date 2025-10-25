
#include "serial_comm/serial_receiver.hpp" // 包含类头文件
#include "serial_comm/message.hpp"         // 包含消息结构
#include "spdlog/spdlog.h"                 // 包含日志库

#include <vector>
#include <cstring>                         // 包含 std::memcpy
#include <stdexcept>                       // 包含运行时错误

/**
 * @brief 构造函数
 * @param port 串口设备路径 (例如 "/dev/ttyUSB0")
 */
SerialReceiver::SerialReceiver(const std::string& port)
    
    : io_ctx_(), 
      serial_port_(io_ctx_),
      read_buffer_(1) 
{
    // 初始化状态机
    currentState = State::WaitingForHead;
    
    spdlog::info("SerialReceiver 正在使用端口 {} 进行初始化...", port);

    try {
      
      serial_port_.open(port);
      serial_port_.set_option(boost::asio::serial_port_base::baud_rate(115200));
      serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

      spdlog::info("串口 {} 打开成功, 波特率 115200", port);

    } catch (const boost::system::system_error& e) {
      
      spdlog::error("打开或配置串口 {} 失败: {}", port, e.what());
      
      throw std::runtime_error("无法打开或配置串口: " + std::string(e.what()));
    }
}

/**
 * @brief 析构函数：确保线程安全关闭
 */
SerialReceiver::~SerialReceiver() {
    spdlog::info("SerialReceiver 正在析构...");
    
    
    if (!io_ctx_.stopped()) {
        
        io_ctx_.stop();
    }
    
    
    if (io_thread_.joinable()) {
        io_thread_.join();
        spdlog::info("IO 线程已加入。");
    }

    
    if (serial_port_.is_open()) {
        serial_port_.close();
        spdlog::info("串口已关闭。");
    }
}

/**
 * @brief 注册用于处理完整消息的回调函数
 * @param callback 当接收到完整 SerialMessage 时调用的函数
 */
void SerialReceiver::registerCallback(CallbackType callback) {
    data_callback_ = std::move(callback);
    spdlog::info("SerialReceiver 注册了回调函数。");
}


/**
 * @brief 启动异步读取
 
 */
void SerialReceiver::start() {
    spdlog::info("SerialReceiver 启动中...");

    if (!serial_port_.is_open()) {
        spdlog::error("串口未打开，无法启动读取。");
        return;
    }

    
    io_thread_ = std::thread([this]() {
        spdlog::info("Boost.asio IO 线程已启动。");
        try {
            io_ctx_.run();
        } catch (const std::exception& e) {
            spdlog::error("IO 线程异常: {}", e.what());
        }
        spdlog::info("Boost.asio IO 线程已退出。");
    });

    
    do_read();
}

/**
 * @brief (新增) 私有辅助函数：启动一次异步读取
 */
void SerialReceiver::do_read() {
    // 异步从串口读取一个字节到 read_buffer_ 中
    serial_port_.async_read_some(
        boost::asio::buffer(read_buffer_),
        // [this] 捕获当前对象的 this 指针，以便在 lambda 内部调用成员函数
        [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
            // 这是在 IO 线程中执行的回调函数
            
            // 6. --- 检查错误码 ---
            if (!ec) {
                // --- 成功读取 ---
                if (bytes_transferred > 0) {
                    // 调用您的状态机来处理这个字节
                    processIncomingData(read_buffer_[0]);
                }

                // --- 循环关键 ---
                // 再次调用 do_read() 来启动下一次读取操作，形成循环
                do_read();

            } else {
                // --- 发生错误 ---
                if (ec == boost::asio::error::operation_aborted) {
                    // 操作被取消 (例如关闭串口或析构时)，这是正常的
                    spdlog::warn("串口读取操作被取消。");
                } else {
                    // 报告其他错误 (例如设备断开)
                    spdlog::error("串口读取错误: {}", ec.message());
                }
            }
        });
}


/**
 * @brief 状态机：处理接收到的单个字节
 * (此函数保持不变)
 * @param data 从串口读取的字节
 */
void SerialReceiver::processIncomingData(uint8_t data) {
    switch (currentState) {
        
        case State::WaitingForHead:
            if (data == serial_comm::SERIAL_MSG_HEAD) {
                
                buffer.clear();
                buffer.push_back(data);
                currentState = State::ReadingBody; 
            }
            break;
        
        
        case State::ReadingBody:
            buffer.push_back(data); 
            
            
            if (buffer.size() == serial_comm::SERIAL_MSG_SIZE) {
                
                
                if (data == serial_comm::SERIAL_MSG_TAIL) {
                    
                    
                    serial_comm::SerialMessage msg;
                    
                    
                    std::memcpy(&msg, buffer.data(), serial_comm::SERIAL_MSG_SIZE);
                    
                    
                    if (data_callback_) {
                        data_callback_(msg); 
                    }

                } else {
                    
                    spdlog::warn("接收到错误帧：帧尾不匹配。");
                }
                
                
                currentState = State::WaitingForHead;
                buffer.clear();
                
            } else if (buffer.size() > serial_comm::SERIAL_MSG_SIZE) {
                
                spdlog::warn("缓冲区溢出，重置状态机。");
                currentState = State::WaitingForHead;
                buffer.clear();
            }
            
            break;
    }
}
