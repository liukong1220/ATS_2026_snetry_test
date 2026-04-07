 
#ifndef RMOSS_BASE__UDP_TRANSPORTER_HPP_
#define RMOSS_BASE__UDP_TRANSPORTER_HPP_

#include <netinet/in.h>
#include <string>
#include "rmoss_base/transporter_interface.hpp"

namespace rmoss_base
{

// UDP传输模块
class UdpTransporter : public TransporterInterface
{
public:
  UdpTransporter() = delete;
  UdpTransporter(uint16_t port, uint16_t target_port, const std::string & target_ip = "127.0.0.1")
  : port_(port), target_ip_(target_ip), target_port_(target_port) {}
  ~UdpTransporter()
  {
    if (is_open_) {close();}
  }
  bool open() override;
  void close() override;
  bool is_open() override;
  int read(void * buffer, size_t len) override;
  int write(const void * buffer, size_t len) override;
  std::string error_message() override {return error_message_;}

private:
  // 设备信息 (本身设备端口,目标设备ip与端口)
  uint16_t port_;
  std::string target_ip_;
  uint16_t target_port_;
  // 设备文件描述符
  int sock_recv_fd_{-1};
  int sock_send_fd_{-1};
  struct sockaddr_in to_addr_;
  socklen_t to_addr_len_;
  // 设备状态
  bool is_open_{false};
  std::string error_message_;
};

}  // namespace rmoss_base

#endif  // RMOSS_BASE__UDP_TRANSPORTER_HPP_
