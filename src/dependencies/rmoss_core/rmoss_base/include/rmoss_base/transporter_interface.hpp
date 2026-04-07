 
#ifndef RMOSS_BASE__TRANSPORTER_INTERFACE_HPP_
#define RMOSS_BASE__TRANSPORTER_INTERFACE_HPP_

#include <memory>
#include <string>

namespace rmoss_base
{

// Transporter device interface to transport data between embedded systems (stm32,c51) and PC
class TransporterInterface
{
public:
  using SharedPtr = std::shared_ptr<TransporterInterface>;
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool is_open() = 0;
  // return recv len>0, return <0 if error
  virtual int read(void * buffer, size_t len) = 0;
  // return send len>0, return <0 if error
  virtual int write(const void * buffer, size_t len) = 0;
  // get error message when open() return false.
  virtual std::string error_message() = 0;
};

}  // namespace rmoss_base

#endif  // RMOSS_BASE__TRANSPORTER_INTERFACE_HPP_
