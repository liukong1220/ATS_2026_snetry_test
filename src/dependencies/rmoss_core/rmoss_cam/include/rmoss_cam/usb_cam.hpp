 #ifndef RMOSS_CAM__USB_CAM_HPP_
#define RMOSS_CAM__USB_CAM_HPP_

#include <string>
#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "rmoss_cam/cam_interface.hpp"

namespace rmoss_cam
{
// the usb camera (UVC) device, based on opencv
class UsbCam : public CamInterface
{
public:
  explicit UsbCam(const std::string & dev_path);
  ~UsbCam();

public:
  bool open() override;
  bool close() override;
  bool is_open() override;
  bool grab_image(cv::Mat & image) override;
  bool set_parameter(CamParamType type, int value) override;
  bool get_parameter(CamParamType type, int & value) override;
  std::string error_message() override {return error_message_;}

private:
  std::string dev_path_;
  std::string error_message_;
  cv::VideoCapture cap_;
  // camera parameters
  std::unordered_map<CamParamType, int> params_;
  // flag
  bool is_open_{false};
};

}  // namespace rmoss_cam

#endif  // RMOSS_CAM__USB_CAM_HPP_
