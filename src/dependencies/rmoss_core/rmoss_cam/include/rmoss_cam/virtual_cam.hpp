 
#ifndef RMOSS_CAM__VIRTUAL_CAM_HPP_
#define RMOSS_CAM__VIRTUAL_CAM_HPP_

#include <string>
#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "rmoss_cam/cam_interface.hpp"

namespace rmoss_cam
{
// the virtual camera device by using image or video, based on opencv
class VirtualCam : public CamInterface
{
public:
  enum { IMAGE_MODE, VIDEO_MODE};
  explicit VirtualCam(int mode, const std::string & path);

  bool open() override;
  bool close() override;
  bool is_open() override;
  bool grab_image(cv::Mat & image) override;
  bool set_parameter(CamParamType type, int value) override;
  bool get_parameter(CamParamType type, int & value) override;
  std::string error_message() override {return error_message_;}

private:
  // for image
  cv::Mat img_;
  // for video
  cv::VideoCapture cap_;
  int total_frames_;
  int current_frame;
  // camera parameters
  std::unordered_map<CamParamType, int> params_;
  std::string init_error_message_;
  std::string error_message_;
  // flag
  bool init_ok_{false};
  bool is_open_{false};
  int current_mode_;
};

}  // namespace rmoss_cam

#endif  // RMOSS_CAM__VIRTUAL_CAM_HPP_
