#include <cv_bridge/cv_bridge.h>

#include <functional>
#include <opencv2/opencv.hpp>
#include <tuple>

#include "nodelet/nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "ros/node_handle.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"

// Inludes common necessary includes for development using depthai library
#include <depthai/depthai.hpp>

namespace oak_ffc_4p {

class QuadcamNodelet : public nodelet::Nodelet {
  std::unique_ptr<dai::Device> _dev;

 public:
  struct FFC4PConfig {
    dai::CameraBoardSocket socket;
    dai::ColorCameraProperties::SensorResolution resolution =
        dai::ColorCameraProperties::SensorResolution::THE_720_P;
    std::string stream_name;
    bool is_master;
    FFC4PConfig(dai::CameraBoardSocket cbs,
                dai::ColorCameraProperties::SensorResolution res,
                std::string name, bool master)
        : socket(cbs), resolution(res), stream_name(name), is_master(master){};
  };

  std::vector<FFC4PConfig> CameraList = {
      {dai::CameraBoardSocket::CAM_A,
       dai::ColorCameraProperties::SensorResolution::THE_720_P,
       std::string("cam_a"), true},
      {dai::CameraBoardSocket::CAM_B,
       dai::ColorCameraProperties::SensorResolution::THE_720_P,
       std::string("cam_b"), false},
      {dai::CameraBoardSocket::CAM_C,
       dai::ColorCameraProperties::SensorResolution::THE_720_P,
       std::string("cam_c"), false},
      {dai::CameraBoardSocket::CAM_D,
       dai::ColorCameraProperties::SensorResolution::THE_720_P,
       std::string("cam_d"), false}};

  virtual void onInit() override {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    useCompress_ = true;
    enableCal_ = false;

    int badParams = 0;
    badParams += !pnh_.getParam("useCompress", useCompress_);
    badParams += !pnh_.getParam("enableCal", enableCal_);

    if (badParams > 0) {
      std::cout << " Bad parameters -> " << badParams << std::endl;
      throw std::runtime_error("Couldn't find %d of the parameters");
    }

    auto device_info = dai::Device::getAllAvailableDevices();
    if (device_info.size() != 1) {
      throw std::runtime_error("Multiple devices or no device detected.\n");
    }

    dai::Pipeline pipeline;
    pipeline = createPipeline();
    _dev = std::make_unique<dai::Device>(pipeline, dai::UsbSpeed::SUPER_PLUS);

    std::cout << "=== Connected to " << _dev->getMxId() << " === " << std::endl;
    std::cout << "      >>> Num of cams: " << _dev->getConnectedCameras().size()
              << std::endl;
    std::cout << "      >>> USB speed: " << _dev->getUsbSpeed() << std::endl;

    if (useCompress_) {
      img_pub_ = nh_.advertise<sensor_msgs::CompressedImage>(
          "/ffc_4p/image/compressed", 1);
    } else {
      img_pub_ = nh_.advertise<sensor_msgs::Image>("/ffc_4p/image", 1);
    }
    cam_timer_ =
        nh_.createTimer(ros::Duration(0.07), &QuadcamNodelet::grabImg, this);
  };

  dai::Pipeline createPipeline();

  void grabImg(const ros::TimerEvent&);

  double focusClearness(cv::Mat& img);
  void showImg(cv::Mat& img, std::string topic);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher img_pub_;
  ros::Timer cam_timer_;

  bool enableCal_, useCompress_;
};

void QuadcamNodelet::showImg(cv::Mat& img, std::string topic) {
  if (img.empty()) {
    ROS_WARN("input is empty\n");
    return;
  }

  double clearness = focusClearness(img);
  std::stringstream info;
  info << topic << " clearness: " << clearness;
  cv::putText(img, info.str(), cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN, 1.5,
              cv::Scalar(255, 255, 0));
  cv::imshow(topic, img);
  cv::waitKey(1);
}

double QuadcamNodelet::focusClearness(cv::Mat& img) {
  if (img.empty()) {
    ROS_WARN("input is empty\n");
    return 0.0f;
  }

  cv::Mat gray, imgSobel;
  cv::Rect2d roi(img.cols / 3, img.rows / 3, img.cols / 3, img.rows / 3);
  cv::rectangle(img, roi, cv::Scalar(255, 0, 0), 1);
  cv::cvtColor(img(roi), gray, cv::COLOR_BGR2GRAY);
  cv::Sobel(gray, imgSobel, CV_16U, 1, 1);
  return cv::mean(imgSobel)[0];
}

void QuadcamNodelet::grabImg(const ros::TimerEvent&) {
  static cv_bridge::CvImage assemble_cv_img;
  static cv::Mat assemble_cv_mat = cv::Mat::zeros(720, 5120, CV_8UC3);
  static auto const camQueue = _dev->getOutputQueue("xOut", 1, false);

  auto msgGrp = camQueue->get<dai::MessageGroup>();
  if (msgGrp == nullptr) {
    ROS_WARN("Queue is empty\n");
    return;
  }

  assemble_cv_img.header.stamp = ros::Time::now();
  assemble_cv_img.header.frame_id = "depthai";
  assemble_cv_img.encoding = "bgr8";
  assemble_cv_img.image = assemble_cv_mat;

  int alloc_pos = 0;
  for (const auto& camera : CameraList) {
    auto packet = msgGrp->get<dai::ImgFrame>(camera.stream_name);
    if (packet) {
      cv::Mat img = packet->getCvFrame();
      // flip image upside down
      cv::flip(img, img, -1);

      if (enableCal_) {
        double clearness = focusClearness(img);
        std::stringstream info;
        info << camera.stream_name << " clearness: " << clearness;
        cv::putText(img, info.str(), cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN,
                    1.5, cv::Scalar(255, 255, 0));
      }
      // allocate image
      img.copyTo(assemble_cv_img.image(cv::Rect(alloc_pos, 0, 1280, 720)));
      alloc_pos += 1280;
    } else {
      ROS_WARN("Failed to get %s frame\n", camera.stream_name.c_str());
      return;
    }
  }

  if (useCompress_) {
    img_pub_.publish(assemble_cv_img.toCompressedImageMsg());
  } else {
    img_pub_.publish(assemble_cv_img.toImageMsg());
  }
}

dai::Pipeline QuadcamNodelet::createPipeline() {
  dai::Pipeline pipeline;
  pipeline.setXLinkChunkSize(0);

  auto sync = pipeline.create<dai::node::Sync>();
  auto xout = pipeline.create<dai::node::XLinkOut>();
  xout->setStreamName("xOut");
  sync->out.link(xout->input);

  // CAM A
  auto cam_a = pipeline.create<dai::node::ColorCamera>();
  cam_a->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
  cam_a->setBoardSocket(dai::CameraBoardSocket::CAM_A);
  cam_a->setInterleaved(false);
  cam_a->setFps(30);
  cam_a->initialControl.setFrameSyncMode(
      dai::CameraControl::FrameSyncMode::OUTPUT);
  cam_a->initialControl.setManualExposure(20000, 200);
  cam_a->initialControl.setManualWhiteBalance(3000);

  cam_a->isp.link(sync->inputs["cam_a"]);

  // CAM B
  auto cam_b = pipeline.create<dai::node::ColorCamera>();
  cam_b->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
  cam_b->setBoardSocket(dai::CameraBoardSocket::CAM_B);
  cam_b->setInterleaved(false);
  cam_b->setFps(30);
  cam_b->initialControl.setFrameSyncMode(
      dai::CameraControl::FrameSyncMode::INPUT);
  cam_b->initialControl.setManualExposure(20000, 200);
  cam_b->initialControl.setManualWhiteBalance(3000);

  cam_b->isp.link(sync->inputs["cam_b"]);

  // CAM C
  auto cam_c = pipeline.create<dai::node::ColorCamera>();
  cam_c->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
  cam_c->setBoardSocket(dai::CameraBoardSocket::CAM_C);
  cam_c->setInterleaved(false);
  cam_c->setFps(30);
  cam_c->initialControl.setFrameSyncMode(
      dai::CameraControl::FrameSyncMode::INPUT);
  cam_c->initialControl.setManualExposure(20000, 200);
  cam_c->initialControl.setManualWhiteBalance(3000);

  cam_c->isp.link(sync->inputs["cam_c"]);

  // CAM D
  auto cam_d = pipeline.create<dai::node::ColorCamera>();
  cam_d->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
  cam_d->setBoardSocket(dai::CameraBoardSocket::CAM_D);
  cam_d->setInterleaved(false);
  cam_d->setFps(30);
  cam_d->initialControl.setFrameSyncMode(
      dai::CameraControl::FrameSyncMode::INPUT);
  cam_d->initialControl.setManualExposure(20000, 200);
  cam_d->initialControl.setManualWhiteBalance(3000);

  cam_d->isp.link(sync->inputs["cam_d"]);

  // tie the FSIN signals of A+D and B+C pairs, setting the GPIO:
  // OAK-FFC-4P requires driving MXI06 high (FSIN_MODE_SELECT) to connect
  // together the A+D FSIN group (4-lane pair) with the B+C group (2-lane pari)
  auto cfg = pipeline.getBoardConfig();
  cfg.gpio[6] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::OUTPUT,
                                       dai::BoardConfig::GPIO::Level::HIGH);
  pipeline.setBoardConfig(cfg);

  return pipeline;
}

PLUGINLIB_EXPORT_CLASS(oak_ffc_4p::QuadcamNodelet, nodelet::Nodelet)
}  // namespace oak_ffc_4p
