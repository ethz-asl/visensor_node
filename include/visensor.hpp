/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef VISENSOR_H_
#define VISENSOR_H_

#include <string>
#include <map>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/fill_image.h>

#include <visensor_node/DriverConfig.h>
#include "visensor_node/visensor_imu.h"
#include "visensor_node/visensor_time_host.h"
#include "visensor_node/visensor_calibration_service.h"
#include "visensor_node/visensor_calibration.h"

#include <visensor/visensor.hpp>

namespace visensor {

static const std::string CAMERA_FRAME_NAME = "camera";
static const std::string ROS_TOPIC_NAME = "viensor/";

static const std::map<SensorId::SensorId, std::string> ROS_CAMERA_NAMES {
  { SensorId::CAM0, "cam0" },
  { SensorId::CAM1, "cam1" },
  { SensorId::CAM2, "cam2" },
  { SensorId::CAM3, "cam3" },
  { SensorId::FLIR0, "tau0" },
  { SensorId::FLIR1, "tau1" },
  { SensorId::FLIR2, "tau2" },
  { SensorId::FLIR3, "tau3" } };
static const std::map<SensorId::SensorId, std::string> ROS_CAMERA_FRAME_NAMES {
  { SensorId::CAM0, "cam0" },
  { SensorId::CAM1, "cam0" },
  { SensorId::CAM2, "cam0" },
  { SensorId::CAM3, "cam0" },
  { SensorId::FLIR0, "tau0" },
  { SensorId::FLIR1, "tau1" },
  { SensorId::FLIR2, "tau2" },
  { SensorId::FLIR3, "tau3" } };
static const std::map<SensorId::SensorId, std::string> ROS_IMU_NAMES {
  { SensorId::IMU0, "imu0" },
  { SensorId::IMU_CAM0, "mpu0" },
  { SensorId::IMU_CAM1, "mpu1" },
  { SensorId::IMU_CAM2, "mpu2" },
  { SensorId::IMU_CAM3, "tau0" } };
static const std::map<SensorId::SensorId, std::string> ROS_IMU_FRAME_NAMES {
  { SensorId::IMU0, "imu0" },
  { SensorId::IMU_CAM0, "mpu0" },
  { SensorId::IMU_CAM1, "mpu1" },
  { SensorId::IMU_CAM2, "mpu2" },
  { SensorId::IMU_CAM3, "tau0" } };

class ViSensor {
 public:
  ViSensor(ros::NodeHandle& nh);
  ~ViSensor();

  void startSensors(const unsigned int cam_rate, const unsigned int imu_rate);

  //sensor callbacks
  void imuCallback(boost::shared_ptr<ViImuMsg> imu_ptr, ViErrorCode error);
  void frameCallback(ViFrame::Ptr frame_ptr, ViErrorCode error);

  bool calibrationServiceCallback(visensor_node::visensor_calibration_service::Request &req,
                                  visensor_node::visensor_calibration_service::Response &res);
  //dynamic reconfigure callback
  void configCallback(visensor_node::DriverConfig &config, uint32_t level);

 private:
  void init();
  bool getRosCameraConfig(const SensorId::SensorId& camera_id, sensor_msgs::CameraInfo& cam_info);
  bool getRosStereoCameraConfig(const SensorId::SensorId& camera_id_0, sensor_msgs::CameraInfo& cam_info_0,
                                const SensorId::SensorId& camera_id_1, sensor_msgs::CameraInfo& cam_info_1);
  bool precacheViCalibration(const SensorId::SensorId& camera_id);

 private:
  ros::NodeHandle nh_;

  std::map<SensorId::SensorId, ros::NodeHandle> nhc_;
  std::map<SensorId::SensorId, image_transport::ImageTransport> itc_;
  std::map<SensorId::SensorId, image_transport::CameraPublisher> image_pub_;
  std::map<SensorId::SensorId, sensor_msgs::CameraInfo> cinfo_;

  std::map<SensorId::SensorId, ros::Publisher> imu_pub_;
  std::map<SensorId::SensorId, ros::Publisher> imu_custom_pub_;
  std::map<SensorId::SensorId, ros::Publisher> calibration_pub_;

  ros::Publisher pub_time_host_;
  ros::ServiceServer calibration_service_;

  ViSensorDriver drv_;

  std::vector<SensorId::SensorId> list_of_available_sensors_;
  std::vector<SensorId::SensorId> list_of_camera_ids_;
  std::vector<SensorId::SensorId> list_of_imu_ids_;

  std::map<std::string, visensor_node::visensor_calibration> camera_imu_calibrations_;

  dynamic_reconfigure::Server<visensor_node::DriverConfig> dr_srv_;

  visensor_node::DriverConfig config_;
};

}  //namespace visensor

#endif /* VISENSOR_H_ */
