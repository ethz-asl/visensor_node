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

#include <Eigen/Dense>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "visensor.hpp"

#define THRESHOLD_DATA_DELAY_WARNING 0.1 // in seconds

namespace visensor {
ViSensor::ViSensor(ros::NodeHandle& nh)
    : nh_(nh) {
  init();
}

ViSensor::~ViSensor() {
}

void ViSensor::init() {
  try {
    drv_.init();
  } catch (visensor::exceptions const &ex) {
    ROS_ERROR("%s", ex.what());
    exit(1);
  }
  list_of_available_sensors_ = drv_.getListOfSensorIDs();
  list_of_camera_ids_ = drv_.getListOfCameraIDs();
  list_of_imu_ids_ = drv_.getListOfImuIDs();

  std::string rootdir = ros::package::getPath("visensor_node");
  std::string tempCameraInfoFileName;

  pub_time_host_ = nh_.advertise<visensor_node::visensor_time_host>("time_host", -1);

  try {
    drv_.setCameraCallback(boost::bind(&ViSensor::frameCallback, this, _1, _2));
    drv_.setImuCallback(boost::bind(&ViSensor::imuCallback, this, _1, _2));
    drv_.setCameraCalibrationSlot(0); // 0 is factory calibration
  } catch (visensor::exceptions const &ex) {
    ROS_WARN("%s", ex.what());
  }

  // initialize cameras
  for (auto camera_id : list_of_camera_ids_) {
    ros::NodeHandle nhc_temp(nh_, ROS_CAMERA_NAMES.at(camera_id));
    nhc_.insert(std::pair<SensorId::SensorId, ros::NodeHandle>(camera_id, nhc_temp));
    image_transport::ImageTransport itc_temp(nhc_[camera_id]);
    itc_.insert(std::pair<SensorId::SensorId, image_transport::ImageTransport>(camera_id, itc_temp));
    ROS_INFO("Register publisher for camera %u with topic name %s", camera_id,
             ROS_CAMERA_NAMES.at(camera_id).c_str());
    image_transport::CameraPublisher image_pub = (itc_.at(camera_id)).advertiseCamera("image_raw", 100);
    image_pub_.insert(std::pair<SensorId::SensorId, image_transport::CameraPublisher>(camera_id, image_pub));

    precacheViCalibration(camera_id);

    sensor_msgs::CameraInfo cinfo_temp;
    if (getRosCameraConfig(camera_id, cinfo_temp))
      ROS_INFO_STREAM("Read calibration for "<< ROS_CAMERA_NAMES.at(camera_id));
    else
      ROS_INFO_STREAM("Could not read calibration for "<< ROS_CAMERA_NAMES.at(camera_id));

    cinfo_.insert(std::pair<SensorId::SensorId, sensor_msgs::CameraInfo>(camera_id, cinfo_temp));

    ros::Publisher temp_pub;
    temp_pub = nhc_temp.advertise<visensor_node::visensor_calibration>("calibration", -1);
    calibration_pub_.insert(std::pair<visensor::SensorId::SensorId, ros::Publisher>(camera_id, temp_pub));
  }


  if (!drv_.isStereoCameraFlipped()){
    //Generate Stereo ROS config, assuming than cam0 and cam1 are in fronto-parallel stereo configuration
    if (getRosStereoCameraConfig(SensorId::CAM0, cinfo_.at(SensorId::CAM0),
                                 SensorId::CAM1, cinfo_.at(SensorId::CAM1)))
      ROS_INFO("Generated ROS Stereo Calibration, assuming cam0 (left) and cam1 (right) are a stereo pair.");
    else
      ROS_INFO("Could not read stereo calibration for cam0 and cam1.");
  }
  else{
    //Generate Stereo ROS config, assuming than cam0 and cam1 are in fronto-parallel stereo configuration
    if (getRosStereoCameraConfig(SensorId::CAM1, cinfo_.at(SensorId::CAM1),
                                 SensorId::CAM0, cinfo_.at(SensorId::CAM0)))
      ROS_INFO("Generated ROS Stereo Calibration, assuming cam1 (left) and cam0 (right) are a stereo pair.");
    else
      ROS_INFO("Could not read stereo calibration for cam0 and cam1.");
  }

  // Initialize imus
  for (auto imu_id : list_of_imu_ids_) {
    ros::Publisher temp_pub;
    temp_pub = nh_.advertise<sensor_msgs::Imu>(ROS_IMU_NAMES.at(imu_id), -1);
    printf("register publisher for imu %u\n", imu_id);
    imu_pub_.insert(std::pair<visensor::SensorId::SensorId, ros::Publisher>(imu_id, temp_pub));
    temp_pub = nh_.advertise<visensor_node::visensor_imu>("cust_" + ROS_IMU_NAMES.at(imu_id), -1);
    imu_custom_pub_.insert(std::pair<visensor::SensorId::SensorId, ros::Publisher>(imu_id, temp_pub));
  }

  calibration_service_ = nh_.advertiseService("get_camera_calibration", &ViSensor::calibrationServiceCallback, this);

  // init dynamic reconfigure
  dr_srv_.setCallback(boost::bind(&ViSensor::configCallback, this, _1, _2));
}

void ViSensor::startSensors(const unsigned int cam_rate, const unsigned int imu_rate) {
  drv_.startAllCameras(cam_rate);
  drv_.startAllImus(imu_rate);
}

void ViSensor::imuCallback(boost::shared_ptr<ViImuMsg> imu_ptr, ViErrorCode error) {
  if (error == ViErrorCodes::MEASUREMENT_DROPPED) {
    ROS_WARN("dropped imu measurement on sensor %u (check network bandwidth/sensor rate)", imu_ptr->imu_id);
    return;
  }

  const double sigma2_gyr_adis16375_d = 0;
  const double sigma2_acc_adis16375_d = 0;

  ros::Time msg_time;
  msg_time.fromNSec(imu_ptr->timestamp);

  sensor_msgs::Imu imu_msg;

  imu_msg.header.stamp = msg_time;
  imu_msg.header.frame_id = ROS_IMU_FRAME_NAMES.at(static_cast<SensorId::SensorId>(imu_ptr->imu_id));
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation_covariance[0] = 99999.9;
  imu_msg.orientation_covariance[1] = 0.0;
  imu_msg.orientation_covariance[2] = 0.0;
  imu_msg.orientation_covariance[3] = 0.0;
  imu_msg.orientation_covariance[4] = 99999.9;
  imu_msg.orientation_covariance[5] = 0.0;
  imu_msg.orientation_covariance[6] = 0.0;
  imu_msg.orientation_covariance[7] = 0.0;
  imu_msg.orientation_covariance[8] = 99999.9;
  // --- Angular Velocity.
  imu_msg.angular_velocity.x = imu_ptr->gyro[0];
  imu_msg.angular_velocity.y = imu_ptr->gyro[1];
  imu_msg.angular_velocity.z = imu_ptr->gyro[2];
  imu_msg.angular_velocity_covariance[0] = sigma2_gyr_adis16375_d;
  imu_msg.angular_velocity_covariance[1] = 0.0;
  imu_msg.angular_velocity_covariance[2] = 0.0;
  imu_msg.angular_velocity_covariance[3] = 0.0;
  imu_msg.angular_velocity_covariance[4] = sigma2_gyr_adis16375_d;
  imu_msg.angular_velocity_covariance[5] = 0.0;
  imu_msg.angular_velocity_covariance[6] = 0.0;
  imu_msg.angular_velocity_covariance[7] = 0.0;
  imu_msg.angular_velocity_covariance[8] = sigma2_gyr_adis16375_d;
  // --- Linear Acceleration.
  imu_msg.linear_acceleration.x = imu_ptr->acc[0];
  imu_msg.linear_acceleration.y = imu_ptr->acc[1];
  imu_msg.linear_acceleration.z = imu_ptr->acc[2];
  imu_msg.linear_acceleration_covariance[0] = sigma2_acc_adis16375_d;
  imu_msg.linear_acceleration_covariance[1] = 0.0;
  imu_msg.linear_acceleration_covariance[2] = 0.0;
  imu_msg.linear_acceleration_covariance[3] = 0.0;
  imu_msg.linear_acceleration_covariance[4] = sigma2_acc_adis16375_d;
  imu_msg.linear_acceleration_covariance[5] = 0.0;
  imu_msg.linear_acceleration_covariance[6] = 0.0;
  imu_msg.linear_acceleration_covariance[7] = 0.0;
  imu_msg.linear_acceleration_covariance[8] = sigma2_acc_adis16375_d;

  visensor_node::visensor_imu imu2;
  imu2.header.stamp = msg_time;
  imu2.header.frame_id = ROS_IMU_FRAME_NAMES.at(static_cast<SensorId::SensorId>(imu_ptr->imu_id));
  imu2.header.seq = 5;

  imu2.angular_velocity.x = imu_ptr->gyro[0];
  imu2.angular_velocity.y = imu_ptr->gyro[1];
  imu2.angular_velocity.z = imu_ptr->gyro[2];

  imu2.linear_acceleration.x = imu_ptr->acc[0];
  imu2.linear_acceleration.y = imu_ptr->acc[1];
  imu2.linear_acceleration.z = imu_ptr->acc[2];

  imu2.magnetometer.x = imu_ptr->mag[0];
  imu2.magnetometer.y = imu_ptr->mag[1];
  imu2.magnetometer.z = imu_ptr->mag[2];

  imu2.pressure = imu_ptr->baro;
  imu2.temperature = imu_ptr->temperature;

  // --- Publish IMU Message.
  imu_pub_.at(static_cast<SensorId::SensorId>(imu_ptr->imu_id)).publish(imu_msg);

  // --- Publish custom IMU Message.
  imu_custom_pub_.at(static_cast<SensorId::SensorId>(imu_ptr->imu_id)).publish(imu2);
}

void ViSensor::frameCallback(ViFrame::Ptr frame_ptr, ViErrorCode error) {
  if (error == ViErrorCodes::MEASUREMENT_DROPPED) {
    ROS_WARN("dropped camera image on sensor %u (check network bandwidth/sensor rate)", frame_ptr->camera_id);
    return;
  }

  int image_height = frame_ptr->height;
  int image_width = frame_ptr->width;

  // get sensor time of message
  ros::Time msg_time;
  msg_time.fromNSec(frame_ptr->timestamp);

  // check if transmission is delayed
  const double frame_delay = (ros::Time::now() - msg_time).toSec();
  if (frame_delay > THRESHOLD_DATA_DELAY_WARNING)
    ROS_WARN("Data arrived later than expected [ms]: %f", frame_delay * 1000.0);

  // get system time of message
  ros::Time msg_time_host;
  msg_time_host.fromNSec(frame_ptr->timestamp_host);

  // create new time message
  visensor_node::visensor_time_host time_msg;
  time_msg.header.stamp = msg_time;
  time_msg.timestamp_host = msg_time_host;
  pub_time_host_.publish(time_msg);

  // create new image message
  sensor_msgs::Image msg;
  msg.header.stamp = msg_time;
  msg.header.frame_id = ROS_CAMERA_FRAME_NAMES.at(static_cast<SensorId::SensorId>(frame_ptr->camera_id));

  if (frame_ptr->image_type == MONO8)
    sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO8, image_height, image_width, image_width,
                           frame_ptr->getImageRawPtr());
  else if (frame_ptr->image_type == MONO16) {
    cv::Mat image;
    image.create(image_height, image_width, CV_16UC1);

    cv::Mat image_8bit;
    image_8bit.create(image_height, image_width, CV_8UC1);

    memcpy(image.data, frame_ptr->getImageRawPtr(), (image_width) * image_height * 2);

    sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO16, image_height, image_width, image_width * 2,
                           image.data);
  } else
    ROS_WARN("[VI_SENSOR] - unknown image type!");

  // get current CameraInfo data
  sensor_msgs::CameraInfo ci = cinfo_[static_cast<SensorId::SensorId>(frame_ptr->camera_id)];

  // fill header
  ci.header.frame_id = ROS_CAMERA_FRAME_NAMES.at(static_cast<SensorId::SensorId>(frame_ptr->camera_id));
  ci.header.stamp = msg_time;

  ci.height = image_height;
  ci.width = image_width;

  // publish image
  image_pub_[static_cast<SensorId::SensorId>(frame_ptr->camera_id)].publish(msg, ci);

  calibration_pub_[static_cast<SensorId::SensorId>(frame_ptr->camera_id)].publish(
      camera_imu_calibrations_[ROS_CAMERA_NAMES.at(static_cast<SensorId::SensorId>(frame_ptr->camera_id))]);
}

void ViSensor::configCallback(visensor_node::DriverConfig &config, uint32_t level) {

  std::vector<SensorId::SensorId> all_available_sensor_ids = drv_.getListOfSensorIDs();

  // configure MPU 9150 IMU (if available)
  if (std::count(list_of_imu_ids_.begin(), list_of_imu_ids_.end(), visensor::SensorId::IMU_CAM0) > 0)
    drv_.setSensorConfigParam(visensor::SensorId::IMU_CAM0, "digital_low_pass_filter_config", 0);

  if (std::count(list_of_imu_ids_.begin(), list_of_imu_ids_.end(), visensor::SensorId::IMU_CAM1) > 0)
    drv_.setSensorConfigParam(visensor::SensorId::IMU_CAM1, "digital_low_pass_filter_config", 0);

  // ========================= CAMERA 0 ==========================

  if (std::count(list_of_camera_ids_.begin(), list_of_camera_ids_.end(), visensor::SensorId::CAM0) > 0) {
    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "agc_enable", config.cam0_agc_enable);
    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "max_analog_gain", config.cam0_max_analog_gain);
    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "global_analog_gain", config.cam0_global_analog_gain);
    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "global_analog_gain_attenuation",
                              config.cam0_global_analog_gain_attenuation);

    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "aec_enable", config.cam0_aec_enable);
    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "min_coarse_shutter_width",
                              config.cam0_min_coarse_shutter_width);
    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "max_coarse_shutter_width",
                              config.cam0_max_coarse_shutter_width);
    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "coarse_shutter_width", config.cam0_coarse_shutter_width);
    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "fine_shutter_width", config.cam0_fine_shutter_width);

    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "adc_mode", config.cam0_adc_mode);
    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "vref_adc_voltage_level",
                              config.cam0_vref_adc_voltage_level);

    drv_.setSensorConfigParam(visensor::SensorId::CAM0, "black_level_calibration_manual_override",
                              config.cam0_black_level_calibration_manual_override);
    if (config.cam0_black_level_calibration_manual_override)
      drv_.setSensorConfigParam(visensor::SensorId::CAM0, "black_level_calibration_value",
                                config.cam0_black_level_calibration_value);

  }

  // ========================= CAMERA 1 ==========================
  if (std::count(list_of_camera_ids_.begin(), list_of_camera_ids_.end(), visensor::SensorId::CAM1) > 0) {
    drv_.setSensorConfigParam(visensor::SensorId::CAM1, "agc_enable", config.cam1_agc_enable);
    drv_.setSensorConfigParam(visensor::SensorId::CAM1, "max_analog_gain", config.cam1_max_analog_gain);
    drv_.setSensorConfigParam(visensor::SensorId::CAM1, "global_analog_gain", config.cam1_global_analog_gain);
    drv_.setSensorConfigParam(visensor::SensorId::CAM1, "global_analog_gain_attenuation",
                              config.cam1_global_analog_gain_attenuation);

    drv_.setSensorConfigParam(visensor::SensorId::CAM1, "aec_enable", config.cam1_aec_enable);
    drv_.setSensorConfigParam(visensor::SensorId::CAM1, "min_coarse_shutter_width",
                              config.cam1_min_coarse_shutter_width);
    drv_.setSensorConfigParam(visensor::SensorId::CAM1, "max_coarse_shutter_width",
                              config.cam1_max_coarse_shutter_width);
    drv_.setSensorConfigParam(visensor::SensorId::CAM1, "coarse_shutter_width", config.cam1_coarse_shutter_width);

    drv_.setSensorConfigParam(visensor::SensorId::CAM1, "adc_mode", config.cam1_adc_mode);
    drv_.setSensorConfigParam(visensor::SensorId::CAM1, "vref_adc_voltage_level",
                              config.cam1_vref_adc_voltage_level);
  }

  // ========================= CAMERA 2 ==========================
  if (std::count(list_of_camera_ids_.begin(), list_of_camera_ids_.end(), visensor::SensorId::CAM2) > 0) {
    drv_.setSensorConfigParam(visensor::SensorId::CAM2, "agc_enable", config.cam2_agc_enable);
    drv_.setSensorConfigParam(visensor::SensorId::CAM2, "max_analog_gain", config.cam2_max_analog_gain);
    drv_.setSensorConfigParam(visensor::SensorId::CAM2, "global_analog_gain", config.cam2_global_analog_gain);
    drv_.setSensorConfigParam(visensor::SensorId::CAM2, "global_analog_gain_attenuation",
                              config.cam2_global_analog_gain_attenuation);

    drv_.setSensorConfigParam(visensor::SensorId::CAM2, "aec_enable", config.cam2_aec_enable);
    drv_.setSensorConfigParam(visensor::SensorId::CAM2, "min_coarse_shutter_width",
                              config.cam2_min_coarse_shutter_width);
    drv_.setSensorConfigParam(visensor::SensorId::CAM2, "max_coarse_shutter_width",
                              config.cam2_max_coarse_shutter_width);
    drv_.setSensorConfigParam(visensor::SensorId::CAM2, "coarse_shutter_width", config.cam2_coarse_shutter_width);

    drv_.setSensorConfigParam(visensor::SensorId::CAM2, "adc_mode", config.cam2_adc_mode);
    drv_.setSensorConfigParam(visensor::SensorId::CAM2, "vref_adc_voltage_level",
                              config.cam2_vref_adc_voltage_level);
  }

  // ========================= CAMERA 3 ==========================
  if (std::count(list_of_camera_ids_.begin(), list_of_camera_ids_.end(), visensor::SensorId::CAM3) > 0) {
    drv_.setSensorConfigParam(visensor::SensorId::CAM3, "agc_enable", config.cam3_agc_enable);
    drv_.setSensorConfigParam(visensor::SensorId::CAM3, "max_analog_gain", config.cam3_max_analog_gain);
    drv_.setSensorConfigParam(visensor::SensorId::CAM3, "global_analog_gain", config.cam3_global_analog_gain);
    drv_.setSensorConfigParam(visensor::SensorId::CAM3, "global_analog_gain_attenuation",
                              config.cam3_global_analog_gain_attenuation);

    drv_.setSensorConfigParam(visensor::SensorId::CAM3, "aec_enable", config.cam3_aec_enable);
    drv_.setSensorConfigParam(visensor::SensorId::CAM3, "min_coarse_shutter_width",
                              config.cam3_min_coarse_shutter_width);
    drv_.setSensorConfigParam(visensor::SensorId::CAM3, "max_coarse_shutter_width",
                              config.cam3_max_coarse_shutter_width);
    drv_.setSensorConfigParam(visensor::SensorId::CAM3, "coarse_shutter_width", config.cam3_coarse_shutter_width);

    drv_.setSensorConfigParam(visensor::SensorId::CAM3, "adc_mode", config.cam3_adc_mode);
    drv_.setSensorConfigParam(visensor::SensorId::CAM3, "vref_adc_voltage_level",
                              config.cam3_vref_adc_voltage_level);

  }
}

bool ViSensor::calibrationServiceCallback(visensor_node::visensor_calibration_service::Request  &req,
                                            visensor_node::visensor_calibration_service::Response &res)
{
  for (auto i : camera_imu_calibrations_)
    res.calibration.push_back(i.second);
  return true;
}

bool ViSensor::getRosCameraConfig(const SensorId::SensorId& camera_id, sensor_msgs::CameraInfo& cam_info) {
  //TODO(omaris) Remove hardcoded vals.
  int image_width = 752;
  int image_height = 480;

  ViCameraCalibration camera_calibration;
  if (!drv_.getCameraCalibration(camera_id, camera_calibration))
    return false;

  double c[9];
  double d[5];

  d[0] = camera_calibration.dist_coeff[0];
  d[1] = camera_calibration.dist_coeff[1];
  d[2] = camera_calibration.dist_coeff[2];
  d[3] = camera_calibration.dist_coeff[3];
  d[4] = 0.0;
  c[0] = camera_calibration.focal_point[0];
  c[1] = 0.0;
  c[2] = camera_calibration.principal_point[0];
  c[3] = 0.0;
  c[4] = camera_calibration.focal_point[1];
  c[5] = camera_calibration.principal_point[1];
  c[6] = 0.0;
  c[7] = 0.0;
  c[8] = 1.0;

  if (cam_info.D.size() != 5)
    cam_info.D.resize(5);

  for (int i = 0; i < 5; i++) {
    cam_info.D[i] = d[i];
  }

  for (int i = 0; i < 9; i++) {
    cam_info.K[i] = c[i];
  }

  cam_info.width = image_width;
  cam_info.height = image_height;

  cam_info.binning_x = 1;
  cam_info.binning_y = 1;

  cam_info.distortion_model = std::string("plumb_bob");

  return true;
}

bool ViSensor::getRosStereoCameraConfig(const SensorId::SensorId& camera_id_0,
                                          sensor_msgs::CameraInfo& cam_info_0,
                                          const SensorId::SensorId& camera_id_1,
                                          sensor_msgs::CameraInfo& cam_info_1) {
  ViCameraCalibration camera_calibration_0, camera_calibration_1;

  if (!drv_.getCameraCalibration(camera_id_0, camera_calibration_0))
    return false;
  if (!drv_.getCameraCalibration(camera_id_1, camera_calibration_1))
    return false;

  //TODO(omaris) Remove hardcoded vals.
  int image_width = 752;
  int image_height = 480;

  double c0[9];
  double d0[5];
  double r0[9];
  double p0[12];
  double rot0[9];
  double t0[3];

  double c1[9];
  double d1[5];
  double r1[9];
  double p1[12];
  double rot1[9];
  double t1[3];

  double r[9];
  double t[3];

  d0[0] = camera_calibration_0.dist_coeff[0];
  d0[1] = camera_calibration_0.dist_coeff[1];
  d0[2] = camera_calibration_0.dist_coeff[2];
  d0[3] = camera_calibration_0.dist_coeff[3];
  d0[4] = 0.0;
  c0[0] = camera_calibration_0.focal_point[0];
  c0[1] = 0.0;
  c0[2] = camera_calibration_0.principal_point[0];
  c0[3] = 0.0;
  c0[4] = camera_calibration_0.focal_point[1];
  c0[5] = camera_calibration_0.principal_point[1];
  c0[6] = 0.0;
  c0[7] = 0.0;
  c0[8] = 1.0;

  d1[0] = camera_calibration_1.dist_coeff[0];
  d1[1] = camera_calibration_1.dist_coeff[1];
  d1[2] = camera_calibration_1.dist_coeff[2];
  d1[3] = camera_calibration_1.dist_coeff[3];
  d1[4] = 0.0;
  c1[0] = camera_calibration_1.focal_point[0];
  c1[1] = 0.0;
  c1[2] = camera_calibration_1.principal_point[0];
  c1[3] = 0.0;
  c1[4] = camera_calibration_1.focal_point[1];
  c1[5] = camera_calibration_1.principal_point[1];
  c1[6] = 0.0;
  c1[7] = 0.0;
  c1[8] = 1.0;

  for (int i = 0; i < 9; ++i) {
    rot0[i] = camera_calibration_0.R[i];
    rot1[i] = camera_calibration_1.R[i];
  }
  for (int i = 0; i < 3; ++i) {
    t0[i] = camera_calibration_0.t[i];
    t1[i] = camera_calibration_1.t[i];
  }

  Eigen::Map < Eigen::Matrix3d > RR0(rot0);
  Eigen::Map < Eigen::Vector3d > tt0(t0);
  Eigen::Map < Eigen::Matrix3d > RR1(rot1);
  Eigen::Map < Eigen::Vector3d > tt1(t1);

  Eigen::Matrix4d T0 = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d T1 = Eigen::Matrix4d::Zero();

  T0.block<3, 3>(0, 0) = RR0;
  T0.block<3, 1>(0, 3) = tt0;
  T0(3, 3) = 1.0;
  T1.block<3, 3>(0, 0) = RR1;
  T1.block<3, 1>(0, 3) = tt1;
  T1(3, 3) = 1.0;

  Eigen::Matrix4d T_rel = Eigen::Matrix4d::Zero();
  T_rel = T1 * T0.inverse();

  Eigen::Map < Eigen::Matrix3d > R_rel(r);
  Eigen::Map < Eigen::Vector3d > t_rel(t);

  R_rel = T_rel.block<3, 3>(0, 0);
  t_rel << T_rel(0, 3), T_rel(1, 3), T_rel(2, 3);

  double r_temp[9];
  r_temp[0] = R_rel(0, 0);
  r_temp[1] = R_rel(0, 1);
  r_temp[2] = R_rel(0, 2);
  r_temp[3] = R_rel(1, 0);
  r_temp[4] = R_rel(1, 1);
  r_temp[5] = R_rel(1, 2);
  r_temp[6] = R_rel(2, 0);
  r_temp[7] = R_rel(2, 1);
  r_temp[8] = R_rel(2, 2);

  //cv::Mat wrapped(rows, cols, CV_32FC1, external_mem, CV_AUTOSTEP);
  cv::Mat C0(3, 3, CV_64FC1, c0, 3 * sizeof(double));
  cv::Mat D0(5, 1, CV_64FC1, d0, 1 * sizeof(double));
  cv::Mat R0(3, 3, CV_64FC1, r0, 3 * sizeof(double));
  cv::Mat P0(3, 4, CV_64FC1, p0, 4 * sizeof(double));

  cv::Mat C1(3, 3, CV_64FC1, c1, 3 * sizeof(double));
  cv::Mat D1(5, 1, CV_64FC1, d1, 1 * sizeof(double));
  cv::Mat R1(3, 3, CV_64FC1, r1, 3 * sizeof(double));
  cv::Mat P1(3, 4, CV_64FC1, p1, 4 * sizeof(double));

  cv::Mat R(3, 3, CV_64FC1, r_temp, 3 * sizeof(double));

  cv::Mat T(3, 1, CV_64FC1, t, 1 * sizeof(double));

  cv::Size img_size(image_width, image_height);

  cv::Rect roi1, roi2;
  cv::Mat Q;

  cv::stereoRectify(C0, D0, C1, D1, img_size, R, T, R0, R1, P0, P1, Q, cv::CALIB_ZERO_DISPARITY, 0,
                    img_size, &roi1, &roi2);

  if (cam_info_0.D.size() != 5)
    cam_info_0.D.resize(5);

  if (cam_info_1.D.size() != 5)
    cam_info_1.D.resize(5);

  for (int i = 0; i < 5; i++) {
    cam_info_0.D[i] = d0[i];
    cam_info_1.D[i] = d1[i];
  }
  for (int i = 0; i < 9; i++) {
    cam_info_0.K[i] = c0[i];
    cam_info_0.R[i] = R0.at<double>(i);
    cam_info_1.K[i] = c1[i];
    cam_info_1.R[i] = R1.at<double>(i);
  }
  for (int i = 0; i < 12; i++) {
    cam_info_0.P[i] = P0.at<double>(i);
    cam_info_1.P[i] = P1.at<double>(i);
  }
  cam_info_0.width = image_width;
  cam_info_1.width = image_width;

  cam_info_0.height = image_height;
  cam_info_1.height = image_height;

  cam_info_0.binning_x = 1;
  cam_info_0.binning_y = 1;
  cam_info_1.binning_x = 1;
  cam_info_1.binning_y = 1;

  cam_info_0.distortion_model = std::string("plumb_bob");
  cam_info_1.distortion_model = std::string("plumb_bob");
  return true;
}

bool ViSensor::precacheViCalibration(const SensorId::SensorId& camera_id) {

  ViCameraCalibration camera_calibration;
  visensor_node::visensor_calibration calibration;

  geometry_msgs::Pose T_IC;
  if (!drv_.getCameraCalibration(camera_id, camera_calibration)) {
    camera_imu_calibrations_.insert(std::pair<std::string, visensor_node::visensor_calibration>(ROS_CAMERA_NAMES.at(camera_id), calibration));
    return false;
  }
  tf::Matrix3x3 R_IC(camera_calibration.R[0], camera_calibration.R[3], camera_calibration.R[6],
                     camera_calibration.R[1], camera_calibration.R[4], camera_calibration.R[7],
                     camera_calibration.R[2], camera_calibration.R[5], camera_calibration.R[8]);

  tf::Quaternion q_IC;
  R_IC.getRotation(q_IC);

  T_IC.orientation.x = q_IC.x();
  T_IC.orientation.y = q_IC.y();
  T_IC.orientation.z = q_IC.z();
  T_IC.orientation.w = q_IC.w();

  T_IC.position.x = camera_calibration.t[0];
  T_IC.position.y = camera_calibration.t[1];
  T_IC.position.z = camera_calibration.t[2];

  calibration.T_IC = T_IC;
  calibration.dist_model = std::string("plumb_bob");
  calibration.dist_coeff.push_back(camera_calibration.dist_coeff[0]);
  calibration.dist_coeff.push_back(camera_calibration.dist_coeff[1]);
  calibration.dist_coeff.push_back(camera_calibration.dist_coeff[2]);
  calibration.dist_coeff.push_back(camera_calibration.dist_coeff[3]);
  calibration.dist_coeff.push_back(camera_calibration.dist_coeff[4]);

  calibration.principal_point.push_back(camera_calibration.principal_point[0]);
  calibration.principal_point.push_back(camera_calibration.principal_point[1]);

  calibration.focal_length.push_back(camera_calibration.focal_point[0]);
  calibration.focal_length.push_back(camera_calibration.focal_point[1]);

  calibration.cam_name = ROS_CAMERA_NAMES.at(camera_id);
  camera_imu_calibrations_.insert(std::pair<std::string, visensor_node::visensor_calibration>(ROS_CAMERA_NAMES.at(camera_id), calibration));
  return true;
}

}  //namespace visensor
