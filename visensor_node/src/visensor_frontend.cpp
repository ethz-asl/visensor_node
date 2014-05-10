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
 *     http://www.apache.org/licenses/LICENSE-2.0
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

#include "visensor_frontend.h"

ViSensorFrontend::ViSensorFrontend(ros::NodeHandle& nh)
    : _nh(nh),
      _min_frame_delay(std::numeric_limits<double>::max()) {
  init();
  initReconfigure();
}

ViSensorFrontend::~ViSensorFrontend() {
}

void ViSensorFrontend::init() {
  _firstrun = true;

  try {
    _drv.init();
  } catch (visensor::exceptions const &ex) {
    std::cout << ex.what() << "\n";
    exit(1);
  }

  _list_camera_ids = _drv.getListOfCameraIDs();
  _list_imu_ids = _drv.getListOfImuIDs();

  _fpga_id = _drv.getFpgaId();

  std::string rootdir = ros::package::getPath("visensor_node");
  std::string tempCameraInfoFileName;

// init time host publisher
  _pub_time_host = _nh.advertise<visensor_msgs::visensor_time_host>(
      "time_host", -1);

// set callback for completed frames
  _drv.setCameraCallback(
      boost::bind(&ViSensorFrontend::frameCallback, this, _1));

// set callback for completed IMU messages
  _drv.setImuCallback(boost::bind(&ViSensorFrontend::imuCallback, this, _1));

// initialize cameras
  for (unsigned int i = 0; i < _list_camera_ids.size(); i++) {
    //create new ros handle for each camera
    ros::NodeHandle nhc_temp(_nh, ROS_CAMERA_NAMES[_list_camera_ids[i]]);
    _nhc.insert(std::pair<int, ros::NodeHandle>(_list_camera_ids[i], nhc_temp));

    // create new image transport for each camera
    image_transport::ImageTransport itc_temp(_nhc[i]);
    _itc.insert(
        std::pair<int, image_transport::ImageTransport>(_list_camera_ids[i],
                                                        itc_temp));

    // create new image publisher
    ROS_INFO_STREAM("register publisher for camera " << _list_camera_ids[i]);
    image_transport::CameraPublisher image_pub = (_itc.at(_list_camera_ids[i]))
        .advertiseCamera("image_raw", 100);
    _image_pub.insert(
        std::pair<int, image_transport::CameraPublisher>(_list_camera_ids[i],
                                                         image_pub));

    std::stringstream ssDir;
    ssDir << "file://" << rootdir << "/calibration/fpga" << _fpga_id << "_cam"
          << _list_camera_ids[i] << ".yaml";
    ssDir >> tempCameraInfoFileName;
    // create new camera info manager
    camera_info_manager::CameraInfoManager* cinfo_temp =
        new camera_info_manager::CameraInfoManager(_nhc[_list_camera_ids[i]],
                                                   ROS_CAMERA_NAMES[i],
                                                   tempCameraInfoFileName);
    _cinfo.insert(
        std::pair<int, camera_info_manager::CameraInfoManager*>(
            _list_camera_ids[i], cinfo_temp));
  }

// Initialize imus
  for (unsigned int i = 0; i < _list_imu_ids.size(); i++) {
    ros::Publisher temp_pub;
    temp_pub = _nh.advertise<sensor_msgs::Imu>(ROS_IMU_NAMES[_list_imu_ids[i]],
                                               -1);
    _imu_pub.insert(std::pair<int, ros::Publisher>(_list_imu_ids[i], temp_pub));
    temp_pub = _nh.advertise<visensor_msgs::visensor_imu>(
        "cust_" + ROS_IMU_NAMES[_list_imu_ids[i]], -1);
    _imu_pub2.insert(
        std::pair<int, ros::Publisher>(_list_imu_ids[i], temp_pub));

  }
}

void ViSensorFrontend::initReconfigure() {

  dynamic_reconfigure::Server<visensor_node::node_example_paramsConfig>::CallbackType cb;
  cb = boost::bind(&ViSensorFrontend::configCallback, this, _1, _2);
  _dr_srv.setCallback(cb);
}

void ViSensorFrontend::startSensors(void) {
  _drv.startAllCameras(CAMERA_FREQUENCY);
  _drv.startAllImus(IMU_FREQUENCY);
}

void ViSensorFrontend::imuCallback(
    boost::shared_ptr<visensor::ViImuMsg> imu_ptr) {

  double sigma2_gyr_adis16375_d = 6.0e-4;
  double sigma2_acc_adis16375_d = 2.0e-3;

  ros::Time temp_time;
  temp_time.fromNSec(imu_ptr->timestamp);

  sensor_msgs::Imu imu_msg;

  imu_msg.header.stamp = temp_time;
  imu_msg.header.frame_id = ROS_IMU_FRAME_NAMES[imu_ptr->imu_id];

// --- Attitude.
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

// ---------- Publish custom IMU. ----------

  visensor_msgs::visensor_imu imu2;
  imu2.header.stamp = temp_time;
  imu2.header.frame_id = ROS_IMU_FRAME_NAMES[imu_ptr->imu_id];
  imu2.header.seq = 5;    //imu_ptr->timestamp;
// --- Angular Velocity.
  imu2.angular_velocity.x = imu_ptr->gyro[0];
  imu2.angular_velocity.y = imu_ptr->gyro[1];
  imu2.angular_velocity.z = imu_ptr->gyro[2];
// --- Linear Acceleration.
  imu2.linear_acceleration.x = imu_ptr->acc[0];
  imu2.linear_acceleration.y = imu_ptr->acc[1];
  imu2.linear_acceleration.z = imu_ptr->acc[2];

  imu2.magnetometer.x = imu_ptr->mag[0];
  imu2.magnetometer.y = imu_ptr->mag[1];
  imu2.magnetometer.z = imu_ptr->mag[2];

  imu2.pressure = imu_ptr->baro;

// --- Publish IMU Message.
  _imu_pub.at(imu_ptr->imu_id).publish(imu_msg);
// --- Publish custom IMU Message.
  _imu_pub2.at(imu_ptr->imu_id).publish(imu2);

}

void ViSensorFrontend::frameCallback(visensor::ViFrame::Ptr frame_ptr) {

  int image_height = frame_ptr->height;
  int image_width = frame_ptr->width;

// get sensor time of message
  ros::Time temp_time;
  temp_time.fromNSec(frame_ptr->timestamp);

// check if transmission is delayed
  const double frame_delay = (ros::Time::now() - temp_time).toSec();
  _min_frame_delay = std::min(frame_delay, _min_frame_delay);
#if 0  
	if (frame_delay > _min_frame_delay * 2)
    std::cout << "frame delay longer than expected [ms]: "
              << frame_delay * 1000.0 << std::endl;
#endif

// get system time of message
  ros::Time temp_time_host;
  temp_time_host.fromNSec(frame_ptr->timestamp_host);

// create new time message
  visensor_msgs::visensor_time_host time_msg;
  time_msg.header.stamp = temp_time;
  time_msg.timestamp_host = temp_time_host;
  _pub_time_host.publish(time_msg);

// create new image message
  sensor_msgs::Image msg;
  msg.header.stamp = temp_time;
  msg.header.frame_id = ROS_CAMERA_FRAME_NAMES[frame_ptr->camera_id];

  if (frame_ptr->image_type == visensor::MONO8)
    sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO8,
                           image_height, image_width, image_width,
                           frame_ptr->getImageRawPtr());
  else if (frame_ptr->image_type == visensor::MONO16) {

    cv::Mat image;
    image.create(image_height, image_width, CV_16UC1);

    cv::Mat image_8bit;
    image_8bit.create(image_height, image_width, CV_8UC1);

    memcpy(image.data, frame_ptr->getImageRawPtr(),
           (image_width) * image_height * 2);

    sensor_msgs::fillImage(msg, sensor_msgs::image_encodings::MONO16,
                           image_height, image_width, image_width * 2,
                           image.data);
  } else
    ROS_WARN("[SLAM_SENSOR] - unknown image type!");

// get current CameraInfo data
  sensor_msgs::CameraInfo ci(
      sensor_msgs::CameraInfo(_cinfo[frame_ptr->camera_id]->getCameraInfo()));

// fill header
  ci.header.frame_id = ROS_CAMERA_FRAME_NAMES[frame_ptr->camera_id];
  ci.header.stamp = temp_time;

  ci.height = image_height;
  ci.width = image_width;

// publish image
  _image_pub[frame_ptr->camera_id].publish(msg, ci);
}

void ViSensorFrontend::configCallback(
    visensor_node::node_example_paramsConfig &config, uint32_t level) {

  _drv.setLedConfigParam("strobe", config.strobe);

// ========================= CAMERA 0 ==========================

  if (std::count(_list_camera_ids.begin(), _list_camera_ids.end(),
                 visensor::SensorId::CAM0) > 0) {
    _drv.setSensorConfigParam(visensor::SensorId::CAM0, "agc_enable",
                              config.cam0_agc_enable);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0, "max_analog_gain",
                              config.cam0_max_analog_gain);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0, "global_analog_gain",
                              config.cam0_global_analog_gain);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0,
                              "global_analog_gain_attenuation",
                              config.cam0_global_analog_gain_attenuation);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0, "aec_enable",
                              config.cam0_aec_enable);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0,
                              "min_coarse_shutter_width",
                              config.cam0_min_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0,
                              "max_coarse_shutter_width",
                              config.cam0_max_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0, "coarse_shutter_width",
                              config.cam0_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0, "fine_shutter_width",
                              config.cam0_fine_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0, "adc_mode",
                              config.cam0_adc_mode);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0,
                              "vref_adc_voltage_level",
                              config.cam0_vref_adc_voltage_level);
    _drv.setSensorConfigParam(
        visensor::SensorId::CAM0, "black_level_calibration_manual_override",
        config.cam0_black_level_calibration_manual_override);
    if (config.cam0_black_level_calibration_manual_override)
      _drv.setSensorConfigParam(visensor::SensorId::CAM0,
                                "black_level_calibration_value",
                                config.cam0_black_level_calibration_value);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0, "row_flip",
                              config.cam0_row_flip);
    _drv.setSensorConfigParam(visensor::SensorId::CAM0, "column_flip",
                              config.cam0_column_flip);
  }

// ========================= CAMERA 1 ==========================
  if (std::count(_list_camera_ids.begin(), _list_camera_ids.end(),
                 visensor::SensorId::CAM1) > 0) {
    _drv.setSensorConfigParam(visensor::SensorId::CAM1, "agc_enable",
                              config.cam1_agc_enable);
    _drv.setSensorConfigParam(visensor::SensorId::CAM1, "max_analog_gain",
                              config.cam1_max_analog_gain);
    _drv.setSensorConfigParam(visensor::SensorId::CAM1, "global_analog_gain",
                              config.cam1_global_analog_gain);
    _drv.setSensorConfigParam(visensor::SensorId::CAM1,
                              "global_analog_gain_attenuation",
                              config.cam1_global_analog_gain_attenuation);
    _drv.setSensorConfigParam(visensor::SensorId::CAM1, "aec_enable",
                              config.cam1_aec_enable);
    _drv.setSensorConfigParam(visensor::SensorId::CAM1,
                              "min_coarse_shutter_width",
                              config.cam1_min_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM1,
                              "max_coarse_shutter_width",
                              config.cam1_max_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM1, "coarse_shutter_width",
                              config.cam1_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM1, "adc_mode",
                              config.cam1_adc_mode);
    _drv.setSensorConfigParam(visensor::SensorId::CAM1,
                              "vref_adc_voltage_level",
                              config.cam1_vref_adc_voltage_level);
    _drv.setSensorConfigParam(visensor::SensorId::CAM1, "row_flip",
                              config.cam1_row_flip);
    _drv.setSensorConfigParam(visensor::SensorId::CAM1, "column_flip",
                              config.cam1_column_flip);
  }

// ========================= CAMERA 2 ==========================
  if (std::count(_list_camera_ids.begin(), _list_camera_ids.end(),
                 visensor::SensorId::CAM2) > 0) {
    _drv.setSensorConfigParam(visensor::SensorId::CAM2, "agc_enable",
                              config.cam2_agc_enable);
    _drv.setSensorConfigParam(visensor::SensorId::CAM2, "max_analog_gain",
                              config.cam2_max_analog_gain);
    _drv.setSensorConfigParam(visensor::SensorId::CAM2, "global_analog_gain",
                              config.cam2_global_analog_gain);
    _drv.setSensorConfigParam(visensor::SensorId::CAM2,
                              "global_analog_gain_attenuation",
                              config.cam2_global_analog_gain_attenuation);
    _drv.setSensorConfigParam(visensor::SensorId::CAM2, "aec_enable",
                              config.cam2_aec_enable);
    _drv.setSensorConfigParam(visensor::SensorId::CAM2,
                              "min_coarse_shutter_width",
                              config.cam2_min_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM2,
                              "max_coarse_shutter_width",
                              config.cam2_max_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM2, "coarse_shutter_width",
                              config.cam2_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM2, "adc_mode",
                              config.cam2_adc_mode);
    _drv.setSensorConfigParam(visensor::SensorId::CAM2,
                              "vref_adc_voltage_level",
                              config.cam2_vref_adc_voltage_level);
    _drv.setSensorConfigParam(visensor::SensorId::CAM2, "row_flip",
                              config.cam2_row_flip);
    _drv.setSensorConfigParam(visensor::SensorId::CAM2, "column_flip",
                              config.cam2_column_flip);
  }

// ========================= CAMERA 3 ==========================
  if (std::count(_list_camera_ids.begin(), _list_camera_ids.end(),
                 visensor::SensorId::CAM3) > 0) {
    _drv.setSensorConfigParam(visensor::SensorId::CAM3, "agc_enable",
                              config.cam3_agc_enable);
    _drv.setSensorConfigParam(visensor::SensorId::CAM3, "max_analog_gain",
                              config.cam3_max_analog_gain);
    _drv.setSensorConfigParam(visensor::SensorId::CAM3, "global_analog_gain",
                              config.cam3_global_analog_gain);
    _drv.setSensorConfigParam(visensor::SensorId::CAM3,
                              "global_analog_gain_attenuation",
                              config.cam3_global_analog_gain_attenuation);
    _drv.setSensorConfigParam(visensor::SensorId::CAM3, "aec_enable",
                              config.cam3_aec_enable);
    _drv.setSensorConfigParam(visensor::SensorId::CAM3,
                              "min_coarse_shutter_width",
                              config.cam3_min_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM3,
                              "max_coarse_shutter_width",
                              config.cam3_max_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM3, "coarse_shutter_width",
                              config.cam3_coarse_shutter_width);
    _drv.setSensorConfigParam(visensor::SensorId::CAM3, "adc_mode",
                              config.cam3_adc_mode);
    _drv.setSensorConfigParam(visensor::SensorId::CAM3,
                              "vref_adc_voltage_level",
                              config.cam3_vref_adc_voltage_level);
    _drv.setSensorConfigParam(visensor::SensorId::CAM3, "row_flip",
                              config.cam3_row_flip);
    _drv.setSensorConfigParam(visensor::SensorId::CAM3, "column_flip",
                              config.cam3_column_flip);
  }

}
