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

#ifndef VI_SENSOR_FRONTEND_H_
#define VI_SENSOR_FRONTEND_H_

#include "visensor/visensor.hpp"

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/fill_image.h>

#include "visensor_msgs/visensor_imu.h"
#include "visensor_msgs/visensor_time_host.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <visensor_node/node_example_paramsConfig.h>

#include <stdint.h>
#include <sstream>
#include <string>

#include <assert.h>
#include <sys/time.h>

class ViSensorFrontend {

 public:
  ViSensorFrontend(ros::NodeHandle& nh);
  ~ViSensorFrontend();
  void startSensors(void);
  void imuCallback(boost::shared_ptr<visensor::ViImuMsg> imu_ptr);
  void frameCallback(visensor::ViFrame::Ptr frame_ptr);
  void configCallback(visensor_node::node_example_paramsConfig &config,
                      uint32_t level);

 private:
  void init();
  void initReconfigure();

  ros::NodeHandle _nh;
  ros::Publisher _pub_time_host;
  dynamic_reconfigure::Server<visensor_node::node_example_paramsConfig> _dr_srv;

  std::map<int, ros::NodeHandle> _nhc;
  std::map<int, image_transport::ImageTransport> _itc;
  std::map<int, image_transport::CameraPublisher> _image_pub;
  std::map<int, camera_info_manager::CameraInfoManager*> _cinfo;
  std::map<int, ros::Publisher> _imu_pub;
  std::map<int, ros::Publisher> _imu_pub2;

  std::vector<int> _list_camera_ids;
  std::vector<int> _list_imu_ids;

  visensor::ViSensorDriver _drv;

  double _min_frame_delay;
  uint32_t _fpga_id;
  bool _firstrun;
};

static const std::string ROS_CAMERA_NAMES[] = { "cam0", "cam1", "cam2", "cam3",
    "cam4", "cam5", "cam6", "cam7" };
static const std::string ROS_CAMERA_FRAME_NAMES[] = { "cam0", "cam0", "cam0",
    "cam0", "cam4", "cam5", "cam6", "cam7" };
static const std::string ROS_IMU_NAMES[] = { "", "", "", "", "imu0", "imu1",
    "imu2", "imu3" };
static const std::string ROS_IMU_FRAME_NAMES[] = { "imu0", "imu1", "imu2",
    "imu3", "imu4", "imu5", "imu6", "imu7" };

#endif /* VI_SENSOR_H_ */
