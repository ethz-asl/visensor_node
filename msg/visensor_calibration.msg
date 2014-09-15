string cam_name #camera name which is used as base topic for publishing
geometry_msgs/Pose 	T_IC #pose mapping a point expressed in camera frame into imu frame
string dist_model #distortion model	
float64[] dist_coeff #distortion coefficients	
float64[] principal_point #principal point in pixels	
float64[] focal_length #focal lengths in pixels
		