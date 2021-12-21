### About

ROS2 image publisher w/o extra dependencies other than v4l library. It can
capture and publish MJPEG and RGB24 images. The idea is to keep it simple,
stupid (KISS ;)) by utilizing standard sensor message instead of creating
ROS2 plugin with MJPEG support.

### Build

 * Install build essentials (c++ compiler etc.)
 * Setup ROS2 environment (follow ROS2 guide)
 * Compile

		~/> mkdir build
		~/> cd build
		~/build> cmake .. && make

### Run

Recognized environment variables (with examples)

	CAMERA_DEV=/dev/video0	# path to camera device node
	CAMERA_GEOM=1920x1080	# desired image size (if supported)
	CAMERA_FPS=30 		# desired framerate (if supported)
	CAMERA_SCALE=-2		# scale down factor (-2..-5)
	CAMERA_JPG=1			# capture jpeg images if supported by camera (undefine to disable)

### Examples

1. Publish images captured by MJPEG enabled camera:

		~/build> CAMERA_DEV=/dev/video0 CAMERA_GEOM=1920x1080 CAMERA_JPG=1 ./roscam

	> * The topic string will look like this: */dev/video0/image_jpg*
	> * *std::sensor_msgs::Image*'s payload will carry encoded image.
	> * *std::sensor_msgs::Image*'s *encoding* string is set to "jpg".
	> * Subscriber must implement JPEG decoding.

2. Publish resized RGB images:

		~/build> CAMERA_DEV=/dev/video0 CAMERA_GEOM=1920x1080 CAMERA_SCALE=-2 ./roscam

	> Published image size will be 960x540 pixels.
