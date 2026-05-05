# crsf_joy
This package provides a hardware abstraction bridge between an ExpressLRS (ELRS) receiver connected via UART, and the standard ROS 2 joystick interface. Its sole responsibility is to consume raw CRSF serial frames coming from the receiver and republish the decoded RC channel data as a sensor_msgs/msg/Joy message.
