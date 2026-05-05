# crsf_joy
This package provides a hardware abstraction bridge between an ExpressLRS (ELRS) receiver connected via UART, and the standard ROS 2 joystick interface. Its sole responsibility is to consume raw CRSF serial frames coming from the receiver and republish the decoded RC channel data as a sensor_msgs/msg/Joy message.



# Datasheet XR4 receiver
https://cdn.shopify.com/s/files/1/0609/8324/7079/files/XR4.pdf?v=1739432399
