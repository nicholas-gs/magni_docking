#include "ros/ros.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs/h"
#include "docking_math.hpp"

#include <cmath>

int main(int argc, char** argv) {
    tf2_ros::Buffer m_TFBuffer;
    tf2_ros::TransformListener m_TFListener(m_TFBuffer);
    return 0;
}
