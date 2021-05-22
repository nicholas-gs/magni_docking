#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <cmath>

namespace magni {
    // Calculate the relative degrees of rotation between 2 coordinate frames
    double degreesOfRotation(geometry_msgs::TransformStamped& benchmark_tf, geometry_msgs::TransformStamped& current_tf) {
        tf2::Quaternion benchmark_quat, current_quat;
        tf2::convert(benchmark_tf.transform.rotation, benchmark_quat);
        tf2::convert(current_tf.transform.rotation, current_quat);
        tf2::Quaternion diff = benchmark_quat * current_quat.inverse();
        double roll, pitch, yaw;
        tf2::Matrix3x3(diff).getRPY(roll, pitch, yaw);
        return abs(yaw);
    }
}
