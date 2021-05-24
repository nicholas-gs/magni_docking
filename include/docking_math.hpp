#include "geometry_msgs/TransformStamped.h"
#include "fiducial_msgs/FiducialTransform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <cmath>

struct Fid2Pos_Data {
    Fid2Pos_Data(){}
    Fid2Pos_Data(double theta, double distance, double theta_bounds)
        :   m_Theta(theta),
            m_Distance(distance),
            m_Theta_Bounds(theta_bounds) {}

    double m_Theta, m_Distance, m_Theta_Bounds;
};

// Calculate the relative degrees of rotation between 2 coordinate frames
double degreesOfRotation(const geometry_msgs::TransformStamped& benchmark_tf, const geometry_msgs::TransformStamped& current_tf) {
        tf2::Quaternion benchmark_quat, current_quat;
        tf2::convert(benchmark_tf.transform.rotation, benchmark_quat);
        tf2::convert(current_tf.transform.rotation, current_quat);
        tf2::Quaternion diff = benchmark_quat * current_quat.inverse();
        double roll, pitch, yaw;
        tf2::Matrix3x3(diff).getRPY(roll, pitch, yaw);
    return fabs(yaw);
}

inline double distanceApart(const geometry_msgs::TransformStamped& benchmark_tf, const geometry_msgs::TransformStamped& current_tf) {
    double x_diff = fabs(benchmark_tf.transform.translation.x - current_tf.transform.translation.x);
    double y_diff = fabs(benchmark_tf.transform.translation.y - current_tf.transform.translation.y);
    return sqrt(pow(x_diff, 2) + pow(y_diff, 2));
}

Fid2Pos_Data fid2pos(fiducial_msgs::FiducialTransform& fid_tf) {
    double x_trans = fid_tf.transform.translation.x;
    double z_trans = fid_tf.transform.translation.z;
    Fid2Pos_Data m_Data;
    m_Data.m_Theta = atan2(x_trans, z_trans);
    m_Data.m_Distance = sqrt(pow(x_trans,2) + pow(z_trans,2));
    ROS_INFO("Distance to fiducial: %.2lf, Angle: %0.2lf", m_Data.m_Distance, m_Data.m_Theta);
    if(m_Data.m_Distance > 2.0) {
        m_Data.m_Theta_Bounds = 0.1;
   } else {
        m_Data.m_Theta_Bounds = m_Data.m_Distance / 5;
   }
//    m_Data.m_Theta_Bounds = 0.1;
    return m_Data;
}
