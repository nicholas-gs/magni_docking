#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "docking_math.hpp"

#include <cmath>

// Forward Declerations
struct DockingParams;
class Searching_Params;
class DockingController;
class FSM_State_Controller;

struct DockingParams {
    DockingParams() {
        if(!ros::param::get("~dock_position", m_Dock_Position)) {
            ROS_WARN("Docking Position not set, check launch file");
        }
        if(!ros::param::get("~linear_vel", m_Linear_Vel)) {
            ROS_WARN("Linear velocity not set, check launch file");
        }
        if(!ros::param::get("~final_linear_vel", m_Final_Linear_Vel)) {
            ROS_WARN("Final approach linear velocity not set, check launch file");
        }
        if(!ros::param::get("~angular_vel", m_Angular_Vel)) {
            ROS_WARN("Angular velocity not set, check launch file");
        }
        if(!ros::param::get("~search_angle", m_Search_Angle)) {
            ROS_WARN("Search angle not set, check launch file");
        }
        if(!ros::param::get("~docking_marker", m_Target_Fid)) {
            ROS_WARN("Target fiducial marker not set, check launch file");
        }
        if(!ros::param::get("~final_approach_distance", m_Final_Approach_Distance)) {
            ROS_WARN("Final approach distance not set, check launch file");
        }
        if(!ros::param::get("~blind_distance", m_Blind_Distance)) {
            ROS_WARN("Blind distance not set, check launch file");
        }
        if(m_Dock_Position == "right") {
            m_Angular_Vel *= -1.0;
        }
    }

    std::string m_Dock_Position;
    double m_Angular_Vel = 0.0;
    double m_Linear_Vel = 0.0;
    double m_Final_Linear_Vel = 0.0;
    double m_Search_Angle = 0.0;
    int32_t m_Target_Fid;
    double m_Final_Approach_Distance;
    double m_Blind_Distance;
};

struct Searching_Params {
        bool m_Found = false;
        geometry_msgs::TransformStamped m_Benchmark_TF;
        geometry_msgs::TransformStamped m_Current_TF;
};

struct Final_Approach_Params {
        geometry_msgs::TransformStamped m_Benchmark_TF;
        geometry_msgs::TransformStamped m_Current_TF;
};

class FSM_State_Controller {
    public:
        enum class States {
            undocked, searching, centering, approaching, final_approach, docked, failed
        };
        FSM_State_Controller() {
            std::string dock_status;
            if(!ros::param::get("/docking_status", dock_status)) {
                ROS_ERROR("Docking status not found!");
            }
            if(dock_status == "docked") {
                m_Current_State = States::docked;
            } else if(dock_status == "undocked") {
                m_Current_State = States::undocked;
            } else {
                ROS_ERROR("Robot is in unknown state!");
            }
        }
        States getState() const {
            return m_Current_State;
        }
        void setState(States new_state) {
            m_Current_State = new_state;
        }
        std::string& errorMsg() {
            return m_Error_Msg;
        }
    private:
        States m_Current_State;
        std::string m_Error_Msg;
};

using FSM_SC_States = FSM_State_Controller::States;

class DockingController {
    public:
        DockingController(ros::NodeHandle nh)
            :   m_Docking_Command_Sub(nh.subscribe<std_msgs::String>("docking_command", 1,
                    &DockingController::Docking_Command_Callback, this)),
                m_Cmd_Pub(nh.advertise<geometry_msgs::Twist>("cmd_vel", 1)),
                m_FiducialsTF_Sub(nh.subscribe("fiducial_transforms", 1,
                    &DockingController::Fiducial_Transforms_Callback, this)),
                m_TFListener(m_TFBuffer),
                m_FSM_Timer(nh.createTimer(ros::Duration(0.1), &DockingController::Manage_FSM_State, this)) {}

        // Callback for "docking_command" topic subscriber
        void Docking_Command_Callback(const std_msgs::String::ConstPtr& msg) {
            if(msg->data == "dock") {
                if(m_FSM_State_Controller.getState() == FSM_SC_States::docked) {
                    ROS_INFO("Robot is already docked");
                } else if(m_FSM_State_Controller.getState() == FSM_SC_States::undocked) {
                    ROS_INFO("Starting docking");
                    m_FSM_State_Controller.setState(FSM_SC_States::searching);
                    // Reset the searching params
                    m_Docking_Params = DockingParams();
                    m_Searching_Params.m_Found = false;
                    getOdomToBaseLinkTF(m_Searching_Params.m_Benchmark_TF);
                }
            } else if(msg->data == "cancel") {
                // Cancel current goal
                FSM_SC_States current_state = m_FSM_State_Controller.getState();
                if(current_state == FSM_SC_States::docked || current_state == FSM_SC_States::undocked) {
                    ROS_INFO("%s is not in docking or undocking process", ros::this_node::getName().c_str());
                } else {
                    // todo -- cancel current docking or undocking goal
                    ROS_INFO("Cancelling goal");
                    m_FSM_State_Controller.setState(FSM_SC_States::failed);
                    m_FSM_State_Controller.errorMsg() = "Process cancelled";
                }
            } else {
                ROS_ERROR("Unknown command to %s node", ros::this_node::getName().c_str());
            }
        }

        // Callback for "fiducial_transforms" topic subscriber
        void Fiducial_Transforms_Callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
            FSM_SC_States current_state = m_FSM_State_Controller.getState();
            for(std::size_t i = 0; i < (msg->transforms).size(); ++i) {
                const fiducial_msgs::FiducialTransform& ft = msg->transforms[i];
                if(ft.fiducial_id == m_Docking_Params.m_Target_Fid) {
                    broadcast_fid_to_base_link_tf(ft);
                    if(current_state == FSM_SC_States::searching) {
                        m_Searching_Params.m_Found = true;
                    } else if(current_state == FSM_SC_States::centering || current_state == FSM_SC_States::approaching) {
                        m_Fid_In_View = true;
                        m_Last_Fid_TF = ft;
                    }
                    return;
                }
            }
            m_Fid_In_View = false;
        }

        // Callback for FSM timer
        void Manage_FSM_State(const ros::TimerEvent& timer_event) {
            switch(m_FSM_State_Controller.getState()) {
                case FSM_SC_States::failed:
                    motion_stop();
                    ROS_INFO("Docking failed: %s", m_FSM_State_Controller.errorMsg().c_str());
                    // Reset the FSM state controller
                    m_FSM_State_Controller = FSM_State_Controller();
                    break;
                case FSM_SC_States::undocked:
                    break;
                case FSM_SC_States::searching:
                    // todo - clean up this if-elif
                    if(m_Searching_Params.m_Found) {
                        m_FSM_State_Controller.setState(FSM_SC_States::centering);
                    } else if(search_state_func()) {
                        ROS_INFO("search_state_func() over");
                        if(m_Searching_Params.m_Found) {
                            m_FSM_State_Controller.setState(FSM_SC_States::centering);
                        } else {
                            m_FSM_State_Controller.setState(FSM_SC_States::failed);
                            m_FSM_State_Controller.errorMsg() = "Cannot find fiducial";
                        }
                    }
                    break;
                case FSM_SC_States::centering:
                     ROS_INFO("In centering");
                    if(centering_state_func()) {
                        m_FSM_State_Controller.setState(FSM_SC_States::approaching);
                    }
                    break;
                case FSM_SC_States::approaching:
                    if(approaching_state_func()) {
                        getOdomToBaseLinkTF(m_Final_Approach_Params.m_Benchmark_TF);
                        m_FSM_State_Controller.setState(FSM_SC_States::final_approach);
                    }
                    break;
                case FSM_SC_States::final_approach:
                    if(final_approach_func()) {
                        m_FSM_State_Controller.setState(FSM_SC_States::docked);
                        ros::param::set("/docking_status", "docked");
                    }
                    break;
            }
        }

    private:
        ros::Subscriber m_Docking_Command_Sub;
        ros::Publisher m_Cmd_Pub;
        ros::Subscriber m_FiducialsTF_Sub;
        tf2_ros::Buffer m_TFBuffer;
        tf2_ros::TransformListener m_TFListener;
        tf2_ros::TransformBroadcaster m_TFBroadcaster;

        FSM_State_Controller m_FSM_State_Controller;
        DockingParams m_Docking_Params;
        ros::Timer m_FSM_Timer;
        Searching_Params m_Searching_Params;
        Final_Approach_Params m_Final_Approach_Params;

        geometry_msgs::Twist m_Twist;
        bool m_Fid_In_View = false;
        fiducial_msgs::FiducialTransform m_Last_Fid_TF;

        // Return true if searching angle is completed
        bool search_state_func() {
            ROS_INFO("Search state");
            getOdomToBaseLinkTF(m_Searching_Params.m_Current_TF);
            return motion_turn(m_Searching_Params.m_Benchmark_TF, m_Docking_Params.m_Search_Angle, m_Docking_Params.m_Angular_Vel);
        }

        // The robot will attempt to rotate on the spot so that the angle between the robot and fiducial
        // is below an upper limit.
        // Return true of centering is complete
        bool centering_state_func() {
            ROS_INFO("Centering state");
            // todo -- Maybe set a limit on the number of attempts of centering?
            // todo -- fix bug!!! Wrong TF
            if(!m_Fid_In_View) {
                ROS_WARN("Cannot see fiducial anymore");
                // motion_stop();
                motion_forward(m_Docking_Params.m_Linear_Vel);
                return false;
            }
            Fid2Pos_Data fid_data = fid2pos(m_Last_Fid_TF);
            ROS_INFO("theta: %.2lf , theta bounds: %.2lf", fid_data.m_Theta, fid_data.m_Theta_Bounds);
            if(fabs(fid_data.m_Theta) > fabs(fid_data.m_Theta_Bounds)) {
                double ang_vel = m_Docking_Params.m_Angular_Vel;
                if(fid_data.m_Theta > 0.0) {
                    ang_vel *= -1.0;
                }
                motion_turn(ang_vel);
                return false;
            }
            motion_stop();
            return true;
        }

        // Gradually move towards the fiducial while ensuring that its approach angle is within a limit.
        // This is to prevent the robot approaching the fiducial at too great an angle.
        // Return true if ready to move to the final approach state
        bool approaching_state_func() {
            ROS_INFO("Approaching state");
            Fid2Pos_Data fid_data = fid2pos(m_Last_Fid_TF);
            // If fiducial is not seen at the moment, then stop for the time being
            if(!m_Fid_In_View) {
                ROS_WARN("Cannot see fiducial anymore");
             //   motion_stop();
                return false;
            }
            if(fabs(fid_data.m_Theta) > fabs(fid_data.m_Theta_Bounds)) {
                ROS_INFO("Approach angle exceeding limit");
                motion_stop();
                m_FSM_State_Controller.setState(FSM_SC_States::centering);
                return false;
            }
            if(fabs(fid_data.m_Distance) < m_Docking_Params.m_Final_Approach_Distance) {
                motion_stop();
                return true;
            } else {
                motion_forward(m_Docking_Params.m_Final_Linear_Vel);
                return false;
            }
        }

        // At the final approach, the RPI camera may not be able to see if fiducial due to
        // focus issues or the angle the camera is mounted. So we just move the robot forward
        // a fixed distance, also know as the "blind distance"
        // Return true if robot has travelled the "blind distance" required
        bool final_approach_func() {
            ROS_INFO("Final Approach State");
            return motion_forward(m_Final_Approach_Params.m_Benchmark_TF , m_Docking_Params.m_Blind_Distance,
                                       m_Docking_Params.m_Final_Linear_Vel);
        }

        // Turn a fixed angle with reference to the ref_tf. The angle is absolute,
        // so to control which direction to turn, change the sign of the angular velocity
        // Returns true if turning is complete, false if robot needs to turn more
        bool motion_turn(const geometry_msgs::TransformStamped& ref_tf, double angle, double ang_vel) {
            geometry_msgs::TransformStamped curr_tf;
            getOdomToBaseLinkTF(curr_tf);
            double deg = degreesOfRotation(ref_tf, curr_tf);
            ROS_INFO("Rotated %lf / %lf", fabs(deg), fabs(angle));
            if(fabs(deg) < fabs(angle)) {
                ROS_INFO("Still need to rotate more");
                m_Twist.angular.z = ang_vel;
                m_Cmd_Pub.publish(m_Twist);
                return false;
            }
            ROS_INFO("Rotated enough, stopping rotation now");
            motion_stop();
            return true;
        }

        // Rotate at specified angular velocity
        void motion_turn(double angular_vel) {
            m_Twist.linear.x = 0.0;
            m_Twist.angular.z = angular_vel;
            m_Cmd_Pub.publish(m_Twist);
        }

        // Move forward at specified velocity
        void motion_forward(double linear_vel) {
            m_Twist.angular.z = 0.0;
            m_Twist.linear.x = fabs(linear_vel);
            m_Cmd_Pub.publish(m_Twist);
        }

        // Move forward for a specified distance from the specified transform frame
        // Return true if specified distance has between travelled
       bool motion_forward(const geometry_msgs::TransformStamped& ref_tf, double distance, double linear_vel) {
            geometry_msgs::TransformStamped curr_tf;
            getOdomToBaseLinkTF(curr_tf);
            double travelled_distance = distanceApart(ref_tf, curr_tf);
            ROS_INFO("Travelled %f / %lf", travelled_distance, distance);
            if(fabs(travelled_distance) < fabs(distance)) {
                motion_forward(linear_vel);
                return false;
            } else {
                motion_stop();
                return true;
            }
        }

        // Stop all motion of the robot
        void motion_stop() {
            m_Twist.angular.x = 0.0;
            m_Twist.linear.x = 0.0;
            m_Cmd_Pub.publish(m_Twist);
        }

        // Get the transform between the /map and the /base_link coordinate frames
        bool getOdomToBaseLinkTF(geometry_msgs::TransformStamped& ts) {
            try {
                ts = m_TFBuffer.lookupTransform("odom", "base_link", ros::Time(0), ros::Duration(0.05));
                return true;
            } catch (tf2::TransformException& exp) {
                ROS_ERROR("Cannot find transform between '/odom' and 'base_link'");
            }
            return false;
        }

        void broadcast_fid_to_base_link_tf(const fiducial_msgs::FiducialTransform& ft) {
            geometry_msgs::TransformStamped ts;
            ts.header.frame_id = "raspicam";
            ts.child_frame_id = "fiducial";
            ts.transform.translation = ft.transform.translation;
            ts.transform.rotation = ft.transform.rotation;
            ts.header.stamp = ros::Time::now();
            m_TFBroadcaster.sendTransform(ts);
        }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "magni_docking");
    ros::NodeHandle nh;
    DockingController dc(nh);
    ros::spin();
    return 0;
}
