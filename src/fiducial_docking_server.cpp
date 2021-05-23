#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <cmath>

using FSM_SC_States = FSM_State_Controller::States;

struct DockingParams {
    DockingParams() {
        if(!ros::param::get("~docking_position", m_Docking_Position)) {
            ROS_WARN("Docking Position not set, check launch file");
        }
        if(!ros::param::get("~linear_vel", m_Docking_Position)) {
            ROS_WARN("Linear velocity not set, check launch file");
        }
        if(!ros::param::get("~angular_vel", m_Docking_Position)) {
            ROS_WARN("Angular velocity not set, check launch file");
        }
        if(!ros::param::get("~search_angle", m_Docking_Position)) {
            ROS_WARN("Search angle not set, check launch file");
        }
        if(!ros::param::get("~docking_marker", m_Target_Fid)) {
            ROS_WARN("Target fiducial marker not set, check launch file");
        }
    }
    std::string m_Dock_Position;
    double m_Angular_Vel = 0.0;
    double m_Linear_Vel = 0.0;
    double m_Search_Angle = 0.0;
    int32_t m_Target_Fid;
};
 
struct Handles {
    Handles(ros::NodeHandle nh, DockingController* dc_ptr)
        :   m_Dock_Instr(nh.subscribe<std_msgs::String>("docking_command"), 1, 
                            &DockingController::Docking_Command_Callback, dc_ptr),
            m_Cmd_Pub(nh.advertise<geometry_msgs::Twist>("cmd_vel", 1)),
            m_FiducialsTF_Sub(nh.subscribe("fiducial_transforms", 1, 
                            &DockingController::Fiducial_Transforms_Callback, dc_ptr)),
            m_TFListener(m_TFBuffer) {}

    ros::Subscriber m_Docking_Command_Sub;
    ros::Publisher m_Cmd_Pub;
    ros::Subscriber m_FiducialsTF_Sub;
    tf2_ros::Buffer m_TFBuffer;
    tf2_ros::TransformListener m_TFListener;    
};

// todo - reset this somehow!
class Searching_Params {
    public:
        bool m_Found = false;
        geometry_msgs::Twist m_Twist;
        geometry_msgs::TransformStamped m_Benchmark_TF;
        geometry_msgs::TransformStamped m_Current_TF;
    private:
       
}

class FSM_State_Controller {
    public:
        enum class States {
            undocked, searching, centering, approaching, final_approach, docked, failed
        }
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
            return m_Error_msg;
        }
    private:
        States m_Current_State;
        std::string m_Error_Msg;
};

class DockingController {
    public:
        DockingController(ros::NodeHandle nh)
            :   m_Handles(nh, this),
                m_FSM_Timer(nh.createTimer(ros::Duration(0.1), &DockingController::Manage_FSM_State, this) {}

        // Callback for "docking_command" topic subscriber
        void Docking_Command_Callback(const std_msgs::String::ConstPtr& msg) {
            if(msg.data == "dock") {
                if(m_FSM_State_Controller.getState() == FSM_SC_States::docked) {
                    ROS_INFO("Robot is already docked");
                } else if(m_FSM_State_Controller.getState() == FSM_SC_States::undocked) {
                    m_FSM_State_Controller.setState(FSM_SC_States::searching);
                    // Reset the searching params
                    m_Searching_Params.m_Found = false;
                    getOdomToBaseLinkTF(m_Searching_Params.m_Benchmark_TF);
                }
            } else if(msg.data == "cancel") {
                // Cancel current goal
                FSM_SC_States current_state = m_FSM_State_Controller.getState();
                if(current_state == FSM_SC_States::docked || current_state == FSM_SC_States::undocked) {
                    ROS_INFO("%s is not in docking or undocking process", ros::this_node::getName().c_str());
                } else {
                    // todo -- cancel current docking or undocking goal
                }
            } else {
                ROS_ERROR("Unknown command to %s node", ros::this_node::getName().c_str());
            }
        }

        // Callback for "fiducial_transforms" topic subscriber
        void Fiducial_Transforms_Callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
            if(m_FSM_State_Controller.getState() == FSM_SC_States::searching) && !m_Searching_Params.m_Found) {
                for(std::size_t i = 0; i < (msg->transforms).size(); ++i) {
                    const fiducial_msgs::FiducialTransform& ft = msg->transforms[i];
                    if(ft.fiducial_id == m_Docking_Params.m_Target_Fid) {
                        m_Searching_Params.m_Found = true;
                    }
                }
            }
        }

        // Callback for FSM timer
        void Manage_FSM_State(const ros::TimerEvent& timer_event) {
            switch(m_FSM_State_Controller.getState()) {
                case FSM_SC_States::failed:
                    ROS_INFO("Docking failed: %s", m_FSM_State_Controller.errorMsg().c_str());
                    reset(); // todo -- implement a reset function!
                    break;
                case FSM_SC_States::undocked:
                    break;
                case FSM_SC_States::searching:
                    if(search_state_func()) {
                        if(m_Searching_Params.m_Found) {
                            m_FSM_State_Controller.setState(FSM_SC_States::centering);
                        } else {
                            m_FSM_State_Controller.setState(FSM_SC_States::failed);
                            m_FSM_State_Controller.errorMsg() = "Cannot find fiducial";
                        }
                    }
                    break;
                case FSM_SC_States::centering:
                    break;
            }
        }
    private:
        FSM_State_Controller m_FSM_State_Controller;
        DockingParams m_Docking_Params;
        Handles m_Handles;
        ros::Timer m_FSM_Timer;
        Searching_Params m_Searching_Params;

        // Check if the robot's rotation has exceeded the search limit. Return true if so.
        bool search_state_func() {
            // todo
            getOdomToBaseLinkTF(m_Searching_Params.m_Current_TF);
            double deg = degreesOfRotation(m_Searching_Params.m_Benchmark_TF, m_Searching_Params.m_Current_TF);
            if(deg < m_Docking_Params.m_Search_Angle) {
                return false;
            }
            return true;
        }

        // Get the transform between the /map and the /base_link coordinate frames
        bool getOdomToBaseLinkTF(geometry_msgs::TransformStamped& ts) {
            try {
                ts = m_Handles.m_TFBuffer.lookupTransform("odom", "base_link", ros::Time::now(), ros::Duration(0.05));
                return true;
            } catch (tf2::TransformException& exp) {
                ROS_ERROR("Cannot find transform between '/odom' and 'base_link'");
            }
            return false;
        }

        // Calculate the relative degrees of rotation between 2 coordinate frames
        double degreesOfRotation(const geometry_msgs::TransformStamped& benchmark_tf, const geometry_msgs::TransformStamped& curren_tf) {
            tf2::Quaternion benchmark_quat, current_quat;
            tf2::convert(benchmark_tf.transform.rotation, benchmark_quat);
            tf2::convert(current_tf.transform.rotation, current_quat);
            tf2::Quaternion diff = benchmark_quat * current_quat.inverse();
            double roll, pitch, yaw;
            tf2::Matrix3x3(diff).getRPY(roll, pitch, yaw);
            return abs(yaw);
        }
};



int main(int argc. char** argv) {
    ros::init(argc, argv, "magni_docking");
    ros::NodeHandle nh;
    return 0;
}