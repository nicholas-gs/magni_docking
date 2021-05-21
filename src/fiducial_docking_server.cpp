#include "actionlib/server/simple_action_server.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "magni_docking/DockingAction.h"
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <string>
#include <vector>
#include <cmath>

namespace magni_docking_impl {

    enum class Docking_State_Type { undocked,
                                    searching,
                                    docking,
                                    docked,
                                    failed };

    // Keep the internal state of the docking process
    Docking_State_Type docking_state;

    // Publisher to "/cmd_vel" to control movement
    ros::Publisher m_CmdPub;

    // Get the "/docking_status" param which tracks the if the robot is currently docked or undocked
    std::string getDockingStatus() {
        std::string docking_status;
        if (!ros::param::get("/docking_status", docking_status)) {
            ROS_ERROR("Docking status param not found! - Setting to undocked");
            docking_status = "undocked";
            ros::param::set("/docking_status", docking_status.c_str());
        }
        return docking_status;
    }

    // Initilize the publishers and subscribers, should be called in main()
    void init(ros::NodeHandle& nh) {
        m_CmdPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        // Initialise the state
        if (getDockingStatus() == std::string("undocked")) {
            docking_state = Docking_State_Type::undocked;
        } else {
            docking_state = Docking_State_Type::docked;
        }
    }

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

    // Retrieve all the rosparam
    struct DockingParams {
        DockingParams() {
            if(!ros::param::get("~dock_position", m_DockPosition)) {
                ROS_WARN("Dock position not set, check launch file");
            }
            if(!ros::param::get("~linear_vel", m_LinearVel)) {
                ROS_WARN("Linear velocity not set, check launch file");
            }
            if(!ros::param::get("~angular_vel", m_LinearVel)) {
                ROS_WARN("Linear velocity not set, check launch file");
            }
            if(!ros::param::get("~search_angle", m_SearchAngle)) {
                ROS_WARN("Search angle not set, check launch file");
            }
        }
        std::string m_DockPosition;
        double m_AngularVel = 0.0;
        double m_LinearVel = 0.0;
        double m_SearchAngle = 0.0;
    };

} // namespace impl

// The business logic of docking the robot
class DockingController {
  public:
    DockingController(int32_t target_fid, ros::NodeHandle& nh)
        :   m_Target_Fid(target_fid),
            m_FidTfSub(nh.subscribe("fiducial_transform", 1, &DockingController::FidTfCallback, this)),
            m_TFListener(m_TFBuffer) {
    }

    // Search for the target fiducial
    bool searchForFiducial() {
        ROS_INFO("Searching for %d fiducial", m_Target_Fid);
        // Rotate the robot for a fixed angle or until fiducial is found
        // Send Twist message to start rotation
        geometry_msgs::Twist twist;
        double ang_vel = m_DockingParams.m_AngularVel;
        // If the dock to the left of the robot, then rotate counterclockwise.
        twist.angular.z = m_DockingParams.m_DockPosition == std::string("left") ? ang_vel : -1 * ang_vel;
        magni_docking_impl::m_CmdPub.publish(twist);
        // Get the current transform between the /map and /base_link frames
        geometry_msgs::TransformStamped benchmark_tf;
        getOdomToBaseLinkTF(benchmark_tf);
        // Maximum rotation in terms of degrees
        double max_rotation = m_DockingParams.m_SearchAngle;
        geometry_msgs::TransformStamped current_tf = benchmark_tf;
        ros::Rate r(0.2);
        while(!m_Fiducial_Found && (magni_docking_impl::degreesOfRotation(benchmark_tf, current_tf) < max_rotation)) {
            r.sleep();
            if(!getOdomToBaseLinkTF(current_tf)) {
                break;
            }
        }
        twist.angular.z = 0.0;
        magni_docking_impl::m_CmdPub.publish(twist);
        // Test only to simulate 360 degree rotation, remove later
        //ros::Time later = ros::Time::now() + ros::Duration(10);
        //while ((later - ros::Time::now()).toSec() > 0) {
        //}
        return m_Fiducial_Found;
    }

    // Get the transform between the /map and /base_link frames
    bool getOdomToBaseLinkTF(geometry_msgs::TransformStamped& ts) {
        std::size_t i = 0;
        ros::Rate r(0.2);
        std::string odom = "odom";
        std::string base_link = "base_link";
        while(i < 3) {
            try {
                ts = m_TFBuffer.lookupTransform(odom, base_link, ros::Time(0));
                return true;
            } catch (tf2::TransformException& exp) {
                r.sleep();
                i++;
                continue;
            }
        }
        ROS_ERROR("Cannot find transform %s and %s", odom.c_str(), base_link.c_str());
        return false;
    }

  private:
    // Target fiducial
    const int32_t m_Target_Fid;
    // Subscriber to the incoming fiducial transforms
    ros::Subscriber m_FidTfSub;
    // Track if the target fiducial has been found
    bool m_Fiducial_Found = false;
    // Set up a transform listener
    tf2_ros::Buffer m_TFBuffer;
    tf2_ros::TransformListener m_TFListener;
    // Keep track of the docking params
    magni_docking_impl::DockingParams m_DockingParams;

    // Fiducial Transforms topic subscriber callback
    void FidTfCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
        // If we are currently searching for the target fiducial
        if (magni_docking_impl::docking_state == magni_docking_impl::Docking_State_Type::searching) {
            search(msg);
        }
    }

    // Iterate through all the FiducialTransforms in the FiducialTransformArray and search for target fiducial
    void search(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
        for (std::size_t i = 0; i < (msg->transforms).size(); ++i) {
            const fiducial_msgs::FiducialTransform& ft = msg->transforms[i];
            if (ft.fiducial_id == m_Target_Fid) {
                m_Fiducial_Found = true;
                ROS_INFO("Target fiducial %d has been found", m_Target_Fid);
                break;
            }
        }
    }
};

// Action Server
class DockingActionServer {
  public:
    DockingActionServer(ros::NodeHandle& nh, const std::string& name)
        :   m_NH(nh),
            m_Name(name),
            m_AS(m_NH, m_Name, boost::bind(&DockingActionServer::goalCallback, this, _1), false) {
        m_AS.start();
    }

    ~DockingActionServer() {}

  protected:
    ros::NodeHandle m_NH;
    // Name of the action server
    std::string m_Name;
    actionlib::SimpleActionServer<magni_docking::DockingAction> m_AS;
    magni_docking::DockingFeedback m_Feedback;
    magni_docking::DockingResult m_Result;

    // Callback when a goal is received by action server
    void goalCallback(const magni_docking::DockingGoalConstPtr& goal) {
        // Check if Magni is already docked. If it is, then do nothing
        if(magni_docking_impl::getDockingStatus() == std::string("docked")) {
            ROS_INFO("Magni already docked!");
            m_Result.success = true;
            m_AS.setSucceeded(m_Result);
            return;
        } else {
            magni_docking_impl::docking_state = magni_docking_impl::Docking_State_Type::undocked;
        }
        // Pass of responsibility of docking the robot to the docking controller
        // Search for the target fiducial
        const int32_t target_fid = goal->target_fiducial;
        DockingController dc(target_fid, m_NH);
        sendFeedback("searching");
        // If target fiducial could not be found, then abort
        magni_docking_impl::docking_state = magni_docking_impl::Docking_State_Type::searching;
        if (!dc.searchForFiducial()) {
            sendFeedback("Fiducial not found");
            ROS_INFO("Fiducial not found");
            m_Result.success = false;
            m_AS.setAborted(m_Result);
            return;
        }
        ROS_INFO("Fiducial found");

        // Successfully docked
        magni_docking_impl::docking_state = magni_docking_impl::Docking_State_Type::docked;
        ros::param::set("/docking_status", "docked");
        m_Result.success = true;
        m_AS.setSucceeded(m_Result);
    }

    // Publish docking action feedback
    void sendFeedback(const std::string& feedback) {
        m_Feedback.status = feedback;
        m_AS.publishFeedback(m_Feedback);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "magni_docking");
    ros::NodeHandle nh;
    magni_docking_impl::init(nh);
    DockingActionServer das(nh, "magni_docking");
    ros::spin();
    return 0;
}
