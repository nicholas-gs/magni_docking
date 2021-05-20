#include "actionlib/server/simple_action_server.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include "geometry_msgs/Twist.h"
#include "magni_docking/DockingAction.h"
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"

#include <string>
#include <vector>

namespace impl {

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
            ROS_INFO("Docking status param not found! - Setting to undocked");
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

} // namespace impl

// The business logic of docking the robot
class DockingController {
  public:
    DockingController(int32_t target_fid, ros::NodeHandle& nh)
        : m_Target_Fid(target_fid),
          m_FidTfSub(nh.subscribe("fiducial_transform", 1, &DockingController::FidTfCallback, this)),
          m_TFListener(m_TFBuffer)  {
    }

    // Search for the target fiducial
    bool searchForFiducial() {
        ROS_INFO("Searching for %d fiducial", static_cast<int>(m_Target_Fid));
        // Rotate the robot 360 degress or until fiducial is found

        // Test only to simulate 360 degree rotation, remove later
        ros::Time later = ros::Time::now() + ros::Duration(10);
        while ((later - ros::Time::now()).toSec() > 0) {
        }
        return m_Fiducial_Found;
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

    // Fiducial Transforms topic subscriber callback
    void FidTfCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
        // If we are currently searching for the target fiducial
        if (impl::docking_state == impl::Docking_State_Type::searching) {
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

class DockingActionServer {
  public:
    DockingActionServer(ros::NodeHandle& nh, const std::string& name)
        : m_NH(nh),
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
        if(impl::getDockingStatus() == std::string("docked")) {
            ROS_INFO("Magni already docked!");
            m_Result.success = true;
            m_AS.setSucceeded(m_Result);
            return;
        } else {
            impl::docking_state = impl::Docking_State_Type::undocked;
        }
        // Pass of responsibility of docking the robot to the docking controller
        // Search for the target fiducial
        const int32_t target_fid = goal->target_fiducial;
        DockingController dc(target_fid, m_NH);
        sendFeedback("searching");
        // If target fiducial could not be found, then abort
        impl::docking_state = impl::Docking_State_Type::searching;
        if (!dc.searchForFiducial()) {
            sendFeedback("fiducial not found");
            m_Result.success = false;
            m_AS.setAborted(m_Result);
            return;
        }

        // Successfully docked
        impl::docking_state = impl::Docking_State_Type::docked;
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
    impl::init(nh);
    DockingActionServer das(nh, "magni_docking");
    ros::spin();
    return 0;
}
