#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "magni_docking/DockingAction.h"
#include "geometry_msgs/Twist.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include "tf2_ros/transform_listener.h"

#include <string>
#include <vector>

namespace impl
{
    enum class Docking_State_Type
    { undocked, searching, not_found, docking, docked, failed };
    // Keep the internal state of the docking process
    Docking_State_Type docking_state;

    // Get the "/docking_status" param which tracks the if the robot is currently docked or undocked
    std::string getDockingStatus()
    {
        std::string docking_status;
        if(!ros::param::get("/docking_status", docking_status))
        {
            ROS_INFO("Docking status param not found! - Setting to undocked");
            docking_status = "undocked";
            ros::param::set("/docking_status", docking_status.c_str());
        }
        return docking_status;
    }
}

// Docking controller -- Controls the movement of the robot during the docking process
class DockingController
{
    public:

        DockingController(ros::NodeHandle& nh, int32_t target_fid)
            : m_Target_Fid(target_fid),
              m_CmdPub(nh.advertise<geometry_msgs::Twist>("cmd_vel", 1)),
              m_FidTfSub(nh.subscribe("fiducial_transforms", 1, &DockingController::FidTfCallback, this)),
              m_TFListener(m_TFBuffer)
        {}

        // Search for the target fiducial
        bool searching()
        {
            ROS_INFO("Searching for %d fiducial", static_cast<int>(m_Target_Fid));
            // Rotate the robot 360 degress or until fiducial is found

            // Test only to simulate 360 degree rotation, remove later
            ros::Time later =  ros::Time::now() + ros::Duration(10);
            while((later - ros::Time::now()).toSec() > 0)
            {}
            return target_found;
        }

        ~DockingController()
        {}

    private:

        // Target fiducial
        int32_t m_Target_Fid;
        // Publisher to "/cmd_vel" to control movement
        ros::Publisher m_CmdPub;
        // Subscriber to the incoming fiducial transforms
        ros::Subscriber m_FidTfSub;
        // Set up a transform listener
        tf2_ros::Buffer m_TFBuffer;
        tf2_ros::TransformListener m_TFListener;
        // Has the target fiducial been found?
        bool target_found = false;

        // Rotate the robot on the spot
        void rotate()
        {

        }

        // Iterate through all the FiducialTransforms in the FiducialTransformArray and search for target fiducial
        void searchForFiducial(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
        {
            for(std::size_t i = 0; i < (msg->transforms).size(); ++i)
            {
                const fiducial_msgs::FiducialTransform& ft = msg->transforms[i];
                if(ft.fiducial_id == m_Target_Fid)
                {
                    target_found = true;
                    ROS_INFO("Target fiducial %d has been found", m_Target_Fid);
                    break;
                }
            }
        }

        // Fiducial Transforms topic subscriber callback
        void FidTfCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
        {
            // If we are currently searching for the target fiducial
            if(impl::docking_state == impl::Docking_State_Type::searching)
            {
                searchForFiducial(msg);
            }
        }

};

// Docking Action Server
class DockingServer
{
    public:

        DockingServer(ros::NodeHandle nh, std::string name)
            : m_NH(nh),
              m_AS(m_NH, name, boost::bind(&DockingServer::goalCB, this, _1), false),
              m_Name(name)
            {
                m_AS.start();
            }

        // Callback when goal is received
        void goalCB(const magni_docking::DockingGoalConstPtr &goal)
        {
            // Check if Magni is already docked. If it is, then do nothing
            if(impl::getDockingStatus() == std::string("docked"))
            {
                ROS_INFO("Magni is already docked!");
                return;
            }
            impl::docking_state = impl::Docking_State_Type::undocked;
            // Search for the target fiducial
            int32_t target_fid = goal->target_fiducial;
            ROS_INFO("Fiducial Goal received: %d", static_cast<int>(target_fid));
            DockingController dc(m_NH, target_fid);
            sendFeedback("searching");
            impl::docking_state = impl::Docking_State_Type::searching;
            bool found_fid = dc.searching();
            // If fiducial cannot be found, then abort
            if(!found_fid)
            {
                sendFeedback("fiducial not found");
                m_Result.success = false;
                m_AS.setAborted(m_Result);
                return;
            }
            // Fiducial was found, so let's start moving the robot towards it
            sendFeedback("docking");

            // End
            ros::param::set("/docking_status", "docked");
            m_Result.success = true;
            m_AS.setSucceeded(m_Result);
        }

        ~DockingServer()
        {}

    private:

        ros::NodeHandle m_NH;
        actionlib::SimpleActionServer<magni_docking::DockingAction> m_AS;
        // Name of the action server
        std::string m_Name;
        magni_docking::DockingFeedback m_Feedback;
        magni_docking::DockingResult m_Result;

        // Publish docking action feedback
        void sendFeedback(const std::string& feedback)
        {
            m_Feedback.status = feedback;
            m_AS.publishFeedback(m_Feedback);
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "magni_docking");
    ros::NodeHandle nh;
    // Initialise the state
    if(impl::getDockingStatus() == std::string("undocked"))
    {
        impl::docking_state = impl::Docking_State_Type::undocked;
    } else
    {
        impl::docking_state = impl::Docking_State_Type::docked;
    }
    DockingServer dockingServer(nh, "magni_docking");
    ros::spin();
    return 0;
}
