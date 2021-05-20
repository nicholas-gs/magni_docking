#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
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
    Docking_State_Type docking_state = Docking_State_Type::undocked;
}

// Docking controller -- Controls the movement of the robot
class DockingController
{
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

        // Fiducial Transforms topic subscriber callback
        void FidTfCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg)
        {
            // Searching for the target fiducial
            if(impl::docking_state == impl::Docking_State_Type::searching)
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
        }

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
            impl::docking_state = impl::Docking_State_Type::searching;
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

};

// Docking Action Server
class DockingServer
{
    protected:
        ros::NodeHandle m_NH;
        actionlib::SimpleActionServer<magni_docking::DockingAction> m_AS;
        std::string m_Name;
        magni_docking::DockingFeedback m_Feedback;
        magni_docking::DockingResult m_Result;

    public:

        DockingServer(std::string name)
            : m_AS(m_NH, name, boost::bind(&DockingServer::goalCB, this, _1), false),
              m_Name(name)
            {
                m_AS.start();
            }

        // Callback when goal is received
        void goalCB(const magni_docking::DockingGoalConstPtr &goal)
        {
            int32_t target_fid = goal->target_fiducial;
            ROS_INFO("Goal received: %d", static_cast<int>(target_fid));
            DockingController dc(m_NH, target_fid);
            m_Feedback.status = "searching";
            m_AS.publishFeedback(m_Feedback);

            // Has the target fiducial been found?
            bool found_fid = dc.searching();
            if(!found_fid)
            {
                // If fiducial cannot be found, then abort
                m_Feedback.status = "fiducial not found";
                m_AS.publishFeedback(m_Feedback);
                m_Result.success = false;
                m_AS.setAborted(m_Result);
                return;
            }
            m_Feedback.status = "docking";
            m_AS.publishFeedback(m_Feedback);


            // End
            m_Result.success = true;
            m_AS.setSucceeded(m_Result);
        }

        ~DockingServer()
        {}

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "magni_docking");
    DockingServer dockingServer("magni_docking");
    ros::spin();
    return 0;
}
