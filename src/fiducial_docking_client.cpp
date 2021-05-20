#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "magni_docking/DockingAction.h"

void doneCB(const actionlib::SimpleClientGoalState& state,
            const magni_docking::DockingResultConstPtr& result)
{
    ROS_INFO("In client Finished in state [%s]", state.toString().c_str());
    ROS_INFO("In Result: %d", result->success);
    ros::shutdown();
}

void activeCB()
{
    ROS_INFO("In client, Goal just went active");
}

void feedbackCB(const magni_docking::DockingFeedbackConstPtr& feedback)
{
    ROS_INFO("In client, Got feedback of %s", (feedback->status).c_str());
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_magni_docking");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<magni_docking::DockingAction> ac("magni_docking", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  magni_docking::DockingGoal goal;
  goal.target_fiducial = 49;
  ac.sendGoal(goal, &doneCB, &activeCB, &feedbackCB);
  ros::spin();
  //exit
  return 0;
}
