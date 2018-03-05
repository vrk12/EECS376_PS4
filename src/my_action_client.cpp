// action client: sends multi-step goals to action server (e.g. to move through the maze) 

//      monitors lidar alarm:     

//      if alarm, halts robot (cancels current goal), sends goal to rotate some amount               

//      resends a multi-step path goal 

// your action client should send multi-pose path commands to the action server 

// your action client should listen to the lidar alarm

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include<myAction.h> //reference action message in this package
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

bool g_goal_active = false; //some global vars for communication with callbacks
int g_result_output = -1;
int g_fdbk = -1;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const my_action_server::myResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result output = %d",result->output);
    g_result_output= result->output;
    g_goal_active=false;
}

//this function wakes up every time the action server has feedback updates for this client
// only the client that sent the current goal will get feedback info from the action server
void feedbackCb(const my_action_server::myFeedbackConstPtr& fdbk_msg) {
    ROS_INFO("feedback status = %d",fdbk_msg->fdbk);
    g_fdbk = fdbk_msg->fdbk; //make status available to "main()"
}

// Called once when the goal becomes active; not necessary, but could be useful diagnostic
void activeCb()
{
  ROS_INFO("Goal just went active");
  g_goal_active=true; //let main() know that the server responded that this goal is in process
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "timer_client_node"); // name this node 
        ros::NodeHandle n;
        ros::Rate main_timer(1.0);
        // here is a "goal" object compatible with the server, as defined in example_action_server/action
        my_action_server::demoGoal goal; 
        
        // use the name of our server, which is: timer_action (named in example_action_server_w_fdbk.cpp)
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        actionlib::SimpleActionClient<my_action_server::demoAction> action_client("my_action", true);
        
        // attempt to connect to the server: need to put a test here, since client might launch before server
        ROS_INFO("attempting to connect to server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(1.0)); // wait for up to 1 second
        // something odd in above: sometimes does not wait for specified seconds, 
        //  but returns rapidly if server not running; so we'll do our own version
        while (!server_exists) { // keep trying until connected
            ROS_WARN("could not connect to server; retrying...");
            server_exists = action_client.waitForServer(ros::Duration(1.0)); // retry every 1 second
        }
        ROS_INFO("connected to action server");  // if here, then we connected to the server;

        geometry_msgs::PoseStamped pose_stamped;
        geometry_msgs::Pose pose;
        pose.position.x = 1.0; // say desired x-coord is 1
        pose.position.y = 0.0;
        pose.position.z = 0.0; // let's hope so!
        pose.orientation.x = 0.0; //always, for motion in horizontal plane
        pose.orientation.y = 0.0; // ditto
        pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
        pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
        pose_stamped.pose = pose;
        goal.input = pose_stamped; //move right
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
        
        quat = convertPlanarPhi2Quaternion(0); // get a quaternion corresponding to this heading
        pose_stamped.pose.orientation = quat;   
        pose_stamped.pose.position.x=3.5; 
        goal.input = pose_stamped; //move right again
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
        
        quat = convertPlanarPhi2Quaternion(1.57); // get a quaternion corresponding to this heading
        pose_stamped.pose.orientation = quat;   
        pose_stamped.pose.position.y = 3.5;
        goal.input = pose_stamped; //spin and move up
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

        quat = convertPlanarPhi2Quaternion(0); // get a quaternion corresponding to this heading
        pose_stamped.pose.orientation = quat;   
        pose_stamped.pose.position.x = 7.5;
        goal.input = pose_stamped; //move right again
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

        quat = convertPlanarPhi2Quaternion(1.56); // get a quaternion corresponding to this heading
        pose_stamped.pose.orientation = quat;   
        pose_stamped.pose.position.y = 5.8;
        goal.input = pose_stamped; //spin and move up
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

        quat = convertPlanarPhi2Quaternion(3.14); // get a quaternion corresponding to this heading
        pose_stamped.pose.orientation = quat;   
        pose_stamped.pose.position.x = 3.5;
        goal.input = pose_stamped; //spin and move up
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

        quat = convertPlanarPhi2Quaternion(1.57); // get a quaternion corresponding to this heading
        pose_stamped.pose.orientation = quat;   
        pose_stamped.pose.position.y = 13.0;
        goal.input = pose_stamped; //spin and move up
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

        quat = convertPlanarPhi2Quaternion(3.14); // get a quaternion corresponding to this heading
        pose_stamped.pose.orientation = quat;   
        pose_stamped.pose.position.x = 1.0;
        goal.input = pose_stamped; //spin and move up
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
    return 0;
}

