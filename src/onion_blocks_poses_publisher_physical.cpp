// this node will publish the topic "onions_blocks_poses"
// including all current onions blocks, pose is 3-D position

// ros communication:
// publish the topic "/onions_blocks_poses"

#include <ros/ros.h>
#include <vector>
#include <string>
#include <std_msgs/Int8MultiArray.h>
#include <sawyer_irl_project/onions_blocks_poses.h>
using namespace std;
// global variables
int g_onions_quantity;
std_msgs::Int8MultiArray g_current_onions_blocks;
vector<double> g_x;
vector<double> g_y;
vector<double> g_z;
double SAWYERRANGE_UPPER_LIMIT, initial_pose_x, initial_pose_y, height_spawning;
int goodonionsConvEnd = 0;
int badonionsConvEnd = 0;
int goodonionsInBin = 0;
int badonionsInBin = 0;

// More Globals
array<array<int, 3>, 4> current_model_states;

string intToString(int a)
{
    stringstream ss;
    ss << a;
    return ss.str();
}

void modelStatesCallback()
{
    // cout<<"I'm in callback!";
    g_onions_quantity = 4;
    g_current_onions_blocks.data.push_back(1);
    g_current_onions_blocks.data.push_back(0);
    g_current_onions_blocks.data.push_back(1);
    g_current_onions_blocks.data.push_back(0);
    current_model_states[0] = {0,0,0};
    current_model_states[1] = {0,0,0};
    current_model_states[2] = {0,0,0};
    current_model_states[3] = {0,0,0};
    vector<double> onions_x;
    vector<double> onions_y;
    vector<double> onions_z;
    onions_x.resize(g_onions_quantity);
    onions_y.resize(g_onions_quantity);
    onions_z.resize(g_onions_quantity);
    // find position of all current onions in topic message
    bool onions_poses_completed = true;
    int ind_del, prev_index = -1;
    for (int i = 0; i < g_onions_quantity; i++)
    {

        // get index of ith onions
        string indexed_model_name;
        if (g_current_onions_blocks.data[i] == 1)
        { //If color variable is 0, good onion else bad
            indexed_model_name = "good_onion_" + intToString(i);
        }
        else
        {
            indexed_model_name = "bad_onion_" + intToString(i);
        }
        
        int model_quantity = 4; // number of models measured
        if (i != model_quantity)
        {
            // this model name exists and has been successfully indexed
            onions_x[i] = current_model_states[i][0];
            onions_y[i] = current_model_states[i][1];
            onions_z[i] = current_model_states[i][2];
            g_x.resize(g_onions_quantity);
            g_y.resize(g_onions_quantity);
            g_z.resize(g_onions_quantity);
            g_x = onions_x;
            g_y = onions_y;
            g_z = onions_z;
        }
    }
    // cout<<"\nI'm leaving callback!\n";
}

int main(int argc, char **argv)
{
    initial_pose_x = 0.75;
    initial_pose_y = -0.59;
    height_spawning = 0.83;
    SAWYERRANGE_UPPER_LIMIT = 0.55;
    ros::init(argc, argv, "onions_blocks_poses_publisher");
    ros::NodeHandle nh;
    modelStatesCallback();
    // initialize publisher for "/onions_blocks_poses"
    ros::Publisher onions_poses_publisher = nh.advertise<sawyer_irl_project::onions_blocks_poses>("onions_blocks_poses_physical", 1);
    ros::Publisher current_onions_publisher = nh.advertise<std_msgs::Int8MultiArray>("current_onions_blocks", 1);
    // publisher for /current_onions
    std_msgs::Int8MultiArray current_onions_msg;
    current_onions_msg.data.clear();
    sawyer_irl_project::onions_blocks_poses current_poses_msg;
    // cout << "\ng_onions_poses_updated: " << g_onions_poses_updated;
    while (ros::ok())
    {
        // only publish when onions positions are updated
        // no need to publish repeated data
        // there is tiny possibility that g_x is not in the length of g_onions_quantity
        int local_onions_quantity = g_x.size(); // so get length of g_x
        current_poses_msg.x.resize(local_onions_quantity);
        current_poses_msg.y.resize(local_onions_quantity);
        current_poses_msg.z.resize(local_onions_quantity);
        current_poses_msg.x = g_x;
        current_poses_msg.y = g_y;
        current_poses_msg.z = g_z;
        //ROS_INFO_STREAM("Onion current pose: "<<current_poses_msg);
        current_onions_msg = g_current_onions_blocks;
        current_onions_publisher.publish(current_onions_msg);
        onions_poses_publisher.publish(current_poses_msg);
        // cout<<"I just published!";
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    return 0;
}

