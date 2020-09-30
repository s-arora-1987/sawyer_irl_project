// Author: Prasanth Suresh(ps32611@uga.edu); 
// Description: We  have good onions and bad onions. 
// Aim of this script is to spawn them on the conveyor surface.

//STILL UNDER CONSTRUCTION!
// Do not edit/copy without permission.

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_srvs/Trigger.h>
#include <sawyer_irl_project/StringArray.h>

using namespace std;
// int to string converter
string intToString(int a) {
    stringstream ss;
    ss << a;
    return ss.str();
}

int main(int argc, char **argv) {

ros::init(argc, argv, "onion_blocks_spawner");
    ros::NodeHandle nh; //("~");
    int i = 0 /*index the onions*/,j = 0 /*position reference for onions*/; 
    int onion_gen = 0;
    double initial_pose_x, initial_pose_y, height_spawning, spawning_interval, 
        conveyor_center_x, belt_width,wrench_duration, randpos, object_width;;
    bool spawn_multiple;
    string good_onion_path,bad_onion_path,OnionCenter_path,Onion0_path, 
        good_xmlStr, bad_xmlStr, OnionCenter_xmlStr, Onion0_xmlStr, model_name;
    int8_t color_index;  // 0 is good, 1 is bad
    ifstream good_inXml, bad_inXml;
    stringstream good_strStream, bad_strStream, OnionCenter_strStream, Onion0_strStream;
    nh.getParam("/initial_pose_x", initial_pose_x);
    nh.getParam("/initial_pose_y", initial_pose_y);
    nh.getParam("/height_spawning", height_spawning);
    nh.getParam("/spawning_interval", spawning_interval);
    nh.getParam("/conveyor_center_x", conveyor_center_x);
    nh.getParam("/belt_width", belt_width);
    nh.getParam("/spawn_multiple", spawn_multiple);
    nh.getParam("/object_width", object_width);
	// get file path of onions from parameter server
    bool get_good_onion_path, get_bad_onion_path, get_OnionCenter_path, get_Onion0_path;
    get_good_onion_path = nh.getParam("/good_onion_path", good_onion_path);
    get_bad_onion_path = nh.getParam("/bad_onion_path", bad_onion_path);
    get_OnionCenter_path = nh.getParam("/OnionCenter_path", OnionCenter_path);
    get_Onion0_path = nh.getParam("/OnionCenter_path",Onion0_path);

    ros::ServiceClient spawn_model_client
        = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel spawn_model_srv_msg;  // service message
    ros::ServiceClient delete_model_client
        = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    gazebo_msgs::DeleteModel delete_model_srv_msg;  // service message
    

	// make sure /gazebo/spawn_sdf_model service is ready
    bool service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/spawn_sdf_model", true);
        ROS_INFO("waiting for spawn_sdf_model service");
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("spawn_sdf_model service is ready");

    if (!(get_good_onion_path && get_bad_onion_path && get_OnionCenter_path && get_Onion0_path))
    {
        ROS_INFO("failed to get parameters");
        return 0;  // return if fail to get parameters
    }

	// prepare the xml for service call, read sdf into string
    // good onion
    good_inXml.open(good_onion_path.c_str());
    good_strStream << good_inXml.rdbuf();
    good_xmlStr = good_strStream.str();
    // bad onion
    bad_inXml.open(bad_onion_path.c_str());
    bad_strStream << bad_inXml.rdbuf();
    bad_xmlStr = bad_strStream.str();
	
	ifstream OnionCenter_inXml(OnionCenter_path.c_str());
    OnionCenter_strStream << OnionCenter_inXml.rdbuf();
    OnionCenter_xmlStr = OnionCenter_strStream.str();
    ifstream Onion0_inXml(Onion0_path.c_str());
    Onion0_strStream << Onion0_inXml.rdbuf();
    Onion0_xmlStr = Onion0_strStream.str();
	// prepare the spawn model service message
    spawn_model_srv_msg.request.initial_pose.position.x = initial_pose_x; // 4.0
    spawn_model_srv_msg.request.initial_pose.position.y = initial_pose_y; // 4.0
    spawn_model_srv_msg.request.initial_pose.position.z = height_spawning; // 0.2 ; on the conveyor belt
	spawn_model_srv_msg.request.initial_pose.orientation.x = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.y = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.z = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.w = 1.0;
    spawn_model_srv_msg.request.reference_frame = "world";

	//begin spawning onions

        // publisher for /modelnames
        ros::Publisher modelnames_publisher
            = nh.advertise<sawyer_irl_project::StringArray>("modelnames", 1);

        ros::Publisher current_onions_publisher
            = nh.advertise<std_msgs::Int8MultiArray>("current_onions_blocks", 1);

        sawyer_irl_project::StringArray modelnames_msg;
        modelnames_msg.modelnames.clear();
        // publisher for /current_onions
        std_msgs::Int8MultiArray current_onions_msg;
        current_onions_msg.data.clear();

    ros::Duration(1).sleep();
    while (ros::ok()) {


        while(i < 4) {
            string index_string = intToString(i);
            ++j;
            if(i%2 == 0){
                spawn_model_srv_msg.request.initial_pose.position.x = 0.87 - j*0.05; //width of sphere is 0.02, well within 0.05
                spawn_model_srv_msg.request.initial_pose.position.y = -0.65 + j*0.05; //width of sphere is 0.02, well within 0.05
            }
            else{
                spawn_model_srv_msg.request.initial_pose.position.x = 0.87 - j*0.05;
                spawn_model_srv_msg.request.initial_pose.position.y = -0.65;
            }
            ROS_INFO_STREAM("x position of new onion: "
                << spawn_model_srv_msg.request.initial_pose.position.x);
            ROS_INFO_STREAM("y position of new onion: "
                << spawn_model_srv_msg.request.initial_pose.position.y);

            /* initialize random seed: */
            srand (time(NULL));

            /* generate random number between 1 and 10: */
            //onion_gen = std::rand() % 100;
	        onion_gen = i;


            if((onion_gen % 2) == 0){
                cout<<"Generating good onion";
                color_index = 1;
                model_name = "good_onion_" + index_string;  // initialize model_name
                spawn_model_srv_msg.request.model_name = model_name;
                spawn_model_srv_msg.request.robot_namespace = "good_onion_" + index_string;
                spawn_model_srv_msg.request.model_xml = good_xmlStr;

            }

            else{
                cout<<"Generating bad onion";
                color_index = 0;
                model_name = "bad_onion_" + index_string;  // initialize model_name
                spawn_model_srv_msg.request.model_name = model_name;
                spawn_model_srv_msg.request.robot_namespace = "bad_onion_" + index_string;
                spawn_model_srv_msg.request.model_xml = bad_xmlStr;

            }
                
            // call spawn model service
            bool call_service = spawn_model_client.call(spawn_model_srv_msg);
            if (call_service) {
                if (spawn_model_srv_msg.response.success) {
                    ROS_INFO_STREAM(model_name << " has been spawned");
                }
                else {
                    ROS_INFO_STREAM(model_name << " spawn failed");
                }
            }
            else {
                ROS_INFO("fail in first call");
                ROS_ERROR("fail to connect with gazebo server");
                return 0;
            }
            // publish current onion blocks status, all onion blocks will be published
            // no matter if it's successfully spawned, or successfully initialized in speed
            current_onions_msg.data.push_back(color_index);
            
			//ROS_INFO_STREAM("Current onion poses: "<< current_onions_msg);
            modelnames_msg.modelnames.push_back(spawn_model_srv_msg.request.model_name);
            
            // loop end, increase index by 1
            i++;
        }
        current_onions_publisher.publish(current_onions_msg);
        modelnames_publisher.publish(modelnames_msg);
        //ros::Duration(5).sleep();
    ROS_INFO_STREAM("Current onion color indices"<<current_onions_msg);	//Debug print statement
    ROS_INFO_STREAM("Current Model names"<<modelnames_msg);	//Debug print statement
    ros::spinOnce();
    ros::Duration(spawning_interval).sleep(); // frequency control, spawn one onion in each loop
    // delay time decides density of the onions
    }
    /*if(!ros::ok()){
        for(int p = 0; p < i; p++){
            model_name = "good_onion_" + intToString(p);  // initialize model_name
            // call delete model service
            spawn_model_srv_msg.request.model_name = model_name;
            bool call_service = delete_model_client.call(delete_model_srv_msg);
            if (call_service) {
                if (delete_model_srv_msg.response.success) {
                    ROS_INFO_STREAM("Deleted");
                }
                else {
                    ROS_INFO_STREAM("Delete failed");
                }
            }   
        }
    }*/

return 0;
}
