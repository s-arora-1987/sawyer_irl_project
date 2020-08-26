// Author: Prasanth Suresh(ps32611@uga.edu); 
// Description: Adding surrounding collision objects to sawyer world in Moveit; 
// Do not edit/copy without permission.


#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/Int32.h"

using namespace Eigen;

//Trying to add dynamic collision objects as they are spawned. Yet to be cleaned up./////////////

/*
 void callbackAddObject(const std_msgs::Int32::ConstPtr& msg){

   std::string out_string;
   std::stringstream ss;
   ss << i;
   out_string = ss.str();

   shapes::Mesh* m = shapes::createMeshFromResource("file:///home/saurabharora/catkin_ws/src/sawyer_irl_project/meshes/custom_onion.STL"); 
   shape_msgs::Mesh mesh;
   shapes::ShapeMsg mesh_msg;  
   shapes::constructMsgFromShape(m, mesh_msg);
   mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
 
   moveit_msgs::AttachedCollisionObject attached_object;
   attached_object.link_name = modelname;
   attached_object.object.header.frame_id = "world";
   attached_object.object.id = "onion"+out_string;

   moveit_msgs::PlanningScene planning_scene;
   planning_scene.world.collision_objects.push_back(attached_object.object);
   planning_scene.is_diff = true;
   planning_scene_diff_publisher.publish(planning_scene);
 }
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////



void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold num_objects number of collision objects.
  int num_objects = 5;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(num_objects);

  // Add the first table where the object will originally be kept.
  collision_objects[0].id = "conveyor_table";
  collision_objects[0].header.frame_id = "world";
  shapes::Mesh* m = shapes::createMeshFromResource("package://sawyer_irl_project/meshes/conveyor_box.stl"); 
  ROS_INFO("Conveyor box mesh loaded");
  shape_msgs::Mesh mesh;
  shapes::ShapeMsg mesh_msg;  
  shapes::constructMsgFromShape(m, mesh_msg);
  mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
  collision_objects[0].meshes.resize(1); 
  collision_objects[0].mesh_poses.resize(1);  
  collision_objects[0].mesh_poses[0].position.x = 0.75;
  collision_objects[0].mesh_poses[0].position.y = -0.75;
  collision_objects[0].mesh_poses[0].position.z = -0.91488;
  collision_objects[0].mesh_poses[0].orientation.w= 1.0; 
  collision_objects[0].mesh_poses[0].orientation.x= 0.0; 
  collision_objects[0].mesh_poses[0].orientation.y= 0.0;
  collision_objects[0].mesh_poses[0].orientation.z= 0.0;   
  collision_objects[0].meshes.push_back(mesh);
  collision_objects[0].mesh_poses.push_back(collision_objects[0].mesh_poses[0]);
  collision_objects[0].operation = collision_objects[0].ADD;

  
 //Add sawyer_lab as one stl mesh
  collision_objects[1].id = "sawyer_lab";
  collision_objects[1].header.frame_id = "world";

  // Define the primitive and its dimensions. 
  Vector3d b(1, 1, 1);
  shapes::Mesh* m1 = shapes::createMeshFromResource("package://sawyer_irl_project/meshes/sawyer_lab.stl",b); 
  ROS_INFO("Sawyer lab mesh loaded");
  shape_msgs::Mesh mesh1;
  shapes::ShapeMsg mesh_msg1;  
  shapes::constructMsgFromShape(m1, mesh_msg1);
  mesh1 = boost::get<shape_msgs::Mesh>(mesh_msg1);
  collision_objects[1].meshes.resize(1);
  collision_objects[1].mesh_poses.resize(1);  
  collision_objects[1].mesh_poses[0].position.x = -0.8;
  collision_objects[1].mesh_poses[0].position.y = -1.45;
  collision_objects[1].mesh_poses[0].position.z = -0.91488;
  collision_objects[1].mesh_poses[0].orientation.w= 1.0; 
  collision_objects[1].mesh_poses[0].orientation.x= 0.0; 
  collision_objects[1].mesh_poses[0].orientation.y= 0.0;
  collision_objects[1].mesh_poses[0].orientation.z= 0.0;   
  collision_objects[1].meshes.push_back(mesh1);
  collision_objects[1].mesh_poses.push_back(collision_objects[1].mesh_poses[0]);
  collision_objects[1].operation = collision_objects[1].ADD;

  // Add the onion_bin where the sorted onions will be kept.
  collision_objects[2].id = "onion_bin";
  collision_objects[2].header.frame_id = "world";
  shapes::Mesh* m2 = shapes::createMeshFromResource("package://sawyer_irl_project/meshes/onion_bin.stl"); 
  ROS_INFO("Onion bin mesh loaded");
  shape_msgs::Mesh mesh2;
  shapes::ShapeMsg mesh_msg2;  
  shapes::constructMsgFromShape(m2, mesh_msg2);
  mesh2 = boost::get<shape_msgs::Mesh>(mesh_msg2);
  collision_objects[2].meshes.resize(1); 
  collision_objects[2].mesh_poses.resize(1);  
  collision_objects[2].mesh_poses[0].position.x = 0.1;
  collision_objects[2].mesh_poses[0].position.y = 0.6;
  collision_objects[2].mesh_poses[0].position.z = -0.91488;
  collision_objects[2].mesh_poses[0].orientation.w= 1.0; 
  collision_objects[2].mesh_poses[0].orientation.x= 0.0; 
  collision_objects[2].mesh_poses[0].orientation.y= 0.0;
  collision_objects[2].mesh_poses[0].orientation.z= 0.0;   
  collision_objects[2].meshes.push_back(mesh2);
  collision_objects[2].mesh_poses.push_back(collision_objects[2].mesh_poses[0]);
  collision_objects[2].operation = collision_objects[2].ADD;

  // Add the railing1 above the conveyor.
  collision_objects[3].id = "railing1";
  collision_objects[3].header.frame_id = "world";
  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.01;
  primitive.dimensions[1] = 1.5;
  primitive.dimensions[2] = 0.01;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.6;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.09488;

  collision_objects[3].primitives.push_back(primitive);
  collision_objects[3].primitive_poses.push_back(box_pose);
  collision_objects[3].operation = collision_objects[3].ADD;
  ROS_INFO("Railing 1 loaded");

  // Add the railing2 above the conveyor.
  collision_objects[4].id = "railing2";
  collision_objects[4].header.frame_id = "world";
  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive1;
  primitive1.type = primitive1.BOX;
  primitive1.dimensions.resize(3);
  primitive1.dimensions[0] = 0.01;
  primitive1.dimensions[1] = 1.5;
  primitive1.dimensions[2] = 0.01;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose1;
  box_pose1.orientation.w = 1.0;
  box_pose1.position.x =  0.9;
  box_pose1.position.y = 0.0;
  box_pose1.position.z = -0.09488;

  collision_objects[4].primitives.push_back(primitive1);
  collision_objects[4].primitive_poses.push_back(box_pose1);
  collision_objects[4].operation = collision_objects[4].ADD;
  ROS_INFO("Railing 2 loaded");

  ros::Duration(1.0).sleep();
  planning_scene_interface.applyCollisionObjects(collision_objects);
  sleep(4.0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"planning_scene_sawyer");
  ros::NodeHandle nh;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("right_arm");
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
  ros::WallDuration sleep_t(0.5);
  sleep_t.sleep();
  }
  ros::WallDuration(1.0).sleep();  
  
  group.setPlanningTime(60.0);
  group.setPlannerId(group.getDefaultPlannerId(group.getName()));
  addCollisionObjects(planning_scene_interface);
  
  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();
  ROS_INFO("Now launch ROS Moveit to start planning");
  ros::waitForShutdown();
  return 0;
}
