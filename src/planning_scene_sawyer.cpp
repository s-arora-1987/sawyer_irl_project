#include <ros/ros.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace Eigen;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 1 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "conveyor_table";
  collision_objects[0].header.frame_id = "world";
  ROS_INFO("Conveyor table loaded successfully");

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

/*
  // Define the primitive and its dimensions. 
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.6;
  collision_objects[0].primitives[0].dimensions[1] = 1.5;
  collision_objects[0].primitives[0].dimensions[2] = 0.75;

  // Define the pose of the table. 
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.75;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = -0.5; 

  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  collision_objects[0].primitive_poses[0].orientation.x = 0;
  collision_objects[0].primitive_poses[0].orientation.y = 0;
  collision_objects[0].primitive_poses[0].orientation.z = 0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;
  
  // Add the rear table behind Sawyer.
  collision_objects[1].id = "rear_table";
  collision_objects[1].header.frame_id = "world";

  // Define the primitive and its dimensions. 
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.5; //0.66
  collision_objects[1].primitives[0].dimensions[1] = 3.0;
  collision_objects[1].primitives[0].dimensions[2] = 0.04;

  // Define the pose of the rear table. 
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -0.45;
  collision_objects[1].primitive_poses[0].position.y = 0;
  collision_objects[1].primitive_poses[0].position.z = -0.0; //0.10

  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  collision_objects[1].primitive_poses[0].orientation.x = 0;
  collision_objects[1].primitive_poses[0].orientation.y = 0;
  collision_objects[1].primitive_poses[0].orientation.z = 0;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // Add the rear wall behind Sawyer.
  collision_objects[2].id = "rear_wall";
  collision_objects[2].header.frame_id = "world";

  // Define the primitive and its dimensions. 
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.04;
  collision_objects[2].primitives[0].dimensions[1] = 3.0;
  collision_objects[2].primitives[0].dimensions[2] = 2.5;

  // Define the pose of the rear wall. 
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = -0.77;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.33;

  collision_objects[2].primitive_poses[0].orientation.w = 1.0;
  collision_objects[2].primitive_poses[0].orientation.x = 0;
  collision_objects[2].primitive_poses[0].orientation.y = 0;
  collision_objects[2].primitive_poses[0].orientation.z = 0;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  // Add the side table to the right of Sawyer.
  collision_objects[3].id = "right_table";
  collision_objects[3].header.frame_id = "world";

  // Define the primitive and its dimensions. 
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 3.0;
  collision_objects[3].primitives[0].dimensions[1] = 0.64;
  collision_objects[3].primitives[0].dimensions[2] = 0.04;

  // Define the pose of the right table. 
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.75;
  collision_objects[3].primitive_poses[0].position.y = -1.2;
  collision_objects[3].primitive_poses[0].position.z = -0.10;

  collision_objects[3].primitive_poses[0].orientation.w = 1.0;
  collision_objects[3].primitive_poses[0].orientation.x = 0;
  collision_objects[3].primitive_poses[0].orientation.y = 0;
  collision_objects[3].primitive_poses[0].orientation.z = 0;
  // END_SUB_TUTORIAL

  collision_objects[3].operation = collision_objects[3].ADD;

  // Add the right side wall - Sawyer.
  collision_objects[4].id = "right_wall";
  collision_objects[4].header.frame_id = "world";

  // Define the primitive and its dimensions. 
  collision_objects[4].primitives.resize(1);
  collision_objects[4].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[4].primitives[0].dimensions.resize(3);
  collision_objects[4].primitives[0].dimensions[0] = 3.0;
  collision_objects[4].primitives[0].dimensions[1] = 0.04;
  collision_objects[4].primitives[0].dimensions[2] = 2.5;

  // Define the pose of the right wall. 
  collision_objects[4].primitive_poses.resize(1);
  collision_objects[4].primitive_poses[0].position.x = 0.75;
  collision_objects[4].primitive_poses[0].position.y = -1.5;
  collision_objects[4].primitive_poses[0].position.z = 0.33;

  collision_objects[4].primitive_poses[0].orientation.w = 1.0;
  collision_objects[4].primitive_poses[0].orientation.x = 0;
  collision_objects[4].primitive_poses[0].orientation.y = 0;
  collision_objects[4].primitive_poses[0].orientation.z = 0;
  // END_SUB_TUTORIAL

  collision_objects[4].operation = collision_objects[4].ADD;

  // Add the front pillar.
  collision_objects[5].id = "front_pillar";
  collision_objects[5].header.frame_id = "world";

  // Define the primitive and its dimensions. 
  collision_objects[5].primitives.resize(1);
  collision_objects[5].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[5].primitives[0].dimensions.resize(3);
  collision_objects[5].primitives[0].dimensions[0] = 0.425;
  collision_objects[5].primitives[0].dimensions[1] = 0.36;
  collision_objects[5].primitives[0].dimensions[2] = 2.5;

  // Define the pose of the front pillar. 
  collision_objects[5].primitive_poses.resize(1);
  collision_objects[5].primitive_poses[0].position.x = 1.4;
  collision_objects[5].primitive_poses[0].position.y = 0.85;
  collision_objects[5].primitive_poses[0].position.z = 0.33;

  collision_objects[5].primitive_poses[0].orientation.w = 1.0;
  collision_objects[5].primitive_poses[0].orientation.x = 0;
  collision_objects[5].primitive_poses[0].orientation.y = 0;
  collision_objects[5].primitive_poses[0].orientation.z = 0;
  // END_SUB_TUTORIAL

  collision_objects[5].operation = collision_objects[5].ADD;

  // Add the cross beam.
  collision_objects[6].id = "cross_beam";
  collision_objects[6].header.frame_id = "world";

  // Define the primitive and its dimensions. 
  collision_objects[6].primitives.resize(1);
  collision_objects[6].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[6].primitives[0].dimensions.resize(3);
  collision_objects[6].primitives[0].dimensions[0] = 0.18;
  collision_objects[6].primitives[0].dimensions[1] = 2.3;
  collision_objects[6].primitives[0].dimensions[2] = 0.3;

  // Define the pose of the cross beam. 
  collision_objects[6].primitive_poses.resize(1);
  collision_objects[6].primitive_poses[0].position.x = 1.4;
  collision_objects[6].primitive_poses[0].position.y = -0.35;
  collision_objects[6].primitive_poses[0].position.z = 0.9;

  collision_objects[6].primitive_poses[0].orientation.w = 0.9914449;
  collision_objects[6].primitive_poses[0].orientation.x = -0.130526;
  collision_objects[6].primitive_poses[0].orientation.y = 0;
  collision_objects[6].primitive_poses[0].orientation.z = 0;
  // END_SUB_TUTORIAL

  collision_objects[6].operation = collision_objects[6].ADD;
*/

  ros::Duration(1.0).sleep();
  planning_scene_interface.applyCollisionObjects(collision_objects);
  sleep(4.0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv,"planning_scene_sawyer");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while(planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
  ros::WallDuration sleep_t(0.5);
  sleep_t.sleep();
  }
  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("right_arm");
  
  
  group.setPlanningTime(60.0);
  group.setPlannerId(group.getDefaultPlannerId(group.getName()));
  addCollisionObjects(planning_scene_interface);
  
  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();
  ROS_INFO("Now launch ROS Moveit to start planning");
  ros::waitForShutdown();
  return 0;
}
