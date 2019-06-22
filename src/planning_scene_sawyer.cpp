#include <ros/ros.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include "geometric_shapes/shape_operations.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 2 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.6;
  collision_objects[0].primitives[0].dimensions[1] = 1.5;
  collision_objects[0].primitives[0].dimensions[2] = 0.75;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 1.0;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = -0.45744;

  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  collision_objects[0].primitive_poses[0].orientation.x = 0;
  collision_objects[0].primitive_poses[0].orientation.y = 0;
  collision_objects[0].primitive_poses[0].orientation.z = 0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;
  std::vector<moveit_msgs::CollisionObject> collision_vector;
  collision_objects.push_back(collision_objects[0]);

  // Add the rear table behind Sawyer.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "world";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.66;
  collision_objects[1].primitives[0].dimensions[1] = 3.0;
  collision_objects[1].primitives[0].dimensions[2] = 0.05;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = -0.15;
  collision_objects[1].primitive_poses[0].position.y = 0;
  collision_objects[1].primitive_poses[0].position.z = 0;

  collision_objects[1].primitive_poses[0].orientation.w = 1.0;
  collision_objects[1].primitive_poses[0].orientation.x = 0;
  collision_objects[1].primitive_poses[0].orientation.y = 0;
  collision_objects[1].primitive_poses[0].orientation.z = 0;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;
  std::vector<moveit_msgs::CollisionObject> collision_vector1;
  collision_objects.push_back(collision_objects[1]);

  planning_scene_interface.applyCollisionObjects(collision_vector);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_sawyer");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("right_arm");
  
  group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();
  ros::waitForShutdown();
  return 0;
}
