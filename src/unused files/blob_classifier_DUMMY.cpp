#include <ros/ros.h>
#include "cmvision/Blobs.h" 
#include <string>

using namespace std; 

ros::Publisher pub_msg; 

cmvision::Blobs currentBlobs; 

void blobCallback(cmvision::Blobs blobsData)
{
    currentBlobs = blobsData; 
} 
    
int main(int argc, char **argv) { 
    //ROS Initialization     
    ros::init(argc, argv, "blob_classifier");
    ros::NodeHandle nh;   
    ROS_INFO("Node blob_classifier Connected to roscore");    
    ros::Subscriber blobsubs = nh.subscribe<cmvision::Blobs>("/blobs",1,blobCallback); 
    ros::Rate rate(10);     
    ROS_INFO("SPINNING @ 10Hz");  
    int arrX[100], arrY[100];
    while (ros::ok()){         
        ros::spinOnce();         
        double d = 0;     
        for(int i = 0; i < currentBlobs.blob_count; i++){
            // string blobName = (string)currentBlobs.blobs[i].name;
            // string targetName = "onion";
            if (currentBlobs.blobs[i].name == "Onion"){
                cout<<"Onion";
                // arrX[i] = currentBlobs.blobs[i].x;
                // arrY[i] = currentBlobs.blobs[i].y;
            }
        }
    }   
    rate.sleep();           
    ROS_INFO("ROS-Node Terminated\n"); 
}