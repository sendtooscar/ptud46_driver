/* ptud32 controller node
1) sync with ptu - done using the ptu_ready topic
TODO 2) implement a tilting scannar function. default params -30 deg to +30 deg with 0.3 deg resolution.
TODO 3) make parameters changeble by a ros parameters using a launch file
TODO 4) generate point cloud from the laser scans and publsh
*/


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>

#include <stdio.h>

bool ptuReady;

void ptureadyCallback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data=="*"){
  	ptuReady=true;
  }else{
  	ptuReady=false;
  }
}


int main(int argc, char **argv){

	// Initialize ROS
	ros::init(argc, argv, "ptud46_controller_node");
	ros::NodeHandle n;
	tf::Transform transform;
  	tf::Quaternion q;
	ros::Rate loop_rate(40);
	static tf::TransformBroadcaster br;
	ros::Publisher cmdpose_pub = n.advertise<geometry_msgs::Vector3>("cmd_pose", 100);
	ros::Subscriber sub = n.subscribe("ptu_ready", 1000, ptureadyCallback);
	
	// Loop	
	int pan =0;
	int tilt_cmd =0;
	bool fwdt=true;
	while (ros::ok())
	{   
	  if(ptuReady){
	    geometry_msgs::Vector3 msg;
	    msg.x = 1000;
	    msg.y = tilt_cmd;
	    cmdpose_pub.publish(msg);
	    ptuReady=false;
	     
	    
	    if (tilt_cmd<585 && fwdt){
	    	tilt_cmd=tilt_cmd+5; // 1step=0.0514285°
	    			     // 5step=0.257
	    }
	    else{

	        fwdt=false;
	    }
	    
	    if (tilt_cmd>-585 && !fwdt){
	    	tilt_cmd=tilt_cmd-5;
	    }
	    else{
	        fwdt=true;
	    }
	    
	  }  
	    ros::spinOnce();
	    loop_rate.sleep();
	}
	return 0;
}














