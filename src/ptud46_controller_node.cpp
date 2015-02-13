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
#include "sensor_msgs/LaserScan.h"

#include "hokuyo_node/HokuyoConfig.h"

#include "hokuyo_node/hokuyo.h"
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
/////////////////////////////////////////////
int hokuyo::Laser::pollScan(hokuyo::LaserScan& scan, double min_ang, double max_ang, int cluster, int timeout)
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  int status;

  // Always clear ranges/intensities so we can return easily in case of erro
  scan.ranges.clear();
  scan.intensities.clear();

  // clustering of 0 and 1 are actually the same
  if (cluster == 0)
    cluster = 1;
  
  int min_i = (int)(afrt_ + min_ang*ares_/(2.0*M_PI));
  int max_i = (int)(afrt_ + max_ang*ares_/(2.0*M_PI));
  
  char cmdbuf[MAX_CMD_LEN];
  
  sprintf(cmdbuf,"GD%.4d%.4d%.2d", min_i, max_i, cluster);
  
  status = sendCmd(cmdbuf, timeout);
  
  scan.system_time_stamp = timeHelper() + offset_;
  
  if (status != 0)
    return status;
  
  // Populate configuration
  scan.config.min_angle  =  (min_i - afrt_) * (2.0*M_PI)/(ares_);
  scan.config.max_angle  =  (max_i - afrt_) * (2.0*M_PI)/(ares_);
  scan.config.ang_increment =  cluster*(2.0*M_PI)/(ares_);
  scan.config.time_increment = (60.0)/(double)(rate_ * ares_);
  scan.config.scan_time = 0.0;
  scan.config.min_range  =  dmin_ / 1000.0;
  scan.config.max_range  =  dmax_ / 1000.0;
  
  readData(scan, false, timeout);
  
  long long inc = (long long)(min_i * scan.config.time_increment * 1000000000);
  
  scan.system_time_stamp += inc;
  scan.self_time_stamp += inc;
  
  return 0;
}
///////////////////////////////////////////////////////////////////////////////
int hokuyo::Laser::sendCmd(const char* cmd, int timeout)
{
if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  char buf[100]; 

  laserWrite(cmd);
  laserWrite("\n");

  laserReadlineAfter(buf, 100, cmd, timeout);
  laserReadline(buf,100,timeout);

  if (!checkSum(buf,4))
    HOKUYO_EXCEPT(hokuyo::CorruptedDataException, "Checksum failed on status code.");

  buf[2] = 0;
  
  if (buf[0] - '0' >= 0 && buf[0] - '0' <= 9 && buf[1] - '0' >= 0 && buf[1] - '0' <= 9)
    return (buf[0] - '0')*10 + (buf[1] - '0');
  else
    HOKUYO_EXCEPT(hokuyo::Exception, "Hokuyo error code returned. Cmd: %s --  Error: %s", cmd, buf);
}


///////////////////////////////////////////////////////////////////////////////
void cmdpose_callback(const geometry_msgs::Vector3 cmdpose_msg){
	double old_pan_cmd = pan_cmd;
	double old_tilt_cmd = tilt_cmd;
	pan_cmd=cmdpose_msg.x;
	tilt_cmd=cmdpose_msg.y;
	//std::cout<<pan_ang<<std::endl;
	if(pan_cmd==old_pan_cmd && tilt_cmd==old_tilt_cmd){
		new_cmd=false;
	}
	else{
		new_cmd=true;
	}
}

///////////////////////////////////////////////////////
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
		   // Set up polling callback
    int hz;
    ros::param::param<int>("~hz", hz, PTU_DEFAULT_HZ);
    ros::Timer spin_timer = n.createTimer(ros::Duration(1 / hz),
        &hokoyu_driver::Node::spinCallback, &ptu_node);

    // Spin until there's a problem or we're in shutdown
    ros::spin();
    
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














