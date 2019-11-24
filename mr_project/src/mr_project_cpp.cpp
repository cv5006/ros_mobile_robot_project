#include "PCOp.hpp"
#include "ros/ros.h"
#include "ros/package.h"
#include "math.h"
#include "std_msgs/Float64.h"
#include "vector"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include <nav_msgs/Odometry.h>


// 2d pose
#include <geometry_msgs/Pose2D.h>
// ground truth pose message
#include <nav_msgs/Odometry.h>

// tf2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
// tf2 matrix
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>







#define pi  3.14159265358979323846

enum{ Out_of_Row, Row_Start, On_Row, Row_End };


class MRP
{
public:
	void init();
	// driver
	void ackermann_driver( float cmd_vel, float cmd_deg );
	void translational_driver( float cmd_vel, float cmd_deg );
	void rotational_driver( float vel );
	void vel_cmd_CB( const std_msgs::Float64& d);
	void deg_cmd_CB( const std_msgs::Float64& d);
	void row_Tracking( const Eigen::MatrixXf& res );
	void rviz_line( const Eigen::MatrixXf& res, const sensor_msgs::PointCloud& pms );
	void row_state( const sensor_msgs::PointCloud& pcmeans, const std::vector<float>& stats );
	void groundTruthCB( const nav_msgs::Odometry::ConstPtr& _odom_msg );
	void field_map_PC( const sensor_msgs::PointCloud& pcmeans);
	void scan_CB( const sensor_msgs::PointCloud& pc);
	void run();
private:

	// ros noe handle
	ros::NodeHandle nh;
     // subscriber
     ros::Subscriber vel_sub;
     ros::Subscriber deg_sub;
     ros::Subscriber scan_sub;
     ros::Subscriber ground_pose_sub;

     // publisher
     ros::Publisher v_FR;
     ros::Publisher v_FL;
     ros::Publisher v_BR;
     ros::Publisher v_BL;
     ros::Publisher th_FR;
     ros::Publisher th_FL;
     ros::Publisher th_BR;
     ros::Publisher th_BL;

     ros::Publisher PCOp_pub;
     ros::Publisher row_line;
     ros::Publisher field_map_pub;
     // vel and deg
     float vel, deg;

     // row state
     int RS = 0, RS_prev;
     const double init_x = -0.0;
	// robot geometry
	const float wheel_r = 0.25;
	const float lidar_p = 0.7854, lidar_x = 1.0, lidar_h = 1.5;
     const float X = 2.4/2, Y = 2.0/2;

     const int queue_size = 50;

     geometry_msgs::Pose2D gt_pose_;
     sensor_msgs::PointCloud field_map;
     sensor_msgs::PointCloud field_map_forpub;


	float travel, head;
	bool turn_seq=0;

	int rows = 0, seq = 0;
	geometry_msgs::Pose2D pose_stamp;
};


int main( int argc, char **argv )
{
	ros::init(argc, argv, "mrp_node");

	MRP mrp;
	mrp.init();
		
	mrp.run();
	ros::spin();
	
  return 0;
}

void MRP::run()
{
	ROS_WARN("Gonna Fuckin Run it");
	ground_pose_sub = nh.subscribe("/ground_truth_pose", queue_size, &MRP::groundTruthCB, this);
	scan_sub = nh.subscribe("/block_laser", queue_size, &MRP::scan_CB, this); 
	vel_sub = nh.subscribe("vel_cmd", queue_size, &MRP::vel_cmd_CB, this);
	deg_sub = nh.subscribe("deg_cmd", queue_size, &MRP::deg_cmd_CB, this);

}

void MRP::scan_CB( const sensor_msgs::PointCloud& pc)
{	
	//// point cloud process
	// transform
	sensor_msgs::PointCloud pctf = PC2BFframe( lidar_p, lidar_x, lidar_h, pc);
	// plant extraction
	auto pcplants = extractPlants(pctf);
	// clustering
	sensor_msgs::PointCloud pcmeans = clustering(std::get<0>(pcplants),
											    std::get<1>(pcplants).at(1));
	//// row state
	row_state(pcmeans, std::get<1>(pcplants));
	//// drive

	
	if ( RS == On_Row && turn_seq == 0){
		field_map_PC(PC2Mapframe(gt_pose_.x,gt_pose_.y,gt_pose_.theta,pcmeans));
		// line_detection
		Eigen::MatrixXf res = line_detection(pcmeans);
		// row tracking
		row_Tracking(res);
		rviz_line(res, pcmeans);

		PCOp_pub.publish(pcmeans);
	}else{

		if (RS== Out_of_Row && RS_prev == Row_End)
		{turn_seq = 1; pose_stamp = gt_pose_; seq = 0;}

		travel = pow(pose_stamp.x - gt_pose_.x,2.0)+pow(pose_stamp.y - gt_pose_.y,2.0);
		head = fabsf(pose_stamp.theta - gt_pose_.theta);
		if (turn_seq){
			switch (seq){
				case 0:
					deg = 0.0; ackermann_driver(vel, deg);
					switch (rows){
						case 0: if (travel > 20.0) { seq = 1;} break;
						case 1: if (travel > 16.0) { seq = 1;} break;}
					break;
					
					break;
				case 1:
					rotational_driver(2*vel);
					switch (rows){
						case 0: if (head > pi) {seq=2; pose_stamp = gt_pose_;} break;
						case 1: if (head > pi - pi/15) {seq=2; pose_stamp = gt_pose_;} break;}
					break;
				case 2:
					if (rows == 0){deg = -90.0;}else{deg = +90.0;}
					translational_driver(4*vel, deg);
					if (travel > 4.5){seq = 0; turn_seq =0; rows=1;}
					break;
			}
		}

		
	}
	field_map_pub.publish(field_map_forpub);
}

void MRP::field_map_PC( const sensor_msgs::PointCloud& pcmeans)
{
	float size = field_map.points.size();
	bool E;
	if (size < 3){
		for (int i=0; i<3; i++){
		field_map.points.push_back(pcmeans.points.at(i));
		}
	}else{
	for (int i=size-1; i>size-3; i--){
			E =(pow(pcmeans.points.at(0).x - field_map.points.at(i).x, 2.0) +
			    pow(pcmeans.points.at(0).y - field_map.points.at(i).y, 2.0))   > 0.35*0.35;
			if (E){
				field_map.points.push_back(pcmeans.points.at(0));
			}
	}	
	field_map_forpub = Map2PCframe(gt_pose_.x, gt_pose_.y, gt_pose_.theta, field_map);
	}
}




void MRP::row_state( const sensor_msgs::PointCloud& pcmeans, const std::vector<float>& stats )
{
	using namespace std;
	bool Plants_on_Range = fabsf(stats.at(0)-stats.at(1)) > 0.05;
	bool near_a, near_b, near_c;

	float near_threshold = pow(0.3, 2.0);
	near_a = (pow(pcmeans.points.at(0).x - pcmeans.points.at(1).x,2.0) +
			 pow(pcmeans.points.at(0).y - pcmeans.points.at(1).y,2.0)) > near_threshold;
	near_b = (pow(pcmeans.points.at(1).x - pcmeans.points.at(2).x,2.0) +
			 pow(pcmeans.points.at(1).y - pcmeans.points.at(2).y,2.0)) > near_threshold;
	near_c = (pow(pcmeans.points.at(2).x - pcmeans.points.at(0).x,2.0) +
			 pow(pcmeans.points.at(2).y - pcmeans.points.at(0).y,2.0)) > near_threshold;
	int near_sum = near_a + near_b + near_c;

	float middle_x = lidar_x + lidar_h*cos(lidar_p);
	float x_m = (pcmeans.points.at(0).x + pcmeans.points.at(1).x + pcmeans.points.at(2).x)/3.0;
	
	RS_prev = RS;
	if ( Plants_on_Range == 0){
		RS = Out_of_Row;
	}else{
		if ( near_sum > 1){
			RS = On_Row;
		}else{
			if( x_m > middle_x){
				RS = Row_Start;
			}else{
				RS = Row_End;
			}
		}
	}
	if (RS != RS_prev){
		switch (RS){
			case Out_of_Row:		cout << "Out_of_Row" << endl;	break;
			case On_Row: 		cout << "On_Row" << endl;		break;
			case Row_Start:		cout << "Row_Start" << endl;	break;
			case Row_End: 		cout << "Row_End" << endl;		break;
		}
	}
}




void MRP::row_Tracking( const Eigen::MatrixXf& res )
{	
	float t = res(0,0), b = res(1,0), deg_track;

	bool escape = 0;

	if ( fabsf(t) < 0.17 || escape ){
		if ( fabsf(b) > 0.2 || escape ){
			escape = 1;
			int pp = (b > 0.0) - (b < 0.0);
			deg = 30.0*pp;
			translational_driver(vel, deg);
			if (fabsf(b) < 0.05){
				escape = 0;
			}
		}else{
			deg = 0.0;
			ackermann_driver(vel, deg);
		}
	}else{
		float t2 = (sqrt(1+t*t)-1)/t;
		if ( t > 0){
			deg_track = atan((X*t*t2/(X*t-b)));	
		}else{
			deg_track = atan((X*t*t2/(X*t-b)));	
		}
		deg = deg_track/pi*180;
		ackermann_driver(vel, deg);
	}
}



void MRP::ackermann_driver( float cmd_vel, float cmd_deg )
{
	float cmd_w = cmd_vel/wheel_r;
	float S;
	float th, frs, fls, brs, bls, vs;
	if (fabsf(cmd_deg) > 0.01){
		th = cmd_deg/180.0*pi;
	}else{
		th = 0;
	}
	std_msgs::Float64 thfr, thfl, thbr, thbl, vfr, vfl, vbr, vbl;
	if (th > 0.0){
		S = X / tan(th);
		thfr.data = atan2(2.0*X,S+Y);		thfl.data = atan2(2.0*X,S-Y);
		frs = sqrt((S+Y)*(S+Y)+4.0*X*X);	fls = sqrt((S-Y)*(S-Y)+4.0*X*X);
		brs = S+Y;						bls = S-Y;
		vs = sqrt(X*X + S*S);			
	}else if (th < 0.0){
		S = -X / tan(th);
		thfr.data = -atan2(2.0*X,S-Y);		thfl.data = -atan2(2.0*X,S+Y);
		frs = sqrt((S-Y)*(S-Y)+4.0*X*X);	fls = sqrt((S+Y)*(S+Y)+4.0*X*X);		
		brs = S-Y;						bls = S+Y;
		vs = sqrt(X*X + S*S);
	}else{
		S = 0.0;	
		thfr.data = 0.0;		thfl.data = 0.0;		
		frs = 1.0; 	fls = 1.0;				
		brs = 1.0;	bls = 1.0;		
		vs = 1.0;
	}

	vfr.data = cmd_w*frs/vs;	vfl.data = cmd_w*fls/vs;
	vbr.data = cmd_w*brs/vs; 	vbl.data = cmd_w*bls/vs;
	thbr.data = 0.0; thbl.data = 0.0;
	v_FR.publish(vfr); 	v_FL.publish(vfl); 	v_BR.publish(vbr);	v_BL.publish(vbl);
	th_FR.publish(thfr); th_FL.publish(thfl);	th_BR.publish(thbr); 	th_BL.publish(thbl);
}


void MRP::translational_driver( float cmd_vel, float cmd_deg )
{
	std_msgs::Float64 thfr, thfl, thbr, thbl, vfr, vfl, vbr, vbl;
	cmd_deg = cmd_deg*pi/180;
	vfr.data = cmd_vel;	vfl.data = cmd_vel;
	vbr.data = cmd_vel; 	vbl.data = cmd_vel;
	thfr.data = cmd_deg; 	thfl.data = cmd_deg;
	thbr.data = cmd_deg; 	thbl.data = cmd_deg;

	v_FR.publish(vfr); 	v_FL.publish(vfl); 	v_BR.publish(vbr);	v_BL.publish(vbl);
	th_FR.publish(thfr); th_FL.publish(thfl);	th_BR.publish(thbr); 	th_BL.publish(thbl);
}

void MRP::rotational_driver( float vel )
{
	std_msgs::Float64 thfr, thfl, thbr, thbl, vfr, vfl, vbr, vbl;
	
	float th = atan2(X,Y);
	float cmd_vel = vel*pi;

	vfr.data = +cmd_vel;	vfl.data = -cmd_vel;
	vbr.data = +cmd_vel; 	vbl.data = -cmd_vel;
	thfr.data = +th; 	thfl.data = -th;
	thbr.data = -th; 	thbl.data = +th;

	v_FR.publish(vfr); 	v_FL.publish(vfl); 	v_BR.publish(vbr);	v_BL.publish(vbl);
	th_FR.publish(thfr); th_FL.publish(thfl);	th_BR.publish(thbr); 	th_BL.publish(thbl);
}



void MRP::vel_cmd_CB( const std_msgs::Float64& v){	vel = v.data;}
void MRP::deg_cmd_CB( const std_msgs::Float64& d){	deg = d.data;}



void MRP::rviz_line( const Eigen::MatrixXf& res, const sensor_msgs::PointCloud& pms )
{
	ros::Duration d(0.1);
	float pz = (pms.points.at(0).z + pms.points.at(1).z + pms.points.at(2).z)/3.0;
	geometry_msgs::Point p[2];
	p[0].x = 0.8 + lidar_x;	p[0].y = p[0].x*res(0,0) + res(1,0); 		p[0].z = pz;
	p[1].x = 2.5 + lidar_x;	p[1].y = p[1].x*res(0,0) + res(1,0);		p[1].z = pz;
	visualization_msgs::Marker rl;
	rl.header.frame_id = "base_footprint";
	rl.header.stamp = ros::Time();
	rl.ns = "row_line";
	rl.id = 0;
	rl.type = visualization_msgs::Marker::LINE_STRIP;
	rl.action = visualization_msgs::Marker::ADD;
	rl.pose.orientation.w = 0.0;
	rl.scale.x = 0.1;
	rl.color.a = 1.0; // Don't forget to set the alpha!
	rl.color.r = 0.2;
	rl.color.g = 1.0;
	rl.color.b = 0.2;
	rl.lifetime = d;
	rl.points.push_back(p[0]);
	rl.points.push_back(p[1]);
	row_line.publish( rl );
}



void MRP::groundTruthCB( const nav_msgs::Odometry::ConstPtr& _odom_msg )
{
  //// compute the frame transform. [G] tFo [R0]. [G] is ground truth frame. [R0] is the initial robot frame.
  // The transform is R0_T_G.
  static double cos_theta= cos(0.0);
  static double sin_theta= sin(0.0);
  // need to transform the initial translation
  static double init_x= cos_theta*(init_x) - sin_theta*(0.0);
  static double init_y= sin_theta*(0.0) + cos_theta*(0.0);
  // transform the ground truth to the R
  double x= cos_theta*_odom_msg->pose.pose.position.x - sin_theta*_odom_msg->pose.pose.position.y;
  double y= sin_theta*_odom_msg->pose.pose.position.x + cos_theta*_odom_msg->pose.pose.position.y;
  gt_pose_.x= x + init_x;
  gt_pose_.y= y + init_y;

  // build tf quaternion
  tf2::Quaternion q( _odom_msg->pose.pose.orientation.x, _odom_msg->pose.pose.orientation.y, _odom_msg->pose.pose.orientation.z, _odom_msg->pose.pose.orientation.w );
  // build tf matrix from the quaternion
  tf2::Matrix3x3 m(q);
  // get angles
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  // ground truth of yaw
  gt_pose_.theta= yaw;


}






void MRP::init()
{
	//ackermann_driver(vel, deg);
	vel = 1.5; deg = 0.0;
	field_map.header.frame_id = "base_footprint";
	// drivers
	v_FR = nh.advertise<std_msgs::Float64>("/bonirob_simple/wheel_FR/command", queue_size);
	v_FL = nh.advertise<std_msgs::Float64>("/bonirob_simple/wheel_FL/command", queue_size);
	v_BR = nh.advertise<std_msgs::Float64>("/bonirob_simple/wheel_BR/command", queue_size);
	v_BL = nh.advertise<std_msgs::Float64>("/bonirob_simple/wheel_BL/command", queue_size);
	th_FR = nh.advertise<std_msgs::Float64>("/bonirob_simple/steering_FR/command", queue_size);
	th_FL = nh.advertise<std_msgs::Float64>("/bonirob_simple/steering_FL/command", queue_size);
	th_BR = nh.advertise<std_msgs::Float64>("/bonirob_simple/steering_BR/command", queue_size);
	th_BL = nh.advertise<std_msgs::Float64>("/bonirob_simple/steering_BL/command", queue_size);
	// datas
	PCOp_pub = nh.advertise<sensor_msgs::PointCloud>("/block_laser_op", queue_size);
	row_line = nh.advertise<visualization_msgs::Marker>("visualization_marker", queue_size);
	field_map_pub = nh.advertise<sensor_msgs::PointCloud>("field_map", queue_size);

}