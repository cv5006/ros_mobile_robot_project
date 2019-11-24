#include "ros/ros.h"
#include "math.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Char.h"

class MRP
{
public:
	// driver
	void ackermann_driver( float vel, float deg );
	void rotational_driver( float w );

	void test_publisher( char a);
	void test_subscribe_CB( const std_msgs::Char& b);
	void run();
private:

	// ros noe handle
	ros::NodeHandle nh;
     // subscriber
     ros::Subscriber test_sub;
     // publisher
     ros::Publisher test_pub;

};


int main( int argc, char **argv )
{
	ros::init(argc, argv, "mrp_node");
	
	MRP mrp;
	
		
	char a=1;
	mrp.run();
	ros::spin();
	
  return 0;
}


void MRP::test_publisher( char a )
{
	test_pub = nh.advertise<std_msgs::Char>("test_output",50);
	std_msgs::Char val;
	val.data = a;
	test_pub.publish(val);
}

void MRP::test_subscribe_CB( const std_msgs::Char& b)
{	
	char bd = b.data;
	ROS_INFO("%d",bd);
	MRP::test_publisher(bd);
}

void MRP::run()
{
	test_sub = nh.subscribe("test_input", 50, &MRP::test_subscribe_CB, this);
}