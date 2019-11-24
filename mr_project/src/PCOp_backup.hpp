#include "ros/ros.h"
#include <math.h>
#include <vector>
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "std_msgs/Header.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <array>
#include "include/dkm.hpp"

#define pi  3.14159265358979323846

Eigen::MatrixXf pinv( Eigen::MatrixXf x);


sensor_msgs::PointCloud PC2BFframe( 
	const float lidar_p, const float lidar_x, const float lidar_h,
	const sensor_msgs::PointCloud& pc){

	// copy original msgs
	sensor_msgs::PointCloud pctf = pc;
	
	// transforming points
	float c = cos(lidar_p), s = sin(lidar_p);
	for (int i=0; i < pc.points.size(); i++){
		pctf.points.at(i).x = c*pc.points.at(i).x + s*pc.points.at(i).z + lidar_x;
		pctf.points.at(i).y =   pc.points.at(i).y;
		pctf.points.at(i).z = c*pc.points.at(i).z - s*pc.points.at(i).x + lidar_h;
	};

	// modify frame
	pctf.header.frame_id = "base_footprint";
	
	return pctf;
}


std::tuple<sensor_msgs::PointCloud, float> extractPlants( const sensor_msgs::PointCloud& pc )
{
	sensor_msgs::PointCloud pcex;

	pcex.header = pc.header;

	// confience interval
	using namespace std;
	float xbar = 0, xbar2 = 0, s, CI;
	for (int i = 0; i < pc.points.size(); i++)
	{
		xbar += pc.points.at(i).z;
		xbar2 += pc.points.at(i).z * pc.points.at(i).z;
	}
	xbar /= pc.points.size(); xbar2 /= pc.points.size();
	s = sqrt(xbar2 - xbar*xbar);
	CI = xbar + 1.96*s;

	// extract points
	for (int i = 0; i < pc.points.size(); i++)
	{
		if ( pc.points.at(i).z > CI )
		{
			pcex.points.push_back(pc.points.at(i));
		}
	}

	return std::make_tuple(pcex, CI);
}

sensor_msgs::PointCloud clustering( const sensor_msgs::PointCloud& pc, const float plant_z )
{
	std::vector<std::array<float,3>> v;
	v.assign(pc.points.size(), {0.0, 0.0});
	for (int i = 0; i < pc.points.size(); i++)
	{
		v.at(i) = {pc.points.at(i).x, pc.points.at(i).y, pc.points.at(i).z};
	}
	auto cluster_data = dkm::kmeans_lloyd(v, 3);
	
	auto mean = std::get<0>(cluster_data);

	sensor_msgs::PointCloud center;
	center.header = pc.header;
	geometry_msgs::Point32 mp;
	for (int i = 0; i < 3; i++)
	{
		mp.x = mean.at(i)[0];
		mp.y = mean.at(i)[1];
		mp.z = mean.at(i)[2];
		center.points.push_back(mp);
	}
	return center;
}

float line_detection( const sensor_msgs::PointCloud& center ){
	Eigen::MatrixXf X(3,2), Y(3,1), res;
	for (int i = 0; i < 3; i++)
	{
		X(i,0) = center.points.at(i).x;
		Y(i,0) = center.points.at(i).y;
		X(i,1) = 1.0;
	}
	res = pinv(X)*Y;
	res(0,0) = atan(res(0,0));
	std::cout << res(0,0)*180/pi << std::endl;

	return res(0,0)*180/pi;
}


Eigen::MatrixXf pinv( Eigen::MatrixXf x)
{
	using namespace Eigen;

	JacobiSVD<MatrixXf> svd(x, ComputeThinU | ComputeThinV);
	MatrixXf sv = svd.singularValues();
	
	MatrixXf isv = sv;
	for (int i=0; i<sv.size(); i++)
	{
		if (sv(i,0) > 1.e-6)
			isv(i,0) = 1.0/sv(i,0);
		else
			isv(i,0) = 0;
	}

	MatrixXf pinvm = svd.matrixV()*isv.asDiagonal()*svd.matrixU().transpose();
	return pinvm;
}