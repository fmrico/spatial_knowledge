#ifndef SKCLIENT_H
#define SKCLIENT_H

#include <ros/ros.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap/OcTreeKey.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTreeIterator.hxx>

#include <string>
#include <iostream>

class SKClient
{
public:
	SKClient(const std::string& id, const std::string& frame = "/map", const double res=0.2);

	void publish();

	void updateSK(const octomap::point3d &min, const octomap::point3d &max, double value=1.0);
	void updateSK(const octomap::point3d &coord, double value=1.0);
	//void updateSK(double value);
	bool isOccupied(const octomap::point3d &coord);

protected:
	ros::NodeHandle nh_;

private:
	ros::Publisher sk_pub_;

	std::string id_;
	std::string frame_;
	double res_;

	bool changed_;

	octomap::OcTree sk_map_;
};

#endif
