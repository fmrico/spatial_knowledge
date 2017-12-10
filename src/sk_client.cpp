#include <spatial_knowledge/sk_client.h>

SKClient::SKClient(const std::string& id, const std::string& frame, const double res)
: nh_("~"),
	id_(id),
	frame_(frame),
	changed_(true),
	res_(res),
	sk_map_(res_)
{
	sk_pub_ = nh_.advertise<octomap_msgs::Octomap>("sk_"+id, 10, true); //latched
}

void
SKClient::publish()
{
	if(!changed_) return;

	octomap_msgs::Octomap msg;
	octomap_msgs::binaryMapToMsg(sk_map_, msg);
	msg.header.frame_id = frame_;
	msg.header.stamp = ros::Time::now();
	sk_pub_.publish(msg);

	changed_ = false;
}

void
SKClient::updateSK(const octomap::point3d &min, const octomap::point3d &max, double value)
{

	for(double x=min.x(); x<max.x(); x+=res_/2.0)
		for(double y=min.y(); y<max.y(); y+=res_/2.0)
			for(double z=min.z(); z<max.z(); z+=res_/2.0)
			{
					octomap::OcTreeKey key;
					if (sk_map_.coordToKeyChecked(octomap::point3d(x,y,z), key))
      			sk_map_.updateNode(key, true, true)->setValue(1.0);
			}


	sk_map_.updateInnerOccupancy();
	sk_map_.prune();

	changed_ = true;
}

bool
SKClient::isOccupied(const octomap::point3d &coord)
{
	octomap::OcTreeNode *node =	sk_map_.search(coord);

	return (node!=NULL && node->getValue()>0.05);
}

void
SKClient::updateSK(const octomap::point3d &coord, double value)
{
	octomap::OcTreeKey key;
	if (sk_map_.coordToKeyChecked(coord, key))
      sk_map_.updateNode(key, true, false)->setValue(1.0);

	sk_map_.prune();

	changed_ = true;
}
