#include <ros/ros.h>

#include <spatial_knowledge/sk_client.h>
#include <erl_planning/KMS_Client.h>
#include "mongodb_store/message_store.h"


class SKMap: public KMS_Client
{
public:
	SKMap(): nh_("~"), message_store(nh_)
	{
		start_location();
		update_kms();
	}

	void update_kms()
	{
		std::map<std::string, std::shared_ptr<SKClient> >::iterator it;
		for(it=locations_.begin(); it!=locations_.end(); ++it)
		{
			std::string location_id = it->first;
			add_instance("location", location_id);

			Fact fact_location;
			fact_location.push_back(Fact_Values("l", location_id));
			add_fact(location_id+"_location", fact_location);
		}
	}

	void start_location()
	{
		if(nh_.hasParam("locations/location_ids"))
		{
			std::vector<std::string> locations;
			nh_.getParam("locations/location_ids", locations);

			for(std::vector<std::string>::iterator it=locations.begin(); it!=locations.end(); ++it)
			{
				if(nh_.hasParam("locations/"+*it))
				{
					std::map<std::string, std::string> info;
					nh_.getParam("locations/"+*it, info);

					locations_[*it] = std::shared_ptr<SKClient>(new SKClient("location_"+*it));

					double Mx, mx, My, my, Mz, mz;
					double cx, cy, cz;

					sscanf(info["range"].c_str(), "[%lf, %lf], [%lf, %lf], [%lf, %lf]", &mx, &Mx, &my, &My, &mz, &Mz);
					sscanf(info["position"].c_str(), "%lf, %lf, %lf", &cx, &cy, &cz);

					octomap::point3d start(mx, my, mz);
					octomap::point3d end(Mx, My, Mz);

					locations_[*it]->updateSK(start, end);

					geometry_msgs::PoseStamped pose;
					pose.pose.position.x = cx;
					pose.pose.position.y = cy;
					pose.pose.position.z = 0;

					tf::Quaternion q;
					q.setEuler(cz, 0 ,0);

					pose.pose.orientation.x = q.x();
					pose.pose.orientation.y = q.z();
					pose.pose.orientation.w = q.w();
					pose.pose.orientation.z = q.y();

					locations_id_[*it] = message_store.insertNamed(*it, pose);
				}
			}
		}
	}

	void update()
	{
			std::map<std::string, std::shared_ptr<SKClient> >::iterator it;
			for(it=locations_.begin(); it!=locations_.end(); ++it)
				it->second->publish();
	}

private:
	ros::NodeHandle nh_;

	std::map<std::string, std::shared_ptr<SKClient> > locations_;
	std::map<std::string, std::string > locations_id_;
	mongodb_store::MessageStoreProxy message_store;

};


int main(int argc, char **argv)
{

   ros::init(argc, argv, "sk_map");
   ros::NodeHandle n;

	 SKMap skmap;

   ros::Rate loop_rate(1);

   int count=0;

   while (ros::ok())
   {

		 skmap.update();

     ros::spinOnce();
     loop_rate.sleep();
   }

   return 0;

 }
