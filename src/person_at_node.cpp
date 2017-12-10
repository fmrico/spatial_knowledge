
#include <ros/ros.h>
#include <ros/master.h>

#include <erl_planning/KMS_Client.h>

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

class PersonAtProcessor:public KMS_Client
{
public:
	PersonAtProcessor(): nh_()
	{

	}

	bool isPeopleInLocation(const std::shared_ptr<octomap::OcTree>& people, const std::shared_ptr<octomap::OcTree>& location)
	{
		octomap::OcTree::leaf_iterator it;
		for(it=people->begin_leafs(); it!=people->end_leafs(); ++it)
		{
			if(it->getValue() >= 0.05)
			{
					octomap::point3d p = it.getCoordinate();
					octomap::OcTreeNode *node =	location->search(p);

					if(node!=NULL && node->getValue()>0.05) return true;
			}
		}

		return false;
	}

	void update()
	{
		subscribePeople();
		subscribeLocations();

		std::map<std::string, std::shared_ptr<octomap::OcTree> >::iterator it_people;
		std::map<std::string, std::shared_ptr<octomap::OcTree> >::iterator it_location;

		std::vector<std::string> ops = {"p", "l"};
		std::list<std::string> facts = getFactsWithPredicate("person_at", ops);

		std::set<std::string> kms_current;
		for(std::list<std::string>::iterator it=facts.begin(); it!=facts.end(); ++it)
			kms_current.insert(*it);

		std::set<std::string> perceived_current;
		for(it_people = people.begin(); it_people!=people.end(); ++it_people)
			for(it_location = location.begin(); it_location!=location.end(); ++it_location)
				if(isPeopleInLocation(it_people->second, it_location->second))
				{
					std::string new_fact("person_at "+it_people->first+" "+it_location->first);
					perceived_current.insert(new_fact);
				}

		std::set<std::string> intersection;
		std::set<std::string> only_in_kms;
		std::set<std::string> only_in_perceived;

		std::set_intersection(kms_current.begin(), kms_current.end(),
                          perceived_current.begin(), perceived_current.end(),
                          std::inserter(intersection, intersection.begin()));

		std::set_difference(kms_current.begin(), kms_current.end(),
                          intersection.begin(), intersection.end(),
                          std::inserter(only_in_kms, only_in_kms.begin()));

		std::set_difference(perceived_current.begin(), perceived_current.end(),
                          intersection.begin(), intersection.end(),
                          std::inserter(only_in_perceived, only_in_perceived.begin()));

		std::set<std::string>::iterator it;
		for(it = only_in_kms.begin(); it!=only_in_kms.end(); it++)
			remove_fact("person_at", getFactFromString(*it, ops));

		for(it = only_in_perceived.begin(); it!=only_in_perceived.end(); it++)
			add_fact("person_at", getFactFromString(*it, ops));
	}

private:

	void peopleCB(const ros::MessageEvent<octomap_msgs::Octomap const>& event)
	{
		std::string topic_name = event.getConnectionHeader().at("topic");

		size_t pos = topic_name.rfind("/sk_people_node/sk_people_");
		std::string name = topic_name. substr(pos+std::string("/sk_people_node/sk_people_").length());

  	octomap::AbstractOcTree* tect = octomap_msgs::binaryMsgToMap(*event.getMessage());
    octomap::OcTree* te = dynamic_cast<octomap::OcTree*>(tect);

		ROS_INFO("Adding [%s] as people", name.c_str());
		people[name] = std::shared_ptr<octomap::OcTree>(new octomap::OcTree(*te));
	}

	void locationCB(const ros::MessageEvent<octomap_msgs::Octomap const>& event)
	{
		std::string topic_name = event.getConnectionHeader().at("topic");

		size_t pos = topic_name.rfind("/sk_map_node/sk_location_");
		std::string name = topic_name. substr(pos+std::string("/sk_map_node/sk_location_").length());

  	octomap::AbstractOcTree* tect = octomap_msgs::binaryMsgToMap(*event.getMessage());
    octomap::OcTree* te = dynamic_cast<octomap::OcTree*>(tect);

		ROS_INFO("Adding [%s] as location", name.c_str());
		location[name] = std::shared_ptr<octomap::OcTree>(new octomap::OcTree(*te));
	}


	void subscribePeople()
	{
		ros::master::V_TopicInfo topic_infos;
  	ros::master::getTopics(topic_infos);

		for(ros::master::V_TopicInfo::iterator it=topic_infos.begin(); it!=topic_infos.end(); ++it)
		{
			size_t pos;
			if(pos = it->name.rfind("/sk_people_node/sk_people_") != std::string::npos)
			{
				std::string name = it->name.substr(pos+std::string("/sk_people_node/sk_people_").length()-1);

				if(people.find(name) == people.end())
				{
					people_sub.push_back( nh_.subscribe(it->name, 1, &PersonAtProcessor::peopleCB, this));
					people[name] = std::shared_ptr<octomap::OcTree>(new octomap::OcTree(0.2));
				}
			}
		}
	}

	void subscribeLocations()
	{
		ros::master::V_TopicInfo topic_infos;
  	ros::master::getTopics(topic_infos);

		for(ros::master::V_TopicInfo::iterator it=topic_infos.begin(); it!=topic_infos.end(); ++it)
		{
			size_t pos;
			if(pos = it->name.rfind("/sk_map_node/sk_location_") != std::string::npos)
			{
				std::string name = it->name.substr(pos+std::string("/sk_map_node/sk_location_").length()-1);

				if(location.find(name) == location.end())
				{
					ROS_INFO("subscribing to [%s]", it->name.c_str());
					location_sub.push_back( nh_.subscribe(it->name, 1, &PersonAtProcessor::locationCB, this));
					location[name] = std::shared_ptr<octomap::OcTree>(new octomap::OcTree(0.2));
				}
			}
		}

	}

	ros::NodeHandle nh_;

	std::list<ros::Subscriber> people_sub;
	std::list<ros::Subscriber> location_sub;

	std::map<std::string, std::shared_ptr<octomap::OcTree> > people;
	std::map<std::string, std::shared_ptr<octomap::OcTree> > location;
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "person_at");
	ros::NodeHandle n;

	PersonAtProcessor person_at;

	ros::Rate loop_rate(1);

	int count=0;

	while (ros::ok())
	{

		person_at.update();

		ros::spinOnce();
		loop_rate.sleep();
	}

}
