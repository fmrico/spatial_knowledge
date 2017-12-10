#include <ros/ros.h>

#include <spatial_knowledge/sk_client.h>
#include <erl_planning/KMS_Client.h>


class People: public SKClient, public KMS_Client
{
public:
	People(std::string name, std::string file, std::string cat, std::string ins)
	: SKClient("people_"+name),
		name_(name),
		file_(file),
		categoty_(cat),
		instance_(ins)
	{
		add_instance(instance_, name_);
		Fact fact_cat;
		fact_cat.push_back(Fact_Values("p", name_));
		add_fact(categoty_+"_person", fact_cat);
	}

private:

	std::string name_;
	std::string file_;
	std::string categoty_;
	std::string instance_;
};


class SKPeople
{
public:
	SKPeople(): nh_("~")
	{
		start_people();
	}

	void start_people()
	{
		if(nh_.hasParam("people/people_ids"))
		{
			std::vector<std::string> people;
			nh_.getParam("people/people_ids", people);

			for(std::vector<std::string>::iterator it=people.begin(); it!=people.end(); ++it)
			{
				printf("Creating [%s]", it->c_str());

				if(nh_.hasParam("people/"+*it))
				{
					std::map<std::string, std::string> info;
					nh_.getParam("people/"+*it, info);

					people_[*it] = std::shared_ptr<People>(new People(*it, info["file"], info["category"], info["instance"]));

					double cx, cy, cz;
					double dx, dy, dz;

					sscanf(info["position"].c_str(), "%lf, %lf, %lf", &cx, &cy, &cz);
					sscanf(info["dimensions"].c_str(), "%lf, %lf, %lf", &dx, &dy, &dz);

					octomap::point3d start(cx-dx/2, cy-dy/2, cz);
					octomap::point3d end(cx+dx/2, cy+dy/2, dz);

					people_[*it]->updateSK(start, end);
				}
			}
		}
	}

	void update()
	{
			std::map<std::string, std::shared_ptr<People> >::iterator it;
			for(it=people_.begin(); it!=people_.end(); ++it)
				it->second->publish();
	}

private:
	ros::NodeHandle nh_;

	std::map<std::string, std::shared_ptr<People> > people_;

};


int main(int argc, char **argv)
{

   ros::init(argc, argv, "sk_people");
   ros::NodeHandle n;

	 SKPeople skpeople;

   ros::Rate loop_rate(1);

   int count=0;

   while (ros::ok())
   {

		 skpeople.update();

     ros::spinOnce();
     loop_rate.sleep();
   }

   return 0;

 }
