#include <ros/ros.h>

#include <spatial_knowledge/sk_client.h>
#include <erl_planning/KMS_Client.h>


class Object: public SKClient, public KMS_Client
{
public:
	Object(std::string name, std::string cat)
	: SKClient("object_"+name),
		name_(name),
		category_(cat)
	{
		add_instance("object", name_);

		Fact fact_cat;
		fact_cat.push_back(Fact_Values("p", name_));
		add_fact(category_+"_object", fact_cat);
	}

private:
	std::string name_;
	std::string category_;
};


class SKObject
{
public:
	SKObject(): nh_("~")
	{
		start_object();
	}

	void start_object()
	{
		if(nh_.hasParam("object/object_ids"))
		{
			std::vector<std::string> object;
			nh_.getParam("object/object_ids", object);

			for(std::vector<std::string>::iterator it=object.begin(); it!=object.end(); ++it)
			{
				printf("Creating [%s]", it->c_str());

				if(nh_.hasParam("object/"+*it))
				{
					std::map<std::string, std::string> info;
					nh_.getParam("object/"+*it, info);

					object_[*it] = std::shared_ptr<Object>(new Object(*it, info["category"]));

					double cx, cy, cz;
					double dx, dy, dz;

					sscanf(info["position"].c_str(), "%lf, %lf, %lf", &cx, &cy, &cz);
					sscanf(info["dimensions"].c_str(), "%lf, %lf, %lf", &dx, &dy, &dz);

					octomap::point3d start(cx-dx/2, cy-dy/2, cz-dz/2);
					octomap::point3d end(cx+dx/2, cy+dy/2, cz+dz/2);

					object_[*it]->updateSK(start, end);
				}
			}
		}
	}

	void update()
	{
			std::map<std::string, std::shared_ptr<Object> >::iterator it;
			for(it=object_.begin(); it!=object_.end(); ++it)
				it->second->publish();
	}

private:
	ros::NodeHandle nh_;

	std::map<std::string, std::shared_ptr<Object> > object_;

};


int main(int argc, char **argv)
{

   ros::init(argc, argv, "sk_object");
   ros::NodeHandle n;

	 SKObject skobject;

   ros::Rate loop_rate(1);

   int count=0;

   while (ros::ok())
   {

		 skobject.update();

     ros::spinOnce();
     loop_rate.sleep();
   }

   return 0;

 }
