//headers in this package
#include <load_params.h>

//headers in gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
//#include <ignition/math/Vector3.hh>

namespace gazebo
{
    class gazebo_cog_publisher_plugin : public ModelPlugin
    {
        public:
            gazebo_cog_publisher_plugin()
            {

            }
            ~gazebo_cog_publisher_plugin()
            {

            }
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
            {
                this->model = _parent;
                for(auto link: this->model->GetLinks())
                {
                    links.push_back(link);
                }
                this->update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&gazebo_cog_publisher_plugin::OnUpdate, this));
                LoadParams(_sdf,"publish_rate",publish_rate,50.0);
                std::string reference_link_name;
                LoadParams(_sdf,"reference_link",reference_link_name,std::string("base_footprint"));
                reference_link = this->model->GetLink(reference_link_name);
                rate = common::Time(0, common::Time::SecToNano(publish_rate));
                this->prev_update_time = common::Time::GetWallTime();
            }
            void OnUpdate()
            {
                if (common::Time::GetWallTime() - this->prev_update_time < this->rate)
                    return;
                math::Vector3 total_cog_pos;
                for(auto link_itr = links.begin(); link_itr != links.end(); ++link_itr)
                {
                    physics::LinkPtr link_ptr = *link_itr;
                    physics::InertialPtr inertial_ptr = link_ptr->GetInertial();
                    math::Vector3 cog_pos = inertial_ptr->GetCoG();
                    total_cog_pos.x = cog_pos.x/links.size() + total_cog_pos.x;
                    total_cog_pos.y = cog_pos.y/links.size() + total_cog_pos.y;
                    total_cog_pos.z = cog_pos.z/links.size() + total_cog_pos.z;
                }
                math::Pose reference_link_pose = reference_link->GetWorldPose();
                this->prev_update_time = common::Time::GetWallTime();
            }
        private: 
            physics::ModelPtr model;
            std::vector<physics::LinkPtr> links;
            physics::LinkPtr reference_link;
            event::ConnectionPtr update_connection;
            double publish_rate;
            common::Time rate;
            common::Time prev_update_time;
    };
    GZ_REGISTER_MODEL_PLUGIN(gazebo_cog_publisher_plugin)
}