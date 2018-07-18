//headers in this package
#include <load_params.h>

//headers in gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

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
                rate = common::Time(0, common::Time::SecToNano(publish_rate));
                this->prev_update_time = common::Time::GetWallTime();
            }
            void OnUpdate()
            {
                if (common::Time::GetWallTime() - this->prev_update_time < this->rate)
                    return;
                this->prev_update_time = common::Time::GetWallTime();
            }
        private: 
            physics::ModelPtr model;
            std::vector<physics::LinkPtr> links;
            event::ConnectionPtr update_connection;
            double publish_rate;
            common::Time rate;
            common::Time prev_update_time;
    };
    GZ_REGISTER_MODEL_PLUGIN(gazebo_cog_publisher_plugin)
}