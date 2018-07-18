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
            public: void Load(physics::ModelPtr /*_parent*/, sdf::ElementPtr /*_sdf*/)
            {

            }
    };
    GZ_REGISTER_MODEL_PLUGIN(gazebo_cog_publisher_plugin)
}