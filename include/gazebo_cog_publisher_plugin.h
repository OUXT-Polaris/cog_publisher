#ifndef GAZEBO_COG_PUBLISHER_H_INCLUDED
#define GAZEBO_COG_PUBLISHER_H_INCLUDED

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
            gazebo_cog_publisher_plugin();
            ~gazebo_cog_publisher_plugin();
    };
}

#endif  //GAZEBO_COG_PUBLISHER_H_INCLUDED