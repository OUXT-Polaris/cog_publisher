//headers in this package
#include <load_params.h>

//headers in gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/math/gzmath.hh>
#include <gazebo/math/Matrix3.hh>

//headers in ROS
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>

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
                cog_pub = nh.advertise<geometry_msgs::PointStamped>("/cog/robot", 1);
                total_mass_pub = nh.advertise<std_msgs::Float64>("/cog/total_mass", 1);
                LoadParams(_sdf,"publish_rate",publish_rate,50.0);
                LoadParams(_sdf,"reference_link",reference_link_name,std::string("base_footprint"));
                reference_link = this->model->GetLink(reference_link_name);

                // set connection
                this->update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&gazebo_cog_publisher_plugin::OnUpdate, this));
                rate = common::Time(0, common::Time::SecToNano(publish_rate));
                this->prev_update_time = common::Time::GetWallTime();
            }
            void OnUpdate()
            {
                if (common::Time::GetWallTime() - this->prev_update_time < this->rate)
                    return;
                geometry_msgs::PointStamped cog_msg;
                math::Vector3 total_cog_pos,local_cog_pos;
                double total_mass = 0;
                for(auto link_itr = links.begin(); link_itr != links.end(); ++link_itr)
                {
                    physics::LinkPtr link_ptr = *link_itr;
                    physics::InertialPtr inertial_ptr = link_ptr->GetInertial();
                    math::Vector3 cog_pos = inertial_ptr->GetCoG();
                    total_cog_pos.x = cog_pos.x * inertial_ptr->GetMass() + total_cog_pos.x;
                    total_cog_pos.y = cog_pos.y * inertial_ptr->GetMass() + total_cog_pos.y;
                    total_cog_pos.z = cog_pos.z * inertial_ptr->GetMass() + total_cog_pos.z;
                    total_mass = total_mass + inertial_ptr->GetMass();
                }
                math::Pose reference_link_pose = reference_link->GetWorldPose();
                local_cog_pos.x = total_cog_pos.x/total_mass - reference_link_pose.pos.x;
                local_cog_pos.y = total_cog_pos.y/total_mass - reference_link_pose.pos.y;
                local_cog_pos.z = total_cog_pos.z/total_mass - reference_link_pose.pos.z;
                math::Matrix3 mat = quat_to_mat(reference_link_pose.rot);
                local_cog_pos = mat * local_cog_pos;
                cog_msg.point.x = local_cog_pos.x;
                cog_msg.point.y = local_cog_pos.y;
                cog_msg.point.z = local_cog_pos.z;
                cog_msg.header.frame_id = reference_link_name;
                cog_msg.header.stamp = ros::Time::now();
                cog_pub.publish(cog_msg);
                std_msgs::Float64 total_mass_msg;
                total_mass_msg.data = total_mass;
                total_mass_pub.publish(total_mass_msg);
                this->prev_update_time = common::Time::GetWallTime();
                return;
            }
            math::Matrix3 quat_to_mat(math::Quaternion q)
            {
                double sx = q.x * q.x;
                double sy = q.y * q.y;
                double sz = q.z * q.z;
                double cx = q.y * q.z;
                double cy = q.x * q.z;
                double cz = q.x * q.y;
                double wx = q.w * q.x;
                double wy = q.w * q.y;
                double wz = q.w * q.z;
                double _v00 = 1.0d - 2.0d * (sy + sz);
                double _v01 = 2.0d * (cz + wz);
                double _v02 = 2.0d * (cy - wy);
                double _v10 = 2.0d * (cz - wz);
                double _v11 = 1.0d - 2.0d * (sx + sz);
                double _v12 = 2.0d * (cx + wx);
                double _v20 = 2.0d * (cy + wy);
                double _v21 = 2.0d * (cx - wx);
                double _v22 = 1.0d - 2.0d * (sx + sy);
                math::Matrix3 ret(_v00,_v01,_v02,_v10,_v11,_v12,_v20,_v21,_v22);
                return ret;
            }
        private: 
            physics::ModelPtr model;
            std::vector<physics::LinkPtr> links;
            physics::LinkPtr reference_link;
            std::string reference_link_name;
            event::ConnectionPtr update_connection;
            double publish_rate;
            common::Time rate;
            common::Time prev_update_time;
            ros::NodeHandle nh;
            ros::Publisher total_mass_pub,cog_pub;
    };
    GZ_REGISTER_MODEL_PLUGIN(gazebo_cog_publisher_plugin)
}