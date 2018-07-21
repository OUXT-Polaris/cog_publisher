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
                this->rate = common::Time((int32_t)0, (int32_t)common::Time::SecToNano(1.0/publish_rate));
                this->prev_update_time = common::Time::GetWallTime();
                this->update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&gazebo_cog_publisher_plugin::OnUpdate, this));
            }
            void OnUpdate()
            {
                common::Time now = common::Time::GetWallTime();
                if(now - this->prev_update_time >= this->rate)
                {
                    geometry_msgs::PointStamped cog_msg;
                    math::Vector3 total_cog_pos,local_cog_pos;
                    double total_mass = 0;
                    total_cog_pos.x = 0;
                    total_cog_pos.y = 0;
                    total_cog_pos.z = 0;
                    for(auto link_itr = links.begin(); link_itr != links.end(); ++link_itr)
                    {
                        physics::LinkPtr link_ptr = *link_itr;
                        physics::InertialPtr inertial_ptr = link_ptr->GetInertial();
                        total_mass = total_mass + inertial_ptr->GetMass();
                    }
                    for(auto link_itr = links.begin(); link_itr != links.end(); ++link_itr)
                    {
                        physics::LinkPtr link_ptr = *link_itr;
                        physics::InertialPtr inertial_ptr = link_ptr->GetInertial();
                        math::Vector3 cog_pos = inertial_ptr->GetCoG();
                        total_cog_pos.x = cog_pos.x * inertial_ptr->GetMass() / total_mass + total_cog_pos.x;
                        total_cog_pos.y = cog_pos.y * inertial_ptr->GetMass() / total_mass + total_cog_pos.y;
                        total_cog_pos.z = cog_pos.z * inertial_ptr->GetMass() / total_mass + total_cog_pos.z;
                    }
                    math::Pose reference_link_pose = reference_link->GetWorldPose();
                    cog_msg.point.x = total_cog_pos.x;
                    cog_msg.point.y = total_cog_pos.y;
                    cog_msg.point.z = total_cog_pos.z;
                    cog_msg.header.frame_id = reference_link_name;
                    cog_msg.header.stamp.sec = now.sec;
                    cog_msg.header.stamp.nsec = now.nsec;
                    cog_pub.publish(cog_msg);
                    std_msgs::Float64 total_mass_msg;
                    total_mass_msg.data = total_mass;
                    total_mass_pub.publish(total_mass_msg);
                    /*
                    math::Matrix3 mat = quat_to_mat(reference_link_pose.rot);
                    math::Matrix3 inv_mat;
                    if(get_inv_mat(mat,inv_mat))
                    {
                        local_cog_pos.x = total_cog_pos.x;// - reference_link_pose.pos.x;
                        local_cog_pos.y = total_cog_pos.y;// - reference_link_pose.pos.y;
                        local_cog_pos.z = total_cog_pos.z;// - reference_link_pose.pos.z;
                        //local_cog_pos = inv_mat * local_cog_pos;
                        cog_msg.point.x = local_cog_pos.x;
                        cog_msg.point.y = local_cog_pos.y;
                        cog_msg.point.z = local_cog_pos.z;
                        cog_msg.header.frame_id = reference_link_name;
                        cog_msg.header.stamp.sec = now.sec;
                        cog_msg.header.stamp.nsec = now.nsec;
                        cog_pub.publish(cog_msg);
                        std_msgs::Float64 total_mass_msg;
                        total_mass_msg.data = total_mass;
                        total_mass_pub.publish(total_mass_msg);
                    }
                    */
                    this->prev_update_time = now;
                }
                return;
            }
            /*
            bool get_inv_mat(math::Matrix3 mat,math::Matrix3& inv_mat)
            {
                math::Matrix3 ret;
                double a = mat[0][0]*mat[1][1]*mat[2][2] + mat[0][1]*mat[1][2]*mat[2][0] + mat[0][2]*mat[1][0]*mat[2][1]
                    - mat[0][2]*mat[1][1]*mat[2][0] - mat[0][1]*mat[1][0]*mat[2][2] - mat[0][0]*mat[1][2]*mat[2][1];
                if(a == 0)
                {
                    return false;
                }
                inv_mat[0][0] =  (mat[1][1]*mat[2][2] - mat[1][2]*mat[2][1])/a;
                inv_mat[0][1] = (-mat[0][1]*mat[2][2] + mat[0][2]*mat[2][1])/a;
                inv_mat[0][2] =  (mat[0][1]*mat[1][2] - mat[0][2]*mat[1][1])/a;
                inv_mat[1][0] = (-mat[1][0]*mat[2][2] + mat[1][2]*mat[2][0])/a;
                inv_mat[1][1] =  (mat[1][1]*mat[2][2] - mat[0][2]*mat[2][0])/a;
                inv_mat[1][2] = (-mat[0][0]*mat[1][2] + mat[0][2]*mat[1][0])/a;
                inv_mat[2][0] =  (mat[1][0]*mat[2][1] - mat[1][1]*mat[2][0])/a;
                inv_mat[2][1] = (-mat[0][0]*mat[2][1] + mat[0][1]*mat[2][0])/a;
                inv_mat[2][2] =  (mat[0][0]*mat[1][1] - mat[0][1]*mat[1][0])/a;
                return true;
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
            */
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