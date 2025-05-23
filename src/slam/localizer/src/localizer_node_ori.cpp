#include <queue>
#include <mutex>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/fastlio2/body_cloud";
    std::string odom_topic = "/fastlio2/lio_odom";
    std::string map_frame = "map";
    std::string local_frame = "lidar";
    std::string map_path = "pointcloud.pcd";
    double update_hz = 1.0;
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("localizer_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        loadParameters();

        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

        m_initial_pose_sub = 
            this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/initialpose", 10, std::bind(&LocalizerNode::InitialPoseCB, this, std::placeholders::_1));

        m_reloc_srv = 
            this->create_service<interface::srv::Relocalize>("relocalize", 
                std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));
        m_reloc_check_srv = 
            this->create_service<interface::srv::IsValid>("relocalize_check", 
                std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_map_cloud_pub = 
            this->create_publisher<sensor_msgs::msg::PointCloud2>("/overall_map", 
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
        m_scan_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 2);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 10);
        
        m_timer = this->create_wall_timer(100ms, std::bind(&LocalizerNode::timerCB, this));

        if (m_localizer->loadMap(m_config.map_path)) {
            rclcpp::Time map_time = this->now();
            builtin_interfaces::msg::Time current_time;
            current_time.sec = static_cast<int32_t>(map_time.seconds());
            current_time.nanosec = map_time.nanoseconds() % 1000000000L;
            publishMapCloud(current_time);
        }
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        this->declare_parameter<std::string>("map_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_config.map_path = this->get_parameter("map_path").as_string();
        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        m_config.update_hz = config["update_hz"].as<double>();

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();

        std::cout << "map_path: " << m_config.map_path << std::endl;
        std::cout << "cloud_topic: " << m_config.cloud_topic << std::endl;
        std::cout << "odom_topic: " << m_config.odom_topic << std::endl;
        std::cout << "map_frame: " << m_config.map_frame << std::endl;
        std::cout << "local_frame: " << m_config.local_frame << std::endl;
        std::cout << "update_hz: " << m_config.update_hz << std::endl;

        std::cout << "rough_scan_resolution: " << m_localizer_config.rough_scan_resolution << std::endl;
        std::cout << "rough_map_resolution: " << m_localizer_config.rough_map_resolution << std::endl;
        std::cout << "rough_max_iteration: " << m_localizer_config.rough_max_iteration << std::endl;
        std::cout << "rough_score_thresh: " << m_localizer_config.rough_score_thresh << std::endl;

        std::cout << "refine_scan_resolution: " << m_localizer_config.refine_scan_resolution << std::endl;
        std::cout << "refine_map_resolution: " << m_localizer_config.refine_map_resolution << std::endl;
        std::cout << "refine_max_iteration: " << m_localizer_config.refine_max_iteration << std::endl;
        std::cout << "refine_score_thresh: " << m_localizer_config.refine_score_thresh << std::endl;
    }
    void timerCB()
    {
        if (!m_state.message_received)
            return;

        rclcpp::Duration diff = rclcpp::Clock().now() - m_state.last_send_tf_time;

        bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_received;

        if (!update_tf)
        {
            sendBroadCastTF(m_state.last_message_time);
            return;
        }

        m_state.last_send_tf_time = rclcpp::Clock().now();

        M4F initial_guess = M4F::Identity();
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex>(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
            // m_state.service_received = false;
        }
        else
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }

        bool result = m_localizer->align(initial_guess);
        if (result)
        {
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            // check z 
            // if (map_body_t[2] > 0.02) {
            //     map_body_t[2] = 0.02;
            // } else if (map_body_t[2] < -0.02) {
            //     map_body_t[2] = -0.02;
            // }
            m_state.last_offset_r = map_body_r * current_local_r.transpose();
            m_state.last_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
            if (!m_state.localize_success && m_state.service_received)
            {
                RCLCPP_INFO(this->get_logger(), "Reloc success!");
                std::lock_guard<std::mutex>(m_state.service_mutex);
                m_state.localize_success = true;
                m_state.service_received = false;
            }
        } else if (!m_state.localize_success && m_state.service_received) {
            // RCLCPP_WARN(this->get_logger(), "Reloc failed!");
        }
        sendBroadCastTF(current_time);
        // publishMapCloud(current_time);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex>(m_state.message_mutex);

        pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

        m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                            odom_msg->pose.pose.orientation.x,
                                            odom_msg->pose.pose.orientation.y,
                                            odom_msg->pose.pose.orientation.z)
                             .toRotationMatrix();
        m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                             odom_msg->pose.pose.position.y,
                             odom_msg->pose.pose.position.z);
        m_state.last_message_time = cloud_msg->header.stamp;
        if (!m_state.message_received)
        {
            m_state.message_received = true;
            m_config.local_frame = odom_msg->header.frame_id;
        }
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = 0.0; // t.z()
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
        // get local -> body value
        M3D local_body_r;
        V3D local_body_t;
        builtin_interfaces::msg::Time local_time;
        CloudType scan_cloud;
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            local_body_r = m_state.last_r;
            local_body_t = m_state.last_t;
            local_time = m_state.last_message_time;
            scan_cloud = *m_state.last_cloud;
        }

        M3D map_body_r = m_state.last_offset_r * local_body_r;
        V3D map_body_t = m_state.last_offset_t + m_state.last_offset_r * local_body_t;

        // publish scan cloud in map frame
        int scan_cloud_num = scan_cloud.points.size();
        for (int i = 0; i < scan_cloud_num; i++) {
            pointBodyToWorld(&(scan_cloud.points[i]), &(scan_cloud.points[i]), map_body_r, map_body_t);
        }
        sensor_msgs::msg::PointCloud2 map_scan_cloud;
        pcl::toROSMsg(scan_cloud, map_scan_cloud);
        map_scan_cloud.header.stamp = local_time;
        map_scan_cloud.header.frame_id = "map";

        m_scan_cloud_pub->publish(map_scan_cloud);

        static bool get_body_sensor_static_tf = false;
        if (!get_body_sensor_static_tf) {
            try {
                geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer_.lookupTransform("body", "sensor", tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "Transform received: Translation (%.2f, %.2f, %.2f), Rotation [%.2f, %.2f, %.2f, %.2f]",
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z,
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z,
                transform_stamped.transform.rotation.w);
                
                q_body_sensor_.w() = transform_stamped.transform.rotation.w;
                q_body_sensor_.x() = transform_stamped.transform.rotation.x;
                q_body_sensor_.y() = transform_stamped.transform.rotation.y;
                q_body_sensor_.z() = transform_stamped.transform.rotation.z;
                get_body_sensor_static_tf = true;
            } catch(const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
                return;
            }
        }
        // pub odometry for navigation only when get the static tf between body and sensor
        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = local_time;
        odometry.header.frame_id = "map";
        odometry.child_frame_id = "sensor";
        // set z = 0
        map_body_t[2] = 0.0;
        Eigen::Quaterniond q_map_body(map_body_r);
        Eigen::Quaterniond q_map_sensor = q_map_body * q_body_sensor_;
        odometry.pose.pose.orientation.x = q_map_sensor.x();
        odometry.pose.pose.orientation.y = q_map_sensor.y();
        odometry.pose.pose.orientation.z = q_map_sensor.z();
        odometry.pose.pose.orientation.w = q_map_sensor.w();

        odometry.pose.pose.position.x = map_body_t.x();
        odometry.pose.pose.position.y = map_body_t.y();
        odometry.pose.pose.position.z = map_body_t.z();
        m_odom_pub->publish(odometry);
    }

    void pointBodyToWorld(PointType const * const pi, PointType * const po, const M3D& rot, const V3D& pos)
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(rot * p_body + pos);
    
        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    Eigen::Vector3d QuatnionToEulerAngles(Eigen::Quaterniond quat)
    {
      double roll = atan2(2*(quat.w()*quat.x() + quat.y()*quat.z()), 1-2*(quat.x()*quat.x() + quat.y()*quat.y()));
      double pitch = asin(2*(quat.w()*quat.y() - quat.z()*quat.x()));
      double yaw = atan2(2*(quat.w()*quat.z() + quat.x()*quat.y()), 1-2*(quat.z()*quat.z() + quat.y()*quat.y()));
    
      return Eigen::Vector3d(yaw, pitch, roll);
    }

    void InitialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr init_pose) {
        const float init_x = init_pose->pose.pose.position.x;
        const float init_y = init_pose->pose.pose.position.y;
        const float init_z = init_pose->pose.pose.position.z;
        const auto init_quat = init_pose->pose.pose.orientation;
        Eigen::Quaterniond quat_eigen(init_quat.w, init_quat.x, init_quat.y, init_quat.z);
        Eigen::Vector3f euler_angle = QuatnionToEulerAngles(quat_eigen).cast<float>();
        const float roll = euler_angle[2];
        const float pitch = euler_angle[1];
        const float yaw = euler_angle[0];
        Eigen::Matrix3f rot_matrix = quat_eigen.toRotationMatrix().cast<float>();
        RCLCPP_INFO(this->get_logger(), "Received initial pose, pos: {%f, %f, %f}, roll: %f, pitch: %f, yaw: %f.", init_x, init_y, init_z, roll, pitch, yaw);
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            m_state.initial_guess.setIdentity();
            // m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 3>(0, 0) = rot_matrix;
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(init_x, init_y, init_z);
            m_state.service_received = true;
            m_state.localize_success = false;
        }        
    }

    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
        }

        response->success = true;
        response->message = "relocalize success";
        return;
    }

    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex>(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        // if (m_map_cloud_pub->get_subscription_count() < 1)
        //     return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1) {
            RCLCPP_WARN(this->get_logger(), "Too little points in map cloud, DO NOT publish!");
            return;
        }
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    NodeConfig m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    Eigen::Quaterniond q_body_sensor_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_scan_cloud_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initial_pose_sub;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
