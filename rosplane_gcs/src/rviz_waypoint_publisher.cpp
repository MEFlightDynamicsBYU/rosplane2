#include "rclcpp/rclcpp.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
#include "rclcpp/logging.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "rosplane_msgs/msg/waypoint.hpp"
#include "rosplane_msgs/msg/state.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "std_msgs/msg/header.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#define SCALE 5.0
#define TEXT_SCALE 15.0
using std::placeholders::_1;

namespace rosplane_gcs
{

class rviz_waypoint_publisher : public rclcpp::Node
{
public:
    rviz_waypoint_publisher();
    ~rviz_waypoint_publisher();

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_wp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_mesh_pub_;
    rclcpp::Subscription<rosplane_msgs::msg::Waypoint>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<rosplane_msgs::msg::State>::SharedPtr vehicle_state_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> aircraft_tf2_broadcaster_;

    void new_wp_callback(const rosplane_msgs::msg::Waypoint & wp);
    void state_update_callback(const rosplane_msgs::msg::State & state);
    void update_list();
    void update_mesh();

    rosplane_msgs::msg::State vehicle_state_;

    // Persistent rviz markers
    visualization_msgs::msg::Marker line_list_;
    std::vector<geometry_msgs::msg::Point> line_points_;
    visualization_msgs::msg::Marker aircraft_;

    int num_wps_;
};

rviz_waypoint_publisher::rviz_waypoint_publisher()
    : Node("rviz_waypoint_publisher") {
    
    rclcpp::QoS qos_transient_local_20_(20);
    qos_transient_local_20_.transient_local();
    rviz_wp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/waypoint", qos_transient_local_20_);
    rviz_mesh_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/mesh", 5);
    waypoint_sub_ = this->create_subscription<rosplane_msgs::msg::Waypoint>("waypoint_path", qos_transient_local_20_,
            std::bind(&rviz_waypoint_publisher::new_wp_callback, this, _1));
    vehicle_state_sub_ = this->create_subscription<rosplane_msgs::msg::State>("estimated_state", 10,
            std::bind(&rviz_waypoint_publisher::state_update_callback, this, _1));
    
    aircraft_tf2_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    
    // Create Line List
    update_list();

    // Initialize aircraft
    aircraft_.header.frame_id = "stl_frame";
    aircraft_.ns = "vehicle";
    aircraft_.id = 0;
    aircraft_.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    aircraft_.mesh_resource = "file:///home/jacob/MAGICC/rosflight_ws/src/rosplane/rosplane_gcs/resource/skyhunter.dae";
    aircraft_.mesh_use_embedded_materials = false;
    aircraft_.action = visualization_msgs::msg::Marker::ADD;
    aircraft_.pose.position.x = 0.0;
    aircraft_.pose.position.y = 0.0;
    aircraft_.pose.position.z = 0.0;
    aircraft_.pose.orientation.x = 0.0;
    aircraft_.pose.orientation.y = 0.0;
    aircraft_.pose.orientation.z = 0.0;
    aircraft_.pose.orientation.w = 1.0;
    aircraft_.scale.x = 5.0;
    aircraft_.scale.y = 5.0;
    aircraft_.scale.z = 5.0;
    aircraft_.color.r = 0.67f;
    aircraft_.color.g = 0.67f;
    aircraft_.color.b = 0.67f;
    aircraft_.color.a = 1.0;

    num_wps_ = 0;
}

rviz_waypoint_publisher::~rviz_waypoint_publisher() {}

void rviz_waypoint_publisher::new_wp_callback(const rosplane_msgs::msg::Waypoint & wp) {
    visualization_msgs::msg::Marker new_marker;

    if (wp.clear_wp_list) {
        rclcpp::Time now = this->get_clock()->now();
        new_marker.header.stamp = now;
        new_marker.header.frame_id = "NED";
        new_marker.ns = "wp";
        new_marker.id = 0;
        new_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        rviz_wp_pub_->publish(new_marker);
        num_wps_ = 0;
        return;
    }
    
    // Create marker
    rclcpp::Time now = this->get_clock()->now();
    new_marker.header.stamp = now;
    new_marker.header.frame_id = "NED";
    new_marker.ns = "wp";
    new_marker.id = num_wps_;
    new_marker.type = visualization_msgs::msg::Marker::SPHERE;
    new_marker.action = visualization_msgs::msg::Marker::ADD;
    new_marker.pose.position.x = wp.w[0];
    new_marker.pose.position.y = wp.w[1];
    new_marker.pose.position.z = wp.w[2];
    new_marker.scale.x = SCALE;
    new_marker.scale.y = SCALE;
    new_marker.scale.z = SCALE;
    new_marker.color.r = 1.0f;
    new_marker.color.g = 0.0f;
    new_marker.color.b = 0.0f;
    new_marker.color.a = 1.0;
    
    // Add point to line list
    geometry_msgs::msg::Point new_p;
    new_p.x = wp.w[0];
    new_p.y = wp.w[1];
    new_p.z = wp.w[2];
    line_points_.push_back(new_p);
    update_list();

    // Add Text label to marker
    visualization_msgs::msg::Marker new_text;
    new_text.header.stamp = now;
    new_text.header.frame_id = "NED";
    new_text.ns = "text";
    new_text.id = num_wps_;
    new_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    new_text.action = visualization_msgs::msg::Marker::ADD;
    new_text.pose.position.x = wp.w[0];
    new_text.pose.position.y = wp.w[1];
    new_text.pose.position.z = wp.w[2] - SCALE - 1.0;
    new_text.scale.z = TEXT_SCALE;
    new_text.color.r = 0.0f;
    new_text.color.g = 0.0f;
    new_text.color.b = 0.0f;
    new_text.color.a = 1.0;
    new_text.text = std::to_string(num_wps_);

    rviz_wp_pub_->publish(new_marker);
    rviz_wp_pub_->publish(line_list_);
    rviz_wp_pub_->publish(new_text);

    ++num_wps_;
}

void rviz_waypoint_publisher::update_list() {
    rclcpp::Time now = this->get_clock()->now();
    line_list_.header.stamp = now;
    line_list_.header.frame_id = "NED";
    line_list_.ns = "path";
    line_list_.id = 0;
    line_list_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_list_.action = visualization_msgs::msg::Marker::ADD;
    line_list_.scale.x = 1.0;
    line_list_.scale.y = 1.0;
    line_list_.scale.z = 1.0;
    line_list_.color.r = 0.0f;
    line_list_.color.g = 1.0f;
    line_list_.color.b = 0.0f;
    line_list_.color.a = 1.0;
    line_list_.points = line_points_;
}

void rviz_waypoint_publisher::update_mesh() {
    rclcpp::Time now = this->get_clock()->now();
    aircraft_.header.stamp = now;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "NED";
    t.child_frame_id = "aircraft_body";
    t.transform.translation.x = vehicle_state_.position[0];
    t.transform.translation.y = vehicle_state_.position[1];
    t.transform.translation.z = vehicle_state_.position[2];

    tf2::Quaternion q;
    q.setRPY(vehicle_state_.phi, vehicle_state_.theta, vehicle_state_.chi);
    t.transform.rotation.x = q.x(); //0.0; //vehicle_state_.quat[0];
    t.transform.rotation.y = q.y(); //0.0; //vehicle_state_.quat[1];
    t.transform.rotation.z = q.z(); //0.0; //vehicle_state_.quat[2];
    t.transform.rotation.w = q.w(); //1.0; //vehicle_state_.quat[3];

    aircraft_tf2_broadcaster_->sendTransform(t);
    rviz_mesh_pub_->publish(aircraft_);
}

void rviz_waypoint_publisher::state_update_callback(const rosplane_msgs::msg::State & msg) {
    vehicle_state_ = msg;
    update_mesh();
}

} // namespace rosplane_gcs

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rosplane_gcs::rviz_waypoint_publisher>();

    rclcpp::spin(node);

    return 0;
}