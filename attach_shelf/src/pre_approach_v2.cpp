#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <memory>
#include "msgs_attach_shelf/srv/go_to_loading.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using MyCustomServiceMessage = msgs_attach_shelf::srv::GoToLoading;



class PreApproach : public rclcpp::Node
{
    public:
        PreApproach():Node("pre_approach_node")
        {
            //defining parameters
            this->declare_parameter("obstacle", 0.0);
            this->declare_parameter("degrees", 0);
            this->declare_parameter("final_approach", false);

            //intialising the variables
            getting_params();
            dest_reached = false;
            rotate_status = false;
            called_server = false;

            //callback groups
            odom_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            scan_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            //creating subscriptions options
            rclcpp::SubscriptionOptions odom_options;
            odom_options.callback_group = odom_callback_group;

            rclcpp::SubscriptionOptions scan_options;
            scan_options.callback_group = scan_callback_group;


            //creating subscribers and publishers
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
            scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PreApproach::scan_callback, this, _1), odom_options);
            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PreApproach::odom_callback, this, _1), scan_options);

            //service client
            service_client_ = this->create_client<MyCustomServiceMessage>("/approach_service");
           
            RCLCPP_INFO(this->get_logger(), "pre_approach node has started!...");
        }

    private:
        void getting_params()
        {
            obstacle_distance_parameter = this->get_parameter("obstacle").get_parameter_value().get<float>();
            angle_to_be_rotated_parameter = (this->get_parameter("degrees").get_parameter_value().get<int>()) *  (M_PI / 180.0);
            final_approach_parameter = this->get_parameter("final_approach").as_bool();      

        }



        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            tf2::Quaternion quat;
            tf2::fromMsg(msg->pose.pose.orientation, quat);

            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            current_yaw_angle = yaw; 
        }



        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            front_laser_array_index= int((0 - msg->angle_min)/(msg->angle_increment));
            front_laser_reading = msg->ranges[front_laser_array_index];

    
            if (front_laser_reading > obstacle_distance_parameter  && !dest_reached)
            {
               vel_msg.linear.x = translation_speed;
            }
            else
            {
                vel_msg.linear.x = 0.0;
                dest_reached = true;
            }
            
            publisher_->publish(vel_msg);

            if (dest_reached && !rotate_status)
            {
                rotate_robot();
            }

            //calling the service after the roobt has rotated
            if (rotate_status && dest_reached && !called_server)
            {
                call_server();
            }
        }

        void rotate_robot()
        {
            //setting the velocities for rotation
            vel_msg.linear.x = 0;
            if (angle_to_be_rotated_parameter > 0)
            {
                vel_msg.angular.z = angular_speed;
            }
            else if( angle_to_be_rotated_parameter < 0)
            {
                vel_msg.angular.z = -1 * angular_speed;
            }
            else
            {
                rotate_status = true;
                return;
            }


            RCLCPP_INFO(this->get_logger(), "Rotation started...");
            rclcpp::Rate loop_rate(100);
            float target_angle = current_yaw_angle + angle_to_be_rotated_parameter;

            while (fabs(target_angle - current_yaw_angle) > yaw_threshold)
            {
                publisher_->publish(vel_msg);
                loop_rate.sleep();
            }

            //stopping the robot after rotation
            vel_msg.linear.x = 0.0;
            vel_msg.angular.z = 0;
            publisher_->publish(vel_msg);
            rotate_status = true;
            RCLCPP_INFO(this->get_logger(), "Rotation of %0.2f degrees successfully done...", (angle_to_be_rotated_parameter * 180/M_PI));
        }

        //method to call the service
        void call_server()
        {
            auto request = std::make_shared<MyCustomServiceMessage::Request>();
            request->attach_to_shelf = final_approach_parameter;

            auto result_future = service_client_->async_send_request(request, std::bind(&PreApproach::service_response_callback, this, std::placeholders::_1));
            called_server = true;
        }

        //callback function for service
        void service_response_callback(rclcpp::Client<MyCustomServiceMessage>::SharedFuture future)
        {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready) 
            {
                RCLCPP_INFO(this->get_logger(), "Service called successfully and its response: %s", future.get()->complete? "true" : "false");
            } 
            else 
            {
                RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
            }
        }




    //defining variables
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_ ;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Client<MyCustomServiceMessage>::SharedPtr service_client_;
        
        geometry_msgs::msg::Twist vel_msg;
        //nav_msgs::msg::Odometry odom_msg;

        rclcpp::CallbackGroup::SharedPtr odom_callback_group;
        rclcpp::CallbackGroup::SharedPtr scan_callback_group;

        int front_laser_array_index;
        float front_laser_reading;
        float obstacle_distance_parameter;
        float angle_to_be_rotated_parameter;
        bool final_approach_parameter;

        bool dest_reached;
        bool rotate_status;
        bool called_server;
        float angular_speed = 0.2;
        float translation_speed = 0.2;
        float current_yaw_angle;
        float yaw_threshold= 0.02;
};




int main (int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto pre_approach_node_obj = std::make_shared<PreApproach>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(pre_approach_node_obj);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}