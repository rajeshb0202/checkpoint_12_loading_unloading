#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "msgs_attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/empty.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using MyCustomServiceMessage = msgs_attach_shelf::srv::GoToLoading;

class ApproachServiceServer : public rclcpp::Node  
{
public:
    ApproachServiceServer() : Node("approach_service_server_node")  
    {

        number_table_legs_detected = 0;
        move_robot_status = false;
        publish_tf_cart_frame_status = false;
        
        tf_published_status = false;
        reached_near_the_cart_status = false;
        moved_under_the_cart_status = false;
        first_time_moving_underneath_the_cart = true;

        odom_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        scan_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        odom_subscription_options_ = rclcpp::SubscriptionOptions();
        odom_subscription_options_.callback_group = odom_callback_group_;

        scan_subscription_options_ = rclcpp::SubscriptionOptions();
        scan_subscription_options_.callback_group = scan_callback_group_;

        //laser scan subscriber
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ApproachServiceServer::scan_callback, this, _1), scan_subscription_options_);
        
        //tf broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);
        elevator_up_publisher_ = this->create_publisher<std_msgs::msg::Empty>("elevator_up", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&ApproachServiceServer::timer_callback, this), scan_callback_group_);

        tf_buffer_ =  std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        //Adding service to the server
        service_ = this->create_service<MyCustomServiceMessage>("approach_service", std::bind(&ApproachServiceServer::service_callback, this, _1, _2));    

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&ApproachServiceServer::odom_callback, this, _1), odom_subscription_options_);


    }



private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
            odom_msg_ = msg;
            current_x = odom_msg_->pose.pose.position.x;
            current_y = odom_msg_->pose.pose.position.y;
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
            scan_msg_ = msg;
            detect_table_legs();
    }


    void service_callback(const std::shared_ptr<MyCustomServiceMessage::Request> request,  const std::shared_ptr<MyCustomServiceMessage::Response> response)
    {
        service_attach_shelf_called_status = true;
        if (number_table_legs_detected == 2)
        {
            publish_tf_cart_frame_status = true;
            if (request->attach_to_shelf)
            {
                move_robot_status = true;
                RCLCPP_INFO(this->get_logger(), "moving status: True");
                response->complete = true;
            }
            else if (!request->attach_to_shelf)
            {
                move_robot_status = false;
                RCLCPP_INFO(this->get_logger(), "moving status: False");
            }
        }
        else if(number_table_legs_detected < 2)
        {
            response->complete = false;
            RCLCPP_INFO(this->get_logger(), "Number of shelf legs detected less than 2. So aborting the service.");
        }
    }



    void timer_callback()
    {
        if (service_attach_shelf_called_status)
        {
            if (move_robot_status && tf_published_status && !reached_near_the_cart_status)
            {
                //get the transform between the cart and the robot
                geometry_msgs::msg::TransformStamped transformStamped;
                try
                {
                    transformStamped = tf_buffer_->lookupTransform(base_frame, child_frame, tf2::TimePointZero);
                }
                catch (tf2::TransformException &ex)
                {
                    RCLCPP_INFO(this->get_logger(), "tf listener failed ");
                    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
                    return;
                }

                //get the distance between the cart and the robot
                float error_distance = sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));
                float error_yaw= atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);

                //if the distance is greater than distance gap threshold, then move the robot or else stop the robot.
                if (error_distance > distance_gap_threshold)
                {
                    vel_msg_.linear.x = translation_speed;
                    vel_msg_.angular.z = error_yaw * angular_speed;
                }
                else if (error_distance < distance_gap_threshold)
                {
                    vel_msg_.linear.x = 0;
                    vel_msg_.angular.z = 0;
                    reached_near_the_cart_status = true;
                }

                vel_publisher_->publish(vel_msg_);
            }
            else
            {
                vel_msg_.linear.x = 0;
                vel_msg_.angular.z = 0;
                vel_publisher_->publish(vel_msg_);
            }

            if (reached_near_the_cart_status && !moved_under_the_cart_status)
            {
                move_underneath_the_cart();
            }

            if (moved_under_the_cart_status && !elevator_up_status)
            {
                elevator_up();
            }
        }

    }

    void move_underneath_the_cart()
    {
        if (first_time_moving_underneath_the_cart)
        {
            start_x = current_x;
            start_y = current_y;
            first_time_moving_underneath_the_cart = false;
        }
        
        float distance_moved = sqrt(pow((current_x - start_x), 2) + pow((current_y - start_y), 2));
        //RCLCPP_INFO(this->get_logger(), "distance moved: %f", distance_moved);

        if (distance_moved < distance_to_be_moved_underneath)
        {
            vel_msg_.linear.x = translation_speed;
            vel_msg_.angular.z = 0;
        }
        else
        {
            vel_msg_.linear.x = 0;
            vel_msg_.angular.z = 0;
            moved_under_the_cart_status = true;
            RCLCPP_INFO(this->get_logger(), "moved underneath the cart successfully");
        }
        //stop the robot at the end        
        vel_publisher_->publish(vel_msg_);
    }

    void elevator_up()
    {
        std_msgs::msg::Empty up_msg;
        elevator_up_publisher_->publish(up_msg);
        elevator_up_status = true;
        RCLCPP_INFO(this->get_logger(), "lifted the shelf up successfully");
        service_attach_shelf_called_status = false;
    }




    void detect_table_legs()
    {
        std::vector<int> table_legs_indexes;
        int start_angle_index = 0, end_angle_index = 0;
        bool leg_detection_going_on = false;
        number_table_legs_detected = 0;

        for (size_t i = 0; i < (scan_msg_ -> intensities.size()); i++)
        {
            if (scan_msg_ -> intensities[i] >= intensity_threshold)
            {
                if (!leg_detection_going_on)
                {
                    start_angle_index = i;   
                }
                
                end_angle_index = i;
                leg_detection_going_on = true;
            }
            else
            {
                if (leg_detection_going_on && start_angle_index != end_angle_index)
                {
                    number_table_legs_detected += 1;
                    leg_1_index = int((start_angle_index + end_angle_index)/2);
                    table_legs_indexes.push_back(leg_1_index);
                }
                leg_detection_going_on = false;
                start_angle_index = 0;
                end_angle_index = 0;
            }
        }

        // Check if the last leg was still being detected when the loop ended
        if (leg_detection_going_on && start_angle_index != end_angle_index)
        {
            number_table_legs_detected += 1;
            leg_1_index = int((start_angle_index + end_angle_index)/2);
            table_legs_indexes.push_back(leg_1_index);
        }

        //print the number of table legs detected- debugging
        //RCLCPP_INFO(this->get_logger(), "number of table legs detected: %d", number_table_legs_detected);

        //if there are 2 legs detected, then find the middle point and publish the tf frame
        if (number_table_legs_detected == 2 && publish_tf_cart_frame_status)
        {
            middle_point_table_legs(table_legs_indexes);
        }
    }



    void middle_point_table_legs(std::vector<int> table_legs_indexes)
    {
        float leg_1_x, leg_1_y, leg_2_x, leg_2_y;

        //find the angle of the table legs
        leg_1_x = calculate_coordinate(x_coordinate, table_legs_indexes[0]);
        leg_1_y = calculate_coordinate(y_coordinate, table_legs_indexes[0]);
        leg_2_x = calculate_coordinate(x_coordinate, table_legs_indexes[1]);
        leg_2_y = calculate_coordinate(y_coordinate, table_legs_indexes[1]);

        mid_point_x = (leg_1_x + leg_2_x)/2;
        mid_point_y = (leg_1_y + leg_2_y)/2;

        //RCLCPP_INFO(this->get_logger(), "mid point x: %f", mid_point_x);
        //RCLCPP_INFO(this->get_logger(), "mid point y: %f", mid_point_y);

        //publishing the tf frame of the cart and the robot
        publish_tf_cart_frame();
    }


    //calculate x and y oordinates
    float calculate_coordinate(std::string coordinate, int index)
    {
        float coordinate_value;
        if (coordinate == "x")
        {
            coordinate_value = scan_msg_ -> ranges[index] * cos(scan_msg_ -> angle_min + index * scan_msg_ -> angle_increment);
        }
        else if (coordinate == "y")
        {
            coordinate_value = scan_msg_ -> ranges[index] * sin(scan_msg_ -> angle_min + index *scan_msg_ -> angle_increment);
        }
        return coordinate_value;
    }


    //method to publish the tf frame of the cart
    void publish_tf_cart_frame()
    {
        //publish the tf frame
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = parent_frame;
        transformStamped.child_frame_id = child_frame;
        transformStamped.transform.translation.x = mid_point_x;
        transformStamped.transform.translation.y = mid_point_y;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transformStamped);
        //RCLCPP_INFO(this->get_logger(), "published the tf frame of the cart");

        tf_published_status = true;
    }



    //variables defined
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;   
    rclcpp::Service<MyCustomServiceMessage>::SharedPtr service_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
    rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
    rclcpp::SubscriptionOptions odom_subscription_options_;
    rclcpp::SubscriptionOptions scan_subscription_options_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_up_publisher_;

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
    geometry_msgs::msg::Twist vel_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    float translation_speed = 0.2; 
    float angular_speed = 0.2;
    int intensity_threshold = 8000;
    float distance_gap_threshold = 0.06;
    float distance_to_be_moved_underneath = 0.35;
    std::string parent_frame = "robot_front_laser_base_link";
    std::string child_frame = "cart_frame";
    std::string base_frame = "robot_base_link";

    
    int leg_1_index, leg_2_index;
    int number_table_legs_detected;
    float mid_point_x, mid_point_y;
    float start_x, start_y;
    float current_x, current_y;
    std::string x_coordinate = "x";
    std::string y_coordinate = "y";


    bool tf_published_status;            //whether to start the tf listener or not
    bool move_robot_status;             //whether to move the robot or not
    bool publish_tf_cart_frame_status;  //whether to publish the tf frame of the cart or not
    bool reached_near_the_cart_status;    //whether the robot has moved near the cart or not
    bool moved_under_the_cart_status;   //whether the robot has moved under the cart or not
    bool first_time_moving_underneath_the_cart; //whether the robot has moved under the cart for the first time or not
    bool elevator_up_status;            //whether the elevator has moved up or not
    bool service_attach_shelf_called_status;


};





int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApproachServiceServer>());  
    rclcpp::shutdown();
    return 0;
}
