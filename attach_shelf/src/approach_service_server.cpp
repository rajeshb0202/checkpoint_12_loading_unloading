#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "msgs_attach_shelf/action/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/empty.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/LinearMath/Quaternion.h>



using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using AttachShelfActionMessage = msgs_attach_shelf::action::GoToLoading;
using GoalHandleLoading = rclcpp_action::ServerGoalHandle<AttachShelfActionMessage>;

class ApproachActionServer : public rclcpp::Node  
{
public:
    ApproachActionServer() : Node("approach_service_server_node")  
    {

        number_table_legs_detected = 0;
        move_robot_status = false;
        reached_near_the_cart_status = false;
        moved_under_the_cart_status = false;
        first_time_moving_underneath_the_cart = true;
        back_to_loading_position_status = false;
        rotated_180_status = false;
        operation_complete_status = false;

        //laser scan subscriber
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ApproachActionServer::scan_callback, this, _1));
        
        //tf broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);
        elevator_up_publisher_ = this->create_publisher<std_msgs::msg::Empty>("elevator_up", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&ApproachActionServer::timer_callback, this));

        tf_buffer_ =  std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        //Adding service to the server
        this->action_server_ = rclcpp_action::create_server<AttachShelfActionMessage>(this, "attach_shelf_action", 
                                                                                    std::bind(&ApproachActionServer::handle_goal, this, _1, _2),
                                                                                    std::bind(&ApproachActionServer::handle_cancel, this, _1),
                                                                                    std::bind(&ApproachActionServer::handle_accepted, this, _1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&ApproachActionServer::odom_callback, this, _1));


    }



private:
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const AttachShelfActionMessage::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request %s", goal->attach_to_shelf ? "true" : "false");
        (void)uuid;
        if (!goal->attach_to_shelf)
        {
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleLoading> goal_handle)
    {
        //RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLoading> goal_handle)
    {
        std::thread{std::bind(&ApproachActionServer::execute, this, _1), goal_handle}.detach();
    }



    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
            //RCLCPP_INFO(this->get_logger(), "inside odom callback method");
            odom_msg_ = msg;
            current_x = odom_msg_->pose.pose.position.x;
            current_y = odom_msg_->pose.pose.position.y;

            //finding out the current yaw
            tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;

            m.getRPY(roll, pitch, current_yaw);

    }


    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
            //RCLCPP_INFO(this->get_logger(), "inside scan callback method");
            scan_msg_ = msg;
            detect_table_legs();
    }


    void execute(const std::shared_ptr<GoalHandleLoading> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        bool loop_running_status = true;
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<AttachShelfActionMessage::Feedback>();
        auto & message = feedback->feedback_operation_status;
        message = "Attaching to shelf operation going on";
        auto result = std::make_shared<AttachShelfActionMessage::Result>();
        
        while(loop_running_status && rclcpp::ok())
        {
            if (goal_handle -> is_canceling())
            {
                move_robot_status = false;
                result->complete = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal Cancelled");
            }

            move_robot_status = true;
            goal_handle->publish_feedback(feedback);


            //check if the goal is done
            if (rclcpp::ok() && operation_complete_status)
            {
                result->complete = true;
                loop_running_status = false;
                goal_handle->succeed(result);
                vel_msg_.linear.x = 0;
                vel_msg_.angular.z = 0;
                vel_publisher_->publish(vel_msg_);
                RCLCPP_INFO(this->get_logger(), "Goal Succeeded");

            }
            loop_rate.sleep();
        }
    }



    void timer_callback()
    {
        
        //RCLCPP_INFO(this->get_logger(), "attach_shelf_called status: %s", reached_near_the_cart_status ?"true":"false");        //for debugging
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
                //RCLCPP_INFO(this->get_logger(), "approaching the cart");            //debug
                vel_publisher_->publish(vel_msg_);
            }
            else if (error_distance < distance_gap_threshold)
            {
                vel_msg_.linear.x = 0;
                vel_msg_.angular.z = 0;
                reached_near_the_cart_status = true;
                //RCLCPP_INFO(this->get_logger(), "Reached near the cart");             //debug
                vel_publisher_->publish(vel_msg_);
            }

            
        }
        

        if (reached_near_the_cart_status && !moved_under_the_cart_status)
        {
            move_underneath_the_cart();
        }

        if (moved_under_the_cart_status && !rotated_180_status)
        {
            rotate_180();
        }

        if (rotated_180_status && !elevator_up_status)
        {
            elevator_up();
        }

        if (elevator_up_status && !back_to_loading_position_status)
        {
            back_to_loading_position();
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
            vel_publisher_->publish(vel_msg_);
        }
        else
        {
            vel_msg_.linear.x = 0;
            vel_msg_.angular.z = 0;
            moved_under_the_cart_status = true;
            RCLCPP_INFO(this->get_logger(), "moved underneath the cart successfully");
            vel_publisher_->publish(vel_msg_);
            // operation_complete_status = true;                       //for debug
        }
        //stop the robot at the end        
    }

    void rotate_180()
    {
        float target_yaw = 1.57;
        //rotate the robot until the current_yaw is +1.57 radians
        if (current_yaw < target_yaw)
        {
            vel_msg_.linear.x = 0;
            vel_msg_.angular.z = angular_speed;
            vel_publisher_->publish(vel_msg_);
            RCLCPP_INFO(this->get_logger(), "orienting towards openspace");      //for debug
        }
        else
        {
            vel_msg_.linear.x = 0;
            vel_msg_.angular.z = 0;
            vel_publisher_->publish(vel_msg_);
            rotated_180_status = true;
            RCLCPP_INFO(this->get_logger(), "Successfully rotated towards loading position");
            //operation_complete_status = true;                       //for debug
        }
    }

    void elevator_up()
    {
        std_msgs::msg::Empty up_msg;
        elevator_up_publisher_->publish(up_msg);
        elevator_up_status = true;
        RCLCPP_INFO(this->get_logger(), "lifted the shelf up successfully");
    }

    void back_to_loading_position()
    {
        rclcpp::WallRate loop_rate(2);       //2Hz
        
        vel_msg_.linear.x = translation_speed;
        vel_msg_.angular.z = angular_speed_back_loading;

        for (int i =0; i< time_to_move_loading_point; i++)
        {
            vel_publisher_->publish(vel_msg_);
            loop_rate.sleep();
        }
        
        vel_msg_.linear.x = 0;
        vel_msg_.angular.z = 0;
        vel_publisher_->publish(vel_msg_);

        back_to_loading_position_status = true;
        operation_complete_status = true;
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
        if (number_table_legs_detected == 2)
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
    rclcpp_action::Server<AttachShelfActionMessage>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr elevator_up_publisher_;

    sensor_msgs::msg::LaserScan::SharedPtr scan_msg_;
    geometry_msgs::msg::Twist vel_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    float translation_speed = 0.2; 
    float angular_speed = 0.4;
    float angular_speed_back_loading = 0.2;
    int intensity_threshold = 8000;
    float distance_gap_threshold = 0.06;
    float distance_to_be_moved_underneath = 0.30;
    int time_to_move_loading_point = 22;
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
    double current_yaw;


    bool tf_published_status;            //whether to start the tf listener or not
    bool move_robot_status;             //whether to move the robot or not
    bool reached_near_the_cart_status;    //whether the robot has moved near the cart or not
    bool moved_under_the_cart_status;   //whether the robot has moved under the cart or not
    bool first_time_moving_underneath_the_cart; //whether the robot has moved under the cart for the first time or not
    bool elevator_up_status;            //whether the elevator has moved up or not
    bool rotated_180_status;            // whether the robot has rotated 180 degrees after placing underneath the shelf
    bool back_to_loading_position_status; //whjether the robot is back to its loading position
    bool operation_complete_status;


};





int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApproachActionServer>());  
    rclcpp::shutdown();
    return 0;
}
