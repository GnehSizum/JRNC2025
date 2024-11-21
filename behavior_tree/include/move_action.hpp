#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <chrono>

class MoveActionNode : public BT::StatefulActionNode
{
  public:
    MoveActionNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config), 
          client_node(rclcpp::Node::make_shared("bt_navigator_node")), 
          tf_buffer(std::make_shared<tf2_ros::Buffer>(client_node->get_clock())),
          tf_listener(std::make_shared<tf2_ros::TransformListener>(*tf_buffer)),
          logger(client_node->get_logger())
    {
        nav2_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(client_node, "navigate_to_pose");
    }

    /**
     * @brief  定义该行为树节点所需的输入端口
     * 
     * @return 返回一个端口列表，包含了目标位姿的三个参数(x, y, 和 theta)
     */
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("x"), 
                 BT::InputPort<double>("y"), 
                 BT::InputPort<double>("theta") };
    }

    /**
     * @brief   在每次行为开始时调用，设定导航目标
     * 
     * @details 1.等待 NavigateToPose 服务器响应，以确保机器人可以进行导航。
     *          2.从输入端口获取目标位姿参数 x, y, 和 theta。
     *          3.更新 goal_pose 机器人导航的目标位姿
     *          4.返回 BT::NodeStatus::RUNNING，表示行为正在进行中。
     * 
     * @return  BT::NodeStatus 返回行为节点状态
     */
    BT::NodeStatus onStart() override {
        if(!nav2_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(logger, "NavigateToPose server not available!");
            return BT::NodeStatus::FAILURE;
        }

        double x, y, theta;
        if(!getInput("x", x) || !getInput("y", y) || !getInput("theta", theta)) {
            RCLCPP_ERROR(logger, "Failed to get input pose parameters.");
            return BT::NodeStatus::FAILURE;
        }

        goal_pose.pose.header.frame_id = "map";
        goal_pose.pose.header.stamp = client_node->get_clock()->now();
        goal_pose.pose.pose.position.x = x;
        goal_pose.pose.pose.position.y = y;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        goal_pose.pose.pose.orientation = tf2::toMsg(q);

        return BT::NodeStatus::RUNNING;
    }

    /**
     * @brief   在每次执行周期中调用，检查导航状态
     * 
     * @details 1.调用 getCurrentPose 函数，获取机器人的当前位置。如果无法获取当前位置，则返回 FAILURE。
     *          2.计算当前位置与目标位置的距离，如果距离大于 0.15 米，向导航服务端发送目标位置请求，并返回 RUNNING，表示行为还在执行中。
     *          3.如果机器人已接近目标位置（小于 0.15 米），返回 SUCCESS 表示行为导航成功。
     * 
     * @return  BT::NodeStatus 返回行为节点状态
     */
    BT::NodeStatus onRunning() override {
        if (!getCurrentPose()) {
            RCLCPP_ERROR(logger, "Failed to get current pose from TF.");
            return BT::NodeStatus::FAILURE;
        }

        double dis = getDistance(current_pose.position, goal_pose.pose.pose.position);
        if(dis > 0.15) {
            future_goal_handle = nav2_client->async_send_goal(goal_pose);
            RCLCPP_INFO(logger, "Running. Distance: %fm", dis);
            return BT::NodeStatus::RUNNING;
        } else {
            RCLCPP_INFO(logger, "Navigation to target successful.");
            return BT::NodeStatus::SUCCESS;
        }
    }

    /**
     * @brief 在行为被中止时调用，取消导航
     */
    void onHalted() override {
        RCLCPP_WARN(logger, "Navigation to target was halted.");
        nav2_client->async_cancel_all_goals();
    }

    /**
     * @brief   获取机器人的当前位姿
     * 
     * @details 1.使用 tf2_ros::Buffer 获取 map 到 base_link 的变换，这代表机器人在地图坐标系中的位置。
     *          2.如果成功，更新 current_pose 为机器人当前的位姿，并返回 true。
     *          3.如果变换失败，捕获异常并返回 false，同时输出错误信息。
     * 
     * @return bool 
     */
    bool getCurrentPose() {
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = \
                tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);

            current_pose.position.x = transform_stamped.transform.translation.x;
            current_pose.position.y = transform_stamped.transform.translation.y;
            current_pose.position.z = transform_stamped.transform.translation.z;
            current_pose.orientation = transform_stamped.transform.rotation;

            RCLCPP_INFO(logger, "Current position: %f, %f", current_pose.position.x, current_pose.position.y);
            return true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(logger, "Could not transform: %s", ex.what());
            return false;
        }
    }

    /**
     * @brief  计算当前位置与目标位置之间的欧氏距离
     * 
     * @param  point 当前位姿的坐标点
     * @param  goal  目标位姿的坐标点
     * 
     * @return double 返回 point 和 goal 的距离
     */
    double getDistance(const geometry_msgs::msg::Point &point, const geometry_msgs::msg::Point &goal) {
        double dx = std::abs(point.x - goal.x);
        double dy = std::abs(point.y - goal.y);
        return std::sqrt(dx * dx + dy * dy);
    }

  private:
    // ROS
    rclcpp::Node::SharedPtr client_node;
    rclcpp::Logger logger;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client;
    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future_goal_handle;
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    // Pose
    geometry_msgs::msg::Pose current_pose;
    nav2_msgs::action::NavigateToPose::Goal goal_pose;
}; // class MoveActionNode
