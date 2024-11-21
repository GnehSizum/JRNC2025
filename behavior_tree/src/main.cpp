#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include "move_action.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    BT::BehaviorTreeFactory factory;

    auto node = rclcpp::Node::make_shared("patrol_node");
    // 注册 Move 行为节点
    factory.registerNodeType<MoveActionNode>("MoveActionNode");
    // 从 XML 文件创建行为树
    auto tree = factory.createTreeFromFile("/home/muzs/JRNC_SIM/src/behavior_tree/tree/patrol.xml");

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        // 每次循环 tick 树的根节点
        tree.tickRoot();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
