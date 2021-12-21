// 包含miveit的API头文件
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/OrientationConstraint.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_dance_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(10);
    moveit::planning_interface::MoveGroupInterface group("manipulator_i5");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success;

    while(ros::ok())
    {
        group.setNamedTarget("zero");

        group.setMaxVelocityScalingFactor(0.1);
        group.setMaxAccelerationScalingFactor(1);
        group.setGoalPositionTolerance(0.001);
        group.setGoalOrientationTolerance(0.01);
        success = group.plan(my_plan);
        ROS_INFO("Visualizing plan 0 (pose goal) %s",success?"":"FAILED");

        //让机械臂按照规划的轨迹开始运动。
        if(success)
            group.execute(my_plan);
        group.setNamedTarget("home");
        success = group.plan(my_plan);
        if(success)
            group.asyncExecute(my_plan);
        sleep(2);
        group.stop();
        usleep(800000);
        group.setNamedTarget("home");
        success = group.plan(my_plan);
        if(success)
            group.execute(my_plan);
        loop_rate.sleep();
        ros::spinOnce();
    }
    ros::shutdown();
    return 0;
}

