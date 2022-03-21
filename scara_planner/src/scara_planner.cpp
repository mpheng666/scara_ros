#include "scara_planner.hpp"

namespace scara_ns
{
    ScaraPlanner::ScaraPlanner() : spinner(4),
                                   planner_mode(Mode::cartesian_mode),
                                   visual_tools("panda_link0"),
                                   move_group(PLANNING_GROUP),
                                   traj_list_pub_(private_nh_.advertise<scara_planner::TrajectoryJoints>("trajectory_joints", 10)),
                                   traj_goal_pub_(private_nh_.advertise<geometry_msgs::Point>("trajectory_goal", 10)),
                                   current_pose_pub_(private_nh_.advertise<geometry_msgs::Point>("current_pose", 10)),
                                   current_joint_sub_(private_nh_.subscribe("current_joints", 20, &ScaraPlanner::joints_callback, this)),
                                   joy_sub_(private_nh_.subscribe<sensor_msgs::Joy>("joy", 100, &ScaraPlanner::joyCb, this)),
                                   execution_completion_flag_(true)
    {
        ROS_INFO("scara_node initiated!");
    }

    ScaraPlanner::~ScaraPlanner()
    {
        ;
    }

    void ScaraPlanner::loadParam()
    {
        ROS_INFO("Load ROS param");
        // ros::param::param<double>("~link_1", _link_1_length, 0.4f);
    }

    void ScaraPlanner::startProcess()
    {
        ros::Rate r(LOOP_RATE);
        this->loadParam();
        auto jmg = this->startMoveGroupInterface();
        this->startVisualization();
        target_pose.position.x = 0.28;
        target_pose.position.y = -0.2;
        target_pose.position.z = 0.5;
        // target_pose.position.x = 0.46;
        // target_pose.position.y = 0.0;
        // target_pose.position.z = 0.225;
        target_pose.orientation.w = 0.0;
        target_pose.orientation.w = 0.0;
        target_pose.orientation.w = 0.0;
        target_pose.orientation.w = 1.0;

        while(private_nh_.ok())
        {
            spinner.start();
            this->getRobotState();
            // this->getCurrentPose();
            switch (planner_mode)
            {
                case Mode::cartesian_mode:
                {
                    ROS_INFO("Planner is cartesian mode");
                    current_pose = this->getCurrentPose();
                    // auto target = this->getPoseTarget();
                    this->setPoseTarget(target_pose);
                    this->plan = this->getPosePlan();
                    this->visualizePlan(target_pose, plan, jmg);
                    break;
                }
                case Mode::joint_mode:
                {
                    ROS_INFO("Planner is joint mode");
                    this->getJointTarget(jmg);
                    this->plan = this->setJointTarget(jmg);
                    break;
                }
                case Mode::execution_mode:
                {
                    ROS_INFO("Planner is execution mode");
                    move_group.move();
                    break;
                }
                default:
                    break;

            }
            
        }
        // ros::waitForShutdown();
    }

    void ScaraPlanner::joints_callback(const scara_planner::TrajectoryJoints &msg)
    {
    }

    void ScaraPlanner::joyCb(const sensor_msgs::Joy::ConstPtr &msg)
    {
        if (msg->buttons[0] == 1)
        {
            planner_mode = Mode::cartesian_mode;
            target_pose.position.x = (msg->axes[0]*0.25 + current_pose.position.x);
            target_pose.position.y = (msg->axes[1]*0.25 + current_pose.position.y);
            target_pose.position.z = (msg->axes[2]*0.25 + current_pose.position.z);
        }
        else if (msg->buttons[1] == 1)
        {
            planner_mode = Mode::joint_mode;
            joint_group_positions[0] = msg->axes[1];
            joint_group_positions[1] = msg->axes[3];
            // joint_group_positions[2] = msg->axes[2];
            // joy_joints(0) = msg->axes[0];
            // joy_joints(1) = msg->axes[1];
            // joy_joints(2) = msg->axes[2];
            // joy_joints(3) = msg->axes[3];
        }
        else if (msg->buttons[2] == 1)
        {
            planner_mode = Mode::execution_mode;

        }
    }

    const robot_state::JointModelGroup* ScaraPlanner::startMoveGroupInterface()
    {
        spinner.start();

        const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());

        ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

        ROS_INFO("Available Planning Groups:");
        std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
                  std::ostream_iterator<std::string>(std::cout, "\n"));

        return joint_model_group;
    }

    void ScaraPlanner::startVisualization()
    {
        visual_tools.deleteAllMarkers();
        visual_tools.loadRemoteControl();

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.75;
        visual_tools.publishText(text_pose, "Scara MoveGroupInterface", rvt::WHITE, rvt::XLARGE);
        visual_tools.trigger();
    }

    void ScaraPlanner::visualizePlan(const geometry_msgs::Pose &target_pose, const moveit::planning_interface::MoveGroupInterface::Plan &plan, const robot_state::JointModelGroup *joint_model_group)
    {
        visual_tools.deleteAllMarkers();

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();

        ROS_INFO("Visualizing plan as trajectory line");
        visual_tools.publishAxisLabeled(target_pose, "pose1");

        visual_tools.publishText(text_pose, "Pose Goal", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
        visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);

        visual_tools.trigger();
        // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    }

    void ScaraPlanner::getCurrentJoint()
    {
        std::vector<double> current_joint = move_group.getCurrentJointValues();
        // for(int i = 0; i < current_joint.size(); ++i)
        // {
        //     ROS_INFO("Current joint %i : %f", i, current_joint[i]);
        // }
    }

    void ScaraPlanner::getJointTarget(const robot_state::JointModelGroup *joint_model_group)
    {
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    }

    const moveit::planning_interface::MoveGroupInterface::Plan ScaraPlanner::setJointTarget(const robot_state::JointModelGroup *joint_model_group)
    {
        moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
        // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        for(int i = 0; i < joint_group_positions.size(); ++i)
        {
            ROS_INFO("Joint %i target: %f", i, joint_group_positions[i]);
        }
        move_group.setJointValueTarget(joint_group_positions);

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.75;
        bool success = (move_group.plan(joint_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("joint_plan", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
        
        visual_tools.deleteAllMarkers();
        visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(joint_plan.trajectory_, joint_model_group);
        visual_tools.trigger();
        // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

        return joint_plan;
    }

    geometry_msgs::Pose ScaraPlanner::getCurrentPose()
    {
        geometry_msgs::PoseStamped current_pose_w_stamped = move_group.getCurrentPose();
        geometry_msgs::Pose current_pose = current_pose_w_stamped.pose;
        // ROS_INFO("Current pos x: %f", current_pose.position.x);
        // ROS_INFO("Current pos y: %f", current_pose.position.y);
        // ROS_INFO("Current pos z: %f", current_pose.position.z);
        // ROS_INFO("Current orientation x: %f", current_pose.orientation.x);
        // ROS_INFO("Current orientation y: %f", current_pose.orientation.y);
        // ROS_INFO("Current orientation z: %f", current_pose.orientation.z);
        // ROS_INFO("Current orientation w: %f", current_pose.orientation.w);

        return current_pose;
    }

    geometry_msgs::Pose ScaraPlanner::getPoseTarget()
    {

        // target_pose.position.x = 0.28;
        // target_pose.position.y = -0.2;
        // target_pose.position.z = 0.5;
        // // target_pose.position.x = 0.46;
        // // target_pose.position.y = 0.0;
        // // target_pose.position.z = 0.225;
        // target_pose.orientation.w = 0.0;
        // target_pose.orientation.w = 0.0;
        // target_pose.orientation.w = 0.0;
        // target_pose.orientation.w = 1.0;
        ROS_INFO("Target pos x: %f", target_pose.position.x);
        ROS_INFO("Target pos y: %f", target_pose.position.y);
        ROS_INFO("Target pos z: %f", target_pose.position.z);
        ROS_INFO("Target orientation x: %f", target_pose.orientation.x);
        ROS_INFO("Target orientation y: %f", target_pose.orientation.y);
        ROS_INFO("Target orientation z: %f", target_pose.orientation.z);
        ROS_INFO("Target orientation w: %f", target_pose.orientation.w);

        return target_pose;
    }

    void ScaraPlanner::setPoseTarget(const geometry_msgs::Pose &target_pose)
    {
        move_group.setPoseTarget(target_pose);
        ROS_INFO("Pose goal target is set");
        // ROS_INFO("Set target pos x: %f", target_pose.position.x);
        // ROS_INFO("Set target pos y: %f", target_pose.position.y);
        // ROS_INFO("Set target pos z: %f", target_pose.position.z);
        // ROS_INFO("Set target orientation x: %f", target_pose.orientation.x);
        // ROS_INFO("Set target orientation y: %f", target_pose.orientation.y);
        // ROS_INFO("Set target orientation z: %f", target_pose.orientation.z);
        // ROS_INFO("Set target orientation w: %f", target_pose.orientation.w);
    }

    void ScaraPlanner::getGoalPosTolerance()
    {
        double position_tolerance = move_group.getGoalPositionTolerance();
        ROS_INFO("Postion tolerance: %f", position_tolerance);
    }

    void ScaraPlanner::setGoalPosTolerance(const double &position_tolerance)
    {
        move_group.setGoalPositionTolerance(position_tolerance);
        ROS_INFO("Set postion tolerance: %f", position_tolerance);
    }

    void ScaraPlanner::getGoalOrientTolerance()
    {
        double orientation_tolerance = move_group.getGoalOrientationTolerance();
        ROS_INFO("Orientation tolerance: %f", orientation_tolerance);
    }

    void ScaraPlanner::setGoalOrientTolerance(const double &orientation_tolerance)
    {
        move_group.setGoalOrientationTolerance(orientation_tolerance);
        ROS_INFO("Set orientation tolerance: %f", orientation_tolerance);
    }

    moveit::planning_interface::MoveGroupInterface::Plan ScaraPlanner::getPosePlan()
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        return plan;
    }

    void ScaraPlanner::getRobotState()
    {
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    }

    void ScaraPlanner::moveGoalTarget()
    {
        move_group.move();
    }

    // moveit::planning_interface::MoveItErrorCode ScaraPlanner::executePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan)
    // {
    //     return move_group.execute(plan);
    // }
} // scara_ns