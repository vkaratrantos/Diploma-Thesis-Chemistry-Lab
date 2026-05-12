// =============================================================================
// simple_move.cpp — Liquid Handler Pick & Place
// ROS 2 Humble | MoveIt 2 | Pilz Industrial Motion Planner
//
// Architecture: LIFT (Cartesian) → MOVE (Pilz/OMPL via points) → DROP (Cartesian)
// =============================================================================

// --- Standard C++ Libraries ---
#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <vector>
#include <chrono>

// --- ROS 2 & MoveIt 2 Libraries ---
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

// =============================================================================
// CONSTANTS
// =============================================================================
static constexpr double VEL_SCALE_TRANSIT  = 0.10; 
static constexpr double VEL_SCALE_LIQUID   = 0.05; 
static constexpr double ACC_SCALE_TRANSIT  = 0.10;
static constexpr double ACC_SCALE_LIQUID   = 0.05;

static constexpr double CARTESIAN_EEF_STEP = 0.005; 
static constexpr double CARTESIAN_JUMP_THR = 0.0;   

static constexpr double MIN_CARTESIAN_FRACTION = 0.95; 

static constexpr double SAFE_Z_DEFAULT     = 0.3;  
static constexpr double SAFE_Z_MARGIN      = 0.12;  
static constexpr double SAFE_Z_RETRY_STEP  = 0.03;  
static constexpr double SAFE_Z_MAX_RETRY   = 0.15;  

static constexpr int    STATE_SETTLE_MS    = 500;   

// =============================================================================
// HELPER: wait for robot state to settle after an execute() call
// =============================================================================
void waitForStateSettle(
    moveit::planning_interface::MoveGroupInterface & iface,
    int ms = STATE_SETTLE_MS)
{
    rclcpp::sleep_for(std::chrono::milliseconds(ms));
    iface.startStateMonitor(1.0); 
}

// =============================================================================
// HELPER: Strict Cartesian move for LIFT/DROP. NO OMPL FALLBACK.
// =============================================================================
bool strictCartesianMove(
    moveit::planning_interface::MoveGroupInterface & iface,
    const geometry_msgs::msg::Pose                & target,
    const std::string                             & phase_name)
{
    std::vector<geometry_msgs::msg::Pose> waypoints = {target};
    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = iface.computeCartesianPath(
        waypoints, CARTESIAN_EEF_STEP, CARTESIAN_JUMP_THR, trajectory);

    if (fraction >= MIN_CARTESIAN_FRACTION) {
        std::cout << "    [" << phase_name << "] Cartesian path: "
                  << static_cast<int>(fraction * 100) << "% complete. Executing...\n";
        auto result = iface.execute(trajectory);
        return (result == moveit::core::MoveItErrorCode::SUCCESS);
    } 
    
    std::cout << "[-] [" << phase_name << "] Cartesian coverage too low ("
              << static_cast<int>(fraction * 100) << "%). Aborting to prevent spilling.\n";
    return false;
}

// =============================================================================
// HELPER: find IK-valid overhead pose by scanning YAW angles (Z-axis rotation)
// =============================================================================
bool findValidOverheadPose(
    moveit::planning_interface::MoveGroupInterface & iface,
    double tx, double ty, double safe_z,
    const tf2::Quaternion                          & q_upright,
    geometry_msgs::msg::Pose                       & out_pose)
{
    moveit::core::RobotStatePtr kinematic_state = iface.getCurrentState();
    const moveit::core::JointModelGroup * jmg =
        kinematic_state->getJointModelGroup("arm_group");

    std::vector<int> yaw_angles = {0};
    for (int i = 1; i <= 180; i += 5) {
        yaw_angles.push_back(i);
        yaw_angles.push_back(-i);
    }

    for (int angle : yaw_angles) {
        double yaw = angle * (M_PI / 180.0);
        tf2::Quaternion q_rot;
        q_rot.setRotation(tf2::Vector3(0, 0, 1), yaw); 
        tf2::Quaternion q_final = (q_upright * q_rot).normalized();

        geometry_msgs::msg::Pose test_pose;
        test_pose.position.x    = tx;
        test_pose.position.y    = ty;
        test_pose.position.z    = safe_z;
        test_pose.orientation.x = q_final.x();
        test_pose.orientation.y = q_final.y();
        test_pose.orientation.z = q_final.z();
        test_pose.orientation.w = q_final.w();

        if (kinematic_state->setFromIK(jmg, test_pose, 0.05)) {
            out_pose = test_pose;
            return true;
        }
    }
    return false;
}

// =============================================================================
// HELPER: Attempt a horizontal move keeping the tube upright (LIN -> OMPL)
// =============================================================================
bool horizontalTransitUpright(
    moveit::planning_interface::MoveGroupInterface & iface,
    const geometry_msgs::msg::Pose                 & target_pose,
    const tf2::Quaternion                          & q_upright)
{
    // 1. Try Pilz LIN first
    iface.setPlanningPipelineId("pilz_industrial_motion_planner");
    iface.setPlannerId("LIN");
    iface.setPlanningTime(15.0);
    iface.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (iface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        if (iface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            waitForStateSettle(iface);
            return true;
        }
    }
    
    // 2. LIN failed, fallback to Constrained OMPL
    iface.setPlanningPipelineId("ompl");
    iface.setPlannerId("RRTConnectkConfigDefault");
    iface.setPlanningTime(15.0);
    iface.setPoseTarget(target_pose);

    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = iface.getEndEffectorLink();
    ocm.header.frame_id = iface.getPlanningFrame();
    ocm.orientation.x = q_upright.x();
    ocm.orientation.y = q_upright.y();
    ocm.orientation.z = q_upright.z();
    ocm.orientation.w = q_upright.w();
    ocm.absolute_x_axis_tolerance = 0.15; 
    ocm.absolute_y_axis_tolerance = 0.15;
    ocm.absolute_z_axis_tolerance = 3.14; 
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    iface.setPathConstraints(path_constraints);

    bool success = false;
    if (iface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        if (iface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            waitForStateSettle(iface);
            success = true;
        }
    }
    
    iface.clearPathConstraints(); // Always clean up!
    return success;
}


// =============================================================================
// MAIN
// =============================================================================
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("liquid_handler_master");

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    std::thread spinner([executor]() { executor->spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface arm_interface(node, "arm_group");

    tf2::Quaternion q_upright;
    q_upright.setRPY(0.0, -M_PI / 2.0, M_PI / 2.0);

    std::cout << "\n>>> [INIT] Moving to start position with OMPL...\n";

    arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_TRANSIT);
    arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_TRANSIT);
    arm_interface.setPlanningPipelineId("ompl");
    arm_interface.setPlannerId("RRTConnectkConfigDefault");
    arm_interface.setPlanningTime(5.0);
    arm_interface.setGoalPositionTolerance(0.01);
    arm_interface.setGoalOrientationTolerance(0.05);

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x    = -0.15;
    start_pose.position.y    = -0.15;
    start_pose.position.z    =  0.20; 
    start_pose.orientation.x = q_upright.x();
    start_pose.orientation.y = q_upright.y();
    start_pose.orientation.z = q_upright.z();
    start_pose.orientation.w = q_upright.w();

    arm_interface.setPoseTarget(start_pose);
    MoveGroupInterface::Plan initial_plan;

    if (arm_interface.plan(initial_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        std::cout << ">>> [INIT] Plan found. Executing...\n";
        arm_interface.execute(initial_plan);
        waitForStateSettle(arm_interface);
    } else {
        std::cout << "[-] [INIT] Failed to reach start position!\n";
        rclcpp::shutdown();
        spinner.join();
        return -1;
    }

    arm_interface.setGoalPositionTolerance(0.001);
    arm_interface.setGoalOrientationTolerance(0.001);

    std::cout << "\n>>> Ready. Enter targets as: X Y Z   (or 'q' to quit)\n";

    while (rclcpp::ok()) {
        std::cout << "\nEnter Final Target <X Y Z>: ";
        std::string line;
        std::getline(std::cin, line);

        if (line == "q" || line == "Q") break;

        std::stringstream ss(line);
        double tx, ty, tz;
        if (!(ss >> tx >> ty >> tz)) {
            std::cout << "[-] Invalid input. Please enter three numbers.\n";
            continue;
        }

        double safe_z = SAFE_Z_DEFAULT;
        if (tz + SAFE_Z_MARGIN > safe_z) {
            safe_z = tz + SAFE_Z_MARGIN;
        }

        // =================================================================
        // PHASE 1: LIFT  — straight up to safe_z (Strict Cartesian)
        // =================================================================
        std::cout << "\n--- [PHASE 1] LIFT to Z=" << safe_z << " ---\n";

        arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
        arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

        geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
        geometry_msgs::msg::Pose lift_pose    = current_pose;

        bool lifted = false;

        for (double z_try = safe_z;
             z_try <= safe_z + SAFE_Z_MAX_RETRY && !lifted;
             z_try += SAFE_Z_RETRY_STEP)
        {
            lift_pose.position.z = z_try;

            if (strictCartesianMove(arm_interface, lift_pose, "LIFT")) {
                safe_z = z_try; 
                lifted = true;
                waitForStateSettle(arm_interface);
            } else {
                std::cout << "    Retrying with Z=" << (z_try + SAFE_Z_RETRY_STEP) << "...\n";
            }
        }

        if (!lifted) {
            std::cout << "[-] [PHASE 1] Cannot lift securely. Skipping this target.\n";
            continue;
        }

        // =================================================================
        // PHASE 2: MOVE  — horizontal transfer at safe_z using strict Cartesian
        // =================================================================
        std::cout << "\n--- [PHASE 2] HORIZONTAL MOVE to (" << tx << ", " << ty << ") ---\n";

        bool moved = false; // <--- ADD THIS LINE RIGHT HERE
        
        // 1. Force hardware state sync
        arm_interface.setStartStateToCurrentState();
        rclcpp::sleep_for(std::chrono::milliseconds(200));

        arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
        arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

        // 2. Get current pose (already upright and at safe_z from Phase 1)
        geometry_msgs::msg::Pose horizontal_pose = arm_interface.getCurrentPose().pose;
        
        // 3. Just change the X and Y
        horizontal_pose.position.x = tx;
        horizontal_pose.position.y = ty;
        
        // 4. Force orientation to be perfectly upright (overriding mechanical sag)
        horizontal_pose.orientation.x = q_upright.x();
        horizontal_pose.orientation.y = q_upright.y();
        horizontal_pose.orientation.z = q_upright.z();
        horizontal_pose.orientation.w = q_upright.w();

        // 5. Execute using your existing Cartesian function
        if (strictCartesianMove(arm_interface, horizontal_pose, "HORIZONTAL_MOVE")) {
            waitForStateSettle(arm_interface);
            moved = true;
        } else {
            std::cout << "[-] [PHASE 2] Cartesian horizontal move failed. Target may be out of reach.\n";
            moved = false;
        }

        if (!moved) {
            std::cout << "[-] [PHASE 2] Skipping to next target.\n";
            continue;
        }

        // =================================================================
        // PHASE 3: DROP  — straight down to target Z (Strict Cartesian)
        // =================================================================
        std::cout << "\n--- [PHASE 3] DROP to Z=" << tz << " ---\n";

        arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
        arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

        geometry_msgs::msg::Pose drop_pose = arm_interface.getCurrentPose().pose;
        drop_pose.position.z = tz;

        if (strictCartesianMove(arm_interface, drop_pose, "DROP")) {
            waitForStateSettle(arm_interface);
            std::cout << "\n>>> Sequence complete! Target reached safely.\n";
        } else {
            std::cout << "[-] [PHASE 3] Cannot drop securely. Workspace obstructed.\n";
        }
    }

    std::cout << "\n>>> Shutting down. Goodbye!\n";
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
