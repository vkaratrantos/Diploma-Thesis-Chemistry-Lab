// =============================================================================
// simple_move.cpp — Liquid Handler Pick & Place
// =============================================================================

#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <vector>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

// CONSTANTS

static constexpr double VEL_SCALE_TRANSIT  = 0.10; 
static constexpr double VEL_SCALE_LIQUID   = 0.05; 
static constexpr double ACC_SCALE_TRANSIT  = 0.10;
static constexpr double ACC_SCALE_LIQUID   = 0.05;
static constexpr double CARTESIAN_EEF_STEP = 0.005; 
static constexpr double CARTESIAN_JUMP_THR = 2.0;   
static constexpr double MIN_CARTESIAN_FRACTION = 0.95; 

// Fixed Standard Lift Height for 7-DOF myArm 300 Pi
static constexpr double SAFE_Z_TARGET      = 0.28;  

// Drop Retry Constants (Kept for Phase 3 safety)
static constexpr double DROP_Z_RETRY_STEP  = 0.001;  
static constexpr double DROP_Z_MAX_RETRY   = 0.05;  

static constexpr int    STATE_SETTLE_MS    = 500;   

// HELPER: wait for robot state to settle after an execute() call
void waitForStateSettle(
    moveit::planning_interface::MoveGroupInterface & iface,
    int ms = STATE_SETTLE_MS)
{
    rclcpp::sleep_for(std::chrono::milliseconds(ms));
    iface.startStateMonitor(1.0); 
}

// HELPER: Strict Cartesian move for LIFT/DROP. NO OMPL FALLBACK.
bool strictCartesianMove(
    moveit::planning_interface::MoveGroupInterface & iface,
    const geometry_msgs::msg::Pose                 & target,
    const std::string                              & phase_name)
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

// HELPER: find IK-valid overhead pose by scanning YAW angles (Z-axis rotation)
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

// HELPER: Attempt a horizontal move keeping the tube upright (LIN -> OMPL)
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
    
    iface.clearPathConstraints();
    return success;
}

// HELPER: 2-Stage Smart Drop (Approach & Insert)
bool smartVerticalDrop(
    moveit::planning_interface::MoveGroupInterface & iface,
    double target_z,
    const tf2::Quaternion & q_upright)
{
    geometry_msgs::msg::Pose start_pose = iface.getCurrentPose().pose;
    geometry_msgs::msg::Pose final_pose = start_pose;
    final_pose.position.z = target_z;

    // 1. Attempt Pure Cartesian (Best Case scenario)
    if (strictCartesianMove(iface, final_pose, "DROP_DIRECT")) {
        return true;
    }

    std::cout << "    [-] Direct drop blocked. Using 2-Stage Approach & Insert...\n";

    // 2. Calculate Approach Pose (4 cm above target)
    double approach_z = target_z + 0.04; 
    if (start_pose.position.z <= approach_z) {
        std::cout << "    [-] Arm is already too low for a 2-stage drop.\n";
        return false;
    }

    geometry_msgs::msg::Pose approach_pose = start_pose;
    approach_pose.position.z = approach_z;

    // --- STAGE A: Constrained OMPL Descent ---
    iface.setPlanningPipelineId("ompl");
    iface.setPlannerId("RRTConnectkConfigDefault");
    iface.setPlanningTime(10.0);
    iface.setPoseTarget(approach_pose);

    // Keep the tube upright, but allow the arm to figure out the easiest joint path
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

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool approach_success = false;
    
    if (iface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        if (iface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            waitForStateSettle(iface);
            approach_success = true;
        }
    }
    iface.clearPathConstraints();

    if (!approach_success) {
        std::cout << "    [-] Approach phase failed.\n";
        return false;
    }

    // --- STAGE B: Strict Cartesian Insertion ---
    std::cout << "    [+] Approach reached. Executing final vertical insertion...\n";
    
    // Sync the state and plunge the final 4cm perfectly straight
    iface.setStartStateToCurrentState();
    return strictCartesianMove(iface, final_pose, "DROP_INSERT");
}

// MAIN
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("liquid_handler_master");

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    std::thread spinner([executor]() { executor->spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface arm_interface(node, "arm_group");
    MoveGroupInterface gripper_interface(node, "gripper");

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
        std::cout << "\n[INPUT] Enter <X Y Z>, 'o' (Open), 'c' (Closed), 'p' (Pour) or 'q' (Quit): ";
        std::string line;
        std::getline(std::cin, line);

        if (line == "q" || line == "Q") break;

        // 1. Check for Gripper Commands
        if (line == "o" || line == "O") {
            std::cout << ">>> Opening Gripper...\n";
            gripper_interface.setNamedTarget("open");
            gripper_interface.move(); 
            continue; 
        }
    
        if (line == "c" || line == "C") {
            std::cout << ">>> Closing Gripper...\n";
            gripper_interface.setNamedTarget("closed");
            gripper_interface.move();
            continue;
        }
        

        if (line == "c" || line == "C") {
            std::cout << ">>> Closing Gripper...\n";
            gripper_interface.setNamedTarget("closed");
            gripper_interface.move();
            continue;
        }

        if (line == "p" || line == "P") {
            std::cout << ">>> Pouring the liquid...\n";
            
            // 1. Get current joint positions
            std::vector<double> joint_positions;
            moveit::core::RobotStatePtr current_state = arm_interface.getCurrentState();
            const moveit::core::JointModelGroup* joint_model_group = 
                current_state->getJointModelGroup("arm_group");
            current_state->copyJointGroupPositions(joint_model_group, joint_positions);

            if (!joint_positions.empty()) {
                double original_last_joint = joint_positions.back();
                
                // 2. Add 90 degrees (pi/2) to the last joint
                joint_positions.back() = original_last_joint + (M_PI / 2.0);
                arm_interface.setJointValueTarget(joint_positions);
                
                // 3. Execute the pour
                if (arm_interface.move() == moveit::core::MoveItErrorCode::SUCCESS) {
                    waitForStateSettle(arm_interface);
                    std::cout << "    [+] Poured. Waiting 1 second...\n";
                    
                    // Wait for liquid to drain
                    rclcpp::sleep_for(std::chrono::seconds(1));

                    // 4. Return to the upright state
                    std::cout << "    [+] Returning to upright...\n";
                    joint_positions.back() = original_last_joint;
                    arm_interface.setJointValueTarget(joint_positions);
                    arm_interface.move();
                    waitForStateSettle(arm_interface);
                    
                    std::cout << ">>> Pour sequence complete!\n";
                } else {
                    std::cout << "    [-] Could not execute pour. Joint limit reached?\n";
                }
            }
            continue;
        }
        // ---> END POUR COMMAND <---
        // 2. Parse as Coordinates
        std::stringstream ss(line);
        double tx, ty, tz;
        if (!(ss >> tx >> ty >> tz)) {
            std::cout << "[-] Invalid input. Use 'o', 'c', or three numbers.\n";
            continue;
        }

        // PHASE 1: LIFT  — straight up to fixed SAFE_Z_TARGET
        std::cout << "\n--- [PHASE 1] LIFT to Standard Z=" << SAFE_Z_TARGET << " ---\n";

        arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
        arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

        geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
        geometry_msgs::msg::Pose lift_pose    = current_pose;
        lift_pose.position.z = SAFE_Z_TARGET;

        if (strictCartesianMove(arm_interface, lift_pose, "LIFT")) {
            waitForStateSettle(arm_interface);
        } else {
            std::cout << "[-] [PHASE 1] Cannot reach standard lift height Z=" << SAFE_Z_TARGET 
                      << ". Skipping this target.\n";
            continue;
        }

        // --- [PHASE 2] OPTIMIZED HORIZONTAL MOVE ---
        std::cout << "\n--- [PHASE 2] HORIZONTAL MOVE to (" << tx << ", " << ty << ") ---\n";

        arm_interface.setStartStateToCurrentState();
        geometry_msgs::msg::Pose start_pose = arm_interface.getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.x = tx;
        target_pose.position.y = ty;

        bool moved = false;

        // Try DIRECT path first
        if (strictCartesianMove(arm_interface, target_pose, "HORIZ_DIRECT")) {
            waitForStateSettle(arm_interface);
            moved = true;
        } else {
            std::cout << "    [-] Direct path blocked. Running Virtual Look-Ahead for detours...\n";

            // Geometry for Detour Calculation
            double dx = tx - start_pose.position.x;
            double dy = ty - start_pose.position.y;
            double mid_x = start_pose.position.x + (dx / 2.0);
            double mid_y = start_pose.position.y + (dy / 2.0);
            double perp_x = -dy; 
            double perp_y = dx;
            double mag = std::sqrt(perp_x * perp_x + perp_y * perp_y);
            perp_x /= mag; perp_y /= mag;

            // Broader range of offsets for 7-DOF flexibility
            std::vector<double> offsets = {0.01, -0.01, 0.02, -0.02, 0.03, -0.03, 0.04, -0.04, 0.05, -0.05, 0.06, -0.06, 0.07, -0.07, 0.08, -0.08, 0.09, -0.09, 0.1, -0.1, 0.11, -0.11, 0.12, -0.12, 0.13, -0.13, 0.14, -0.14, 0.15, -0.15, 0.16, -0.16, 0.17, -0.17, 0.18, -0.18, 0.19, -0.19, 0.2, -0.2, 0.21, -0.21, 0.22, -0.22, 0.23, -0.23};

            for (double offset : offsets) {
                geometry_msgs::msg::Pose detour_pose = start_pose;
                detour_pose.position.x = mid_x + (perp_x * offset);
                detour_pose.position.y = mid_y + (perp_y * offset);

                // VIRTUAL CHECK LEG 1 (A -> C)
                moveit_msgs::msg::RobotTrajectory traj1;
                std::vector<geometry_msgs::msg::Pose> way1 = {detour_pose};
                double frac1 = arm_interface.computeCartesianPath(way1, CARTESIAN_EEF_STEP, CARTESIAN_JUMP_THR, traj1);

                if (frac1 >= MIN_CARTESIAN_FRACTION) {
                    // VIRTUAL CHECK LEG 2 (C -> B)
                    // We simulate the robot state at the end of Leg 1
                    moveit::core::RobotState temp_state(*arm_interface.getCurrentState());
                    temp_state.setJointGroupPositions("arm_group", traj1.joint_trajectory.points.back().positions);
                    
                    arm_interface.setStartState(temp_state); 
                    moveit_msgs::msg::RobotTrajectory traj2;
                    std::vector<geometry_msgs::msg::Pose> way2 = {target_pose};
                    double frac2 = arm_interface.computeCartesianPath(way2, CARTESIAN_EEF_STEP, CARTESIAN_JUMP_THR, traj2);

                    if (frac2 >= MIN_CARTESIAN_FRACTION) {
                        std::cout << "    [+] Valid Detour Found (Offset: " << offset*100 << "cm). Executing Leg 1...\n";
                        
                        // Execute Leg 1
                        arm_interface.setStartStateToCurrentState(); // Reset to reality for execution
                        arm_interface.execute(traj1);
                        waitForStateSettle(arm_interface);

                        std::cout << "    [+] Executing Leg 2...\n";
                        arm_interface.execute(traj2);
                        waitForStateSettle(arm_interface);
                        
                        moved = true;
                        break;
                    }
                }
                // Reset start state for next iteration trial
                arm_interface.setStartStateToCurrentState();
            }
        }

        if (!moved) {
            std::cout << "[-] [PHASE 2] All detour attempts failed. Target unreachable upright.\n";
            continue;
        }

        // PHASE 3: DROP  — straight down to target Z
        std::cout << "\n--- [PHASE 3] DROP to Z=" << tz << " ---\n";

        arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
        arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

        bool dropped = false;

        for (double z_try = tz;
             z_try <= tz + DROP_Z_MAX_RETRY && !dropped;
             z_try += DROP_Z_RETRY_STEP)
        {
            // Use our new 2-stage drop function instead of basic Cartesian
            if (smartVerticalDrop(arm_interface, z_try, q_upright)) {
                dropped = true;
                waitForStateSettle(arm_interface);
                
                if (z_try == tz) {
                    std::cout << "\n>>> Sequence complete! Target reached safely at exact Z=" << z_try << ".\n";
                } else {
                    std::cout << "\n>>> Sequence complete! Target reached safely, but slightly higher at Z=" << z_try << ".\n";
                }
            } else {
                std::cout << "    Retrying DROP at a higher Z=" << (z_try + DROP_Z_RETRY_STEP) << "...\n";
            }
        }

        if (!dropped) {
            std::cout << "[-] [PHASE 3] Cannot drop securely. Workspace obstructed or kinematic limits reached.\n";
        }
    }

    std::cout << "\n>>> Shutting down. Goodbye!\n";
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
