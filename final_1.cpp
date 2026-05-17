// =============================================================================
// simple_move.cpp — Liquid Handler Pick & Place (ROS 2 Topic Version)
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
#include <std_msgs/msg/string.hpp> // <-- NEW: For ROS 2 Topic Communication
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// CONSTANTS

static constexpr double VEL_SCALE_TRANSIT  = 0.9; 
static constexpr double VEL_SCALE_LIQUID   = 0.8; 
static constexpr double ACC_SCALE_TRANSIT  = 0.80;
static constexpr double ACC_SCALE_LIQUID   = 0.60;
static constexpr double CARTESIAN_EEF_STEP = 0.005; 
static constexpr double CARTESIAN_JUMP_THR = 2.0;   
static constexpr double MIN_CARTESIAN_FRACTION = 0.95; 

// Fixed Standard Lift Height for 7-DOF myArm 300 Pi

static constexpr double SAFE_Z_TARGET      = 0.28;  

// Drop Retry Constants (Kept for Phase 3 safety)
static constexpr double DROP_Z_RETRY_STEP  = 0.001;  
static constexpr double DROP_Z_MAX_RETRY   = 0.08;  
static constexpr int    STATE_SETTLE_MS    = 500;   

// --- NEW HELPER: Setup Environment Collisions ---

void setupCollisionObjects(const std::string& frame_id) {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // 1. Table/Base Collision

    moveit_msgs::msg::CollisionObject table;
    table.id = "table_base";
    table.header.frame_id = frame_id;
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {1.0, 1.0, 0.02}; // L x W x H
    table.primitive_poses.resize(1);
    table.primitive_poses[0].position.x = 0.0;
    table.primitive_poses[0].position.y = 0.0;
    table.primitive_poses[0].position.z = -0.02; 
    table.primitive_poses[0].orientation.w = 1.0;
    table.operation = table.ADD;
    collision_objects.push_back(table);

    // 2. Obstacle Box
    
    moveit_msgs::msg::CollisionObject wall;
    wall.id = "obstacle_box";
    wall.header.frame_id = frame_id;
    wall.primitives.resize(1);
    wall.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    
    wall.primitives[0].dimensions = {
        0.4 - (-0.4),        
        0.20 - 0,
        0.1 - 0.0
    };
    
    wall.primitive_poses.resize(1);
    wall.primitive_poses[0].position.x = 0.0;
    wall.primitive_poses[0].position.y = 0.18;
    wall.primitive_poses[0].position.z = 0.00;   
    wall.primitive_poses[0].orientation.w = 1.0;
    wall.operation = wall.ADD;
    collision_objects.push_back(wall);
    
    // 3. 2nd Obstacle Box
    
    moveit_msgs::msg::CollisionObject wall2;
    wall2.id = "obstacle_box_2";
    wall2.header.frame_id = frame_id;
    wall2.primitives.resize(1);
    wall2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    
    wall2.primitives[0].dimensions = {
        0.2 - 0.0,        
        0.4 - 0.0,
        0.3 - 0.0
    };
    
    wall2.primitive_poses.resize(1);
    wall2.primitive_poses[0].position.x = -0.4;
    wall2.primitive_poses[0].position.y = 0.1;
    wall2.primitive_poses[0].position.z = 0.00;   
    wall2.primitive_poses[0].orientation.w = 1.0;
    wall2.operation = wall2.ADD;
    collision_objects.push_back(wall2);
    
    planning_scene_interface.applyCollisionObjects(collision_objects);
    std::cout << ">>> [INIT] Collision objects loaded into scene.\n";
}

void waitForStateSettle(
    moveit::planning_interface::MoveGroupInterface & iface,
    int ms = STATE_SETTLE_MS)
{
    rclcpp::sleep_for(std::chrono::milliseconds(ms));
    iface.startStateMonitor(1.0); 
}

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

// Find IK-valid overhead pose by scanning YAW angles (Z-axis rotation)

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
    iface.setPlanningTime(5.0);
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
    iface.setPlanningTime(5.0);
    iface.setPoseTarget(target_pose);

    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = iface.getEndEffectorLink();
    ocm.header.frame_id = iface.getPlanningFrame();
    ocm.orientation.x = q_upright.x();
    ocm.orientation.y = q_upright.y();
    ocm.orientation.z = q_upright.z();
    ocm.orientation.w = q_upright.w();
    ocm.absolute_x_axis_tolerance = 3.14; 
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2; 
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

    // 1. Attempt Cartesian
    
    if (strictCartesianMove(iface, final_pose, "DROP_DIRECT")) {
        return true;
    }

    std::cout << "    [-] Direct drop blocked. Using 2-Stage Approach & Insert...\n";

    // 2. Calculate Approach Pose
    double approach_z = target_z + 0.05; 
    if (start_pose.position.z <= approach_z) {
        std::cout << "    [-] Arm is already too low for a 2-stage drop.\n";
        return false;
    }

    geometry_msgs::msg::Pose approach_pose = start_pose;
    approach_pose.position.z = approach_z;

    // --- STAGE A: Constrained OMPL Descent ---
    
    iface.setPlanningPipelineId("ompl");
    iface.setPlannerId("RRTConnectkConfigDefault");
    iface.setPlanningTime(5.0);
    iface.setPoseTarget(approach_pose);

    // Keep the tube rigidly aligned to the specified RPY constraints
    
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = iface.getEndEffectorLink();
    ocm.header.frame_id = iface.getPlanningFrame();
    ocm.orientation.x = q_upright.x();
    ocm.orientation.y = q_upright.y();
    ocm.orientation.z = q_upright.z();
    ocm.orientation.w = q_upright.w();
    
    // STRICT tolerances to enforce precise RPY during OMPL planning
    
    ocm.absolute_x_axis_tolerance = 3.14; 
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2; 
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

// 2-Stage Smart Lift (Extraction & Ascent)

bool smartVerticalLift(
    moveit::planning_interface::MoveGroupInterface & iface,
    double safe_z_target,
    const tf2::Quaternion & q_upright)
{
    geometry_msgs::msg::Pose start_pose = iface.getCurrentPose().pose;
    geometry_msgs::msg::Pose final_pose = start_pose;
    final_pose.position.z = safe_z_target;

    if (start_pose.position.z >= safe_z_target) return true;

    // Pure Cartesian
    
    if (strictCartesianMove(iface, final_pose, "LIFT_DIRECT")) {
        return true;
    }

    std::cout << "    [-] Direct lift blocked. Using 2-Stage Extraction & Ascent...\n";

    // Calculate Extraction Pose
    
    double extraction_z = start_pose.position.z + 0.05; 
    if (extraction_z >= safe_z_target) {
        // If 5cm puts us past the target and direct failed, abort.
        return false;
    }

    geometry_msgs::msg::Pose extraction_pose = start_pose;
    extraction_pose.position.z = extraction_z;

    // Strict Cartesian pull to clear the rack
    
    if (!strictCartesianMove(iface, extraction_pose, "LIFT_EXTRACT")) {
        std::cout << "    [-] Extraction phase failed. Cannot clear the immediate area strictly.\n";
        return false;
    }

    // 3. STAGE B: Constrained OMPL Ascent to final height
    
    std::cout << "    [+] Extraction reached. Executing final constrained ascent...\n";
    
    // Sync the state after the extraction move
    
    iface.setStartStateToCurrentState();
    
    iface.setPlanningPipelineId("ompl");
    iface.setPlannerId("RRTConnectkConfigDefault");
    iface.setPlanningTime(5.0);
    iface.setPoseTarget(final_pose);

    // Keep the tube rigidly aligned to the specified RPY constraints
    
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = iface.getEndEffectorLink();
    ocm.header.frame_id = iface.getPlanningFrame();
    ocm.orientation.x = q_upright.x();
    ocm.orientation.y = q_upright.y();
    ocm.orientation.z = q_upright.z();
    ocm.orientation.w = q_upright.w();
    
    // STRICT tolerances to enforce precise RPY during OMPL planning
    
    ocm.absolute_x_axis_tolerance = 3.14; 
    ocm.absolute_y_axis_tolerance = 0.2;
    ocm.absolute_z_axis_tolerance = 0.2; 
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    iface.setPathConstraints(path_constraints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ascent_success = false;
    
    if (iface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        if (iface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            waitForStateSettle(iface);
            ascent_success = true;
        }
    }
    
    iface.clearPathConstraints();

    if (!ascent_success) {
        std::cout << "    [-] Ascent phase failed.\n";
    }

    return ascent_success;
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
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
    rclcpp::sleep_for(std::chrono::seconds(1));
    setupCollisionObjects(arm_interface.getPlanningFrame());

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

    std::cout << "\n>>> Ready. Waiting for GUI commands on '/gui_commands'...\n";

    // Prevent double-execution if GUI buttons are mashed
    bool is_busy = false;

    // --- NEW: ROS 2 Subscription replaces the while() loop ---
    auto gui_sub = node->create_subscription<std_msgs::msg::String>(
        "/gui_commands", 10,
        [&](const std_msgs::msg::String::SharedPtr msg) {
            if (is_busy) {
                std::cout << "[-] Robot is busy. Ignoring command: " << msg->data << "\n";
                return;
            }
            
            is_busy = true;
            std::string line = msg->data;
            std::cout << "\n[GUI COMMAND RECEIVED]: " << line << "\n";

            if (line == "q" || line == "Q") {
                std::cout << ">>> Shutting down.\n";
                rclcpp::shutdown();
                return;
            }

            // 1. Check for Gripper Commands
            if (line == "o" || line == "O") {
                std::cout << ">>> Opening Gripper...\n";
                gripper_interface.setNamedTarget("open");
                gripper_interface.move(); 
                is_busy = false;
                return; 
            }
        
            if (line == "c" || line == "C") {
                std::cout << ">>> Closing Gripper...\n";
                gripper_interface.setNamedTarget("closed");
                gripper_interface.move();
                is_busy = false;
                return;
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
                        std::cout << "    [+] Tube Rotated. Waiting...\n";
                        
                        // Wait for liquid to drain

                        rclcpp::sleep_for(std::chrono::seconds(1));

                        // 4. Return to the upright state

                        std::cout << "    [+] Returning to upright...\n";
                        joint_positions.back() = original_last_joint;
                        arm_interface.setJointValueTarget(joint_positions);
                        arm_interface.move();
                        waitForStateSettle(arm_interface);
                        
                        std::cout << ">>> The liquid was poured!\n";
                    } else {
                        std::cout << "    [-] Could not execute pour.\n";
                    }
                }
                is_busy = false;
                return;
            }
            
            
            double tx, ty, tz;

            if (line[0] == 'm' || line[0] == 'M') {
                int marker_id;
                std::stringstream ss(line.substr(1));
                if (ss >> marker_id) {
                    // If the user requests the anchor itself
                    if (marker_id == 0) {
                        tx = 0.0;
                        ty = -0.065;
                        tz = 0.20;
                        std::cout << "    [+] Anchor Marker 0 at Base Coordinates: (" << tx << ", " << ty << ", " << tz << ")\n";
                    } else {
                        // Look up dynamically via TF2
                        std::cout << ">>> Looking up position for Marker " << marker_id << " via camera TF...\n";
                        try {
                            std::string target_frame = "marker_" + std::to_string(marker_id);
                            
                            // We ask the buffer: "Where is marker_X relative to marker_base?"
                            geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(
                                "marker_base", target_frame, tf2::TimePointZero);
                            
                            // Convert from marker_base frame to robot base frame
                            tx = t.transform.translation.x + 0.0;
                            ty = t.transform.translation.y + 0.11;
                            tz = 0.15; // Enforce the hardcoded Z height 
                            
                            std::cout << "    [+] Marker " << marker_id << " Found! Robot Coordinates: (" << tx << ", " << ty << ", " << tz << ")\n";
                        } catch (const tf2::TransformException & ex) {
                            std::cout << "    [-] Could not find Marker " << marker_id << " in TF tree. Falling back to hardcoded positions...\n";
                            
                            // --- NEW FALLBACK LOGIC ---
                            if (marker_id == 1) {
                                tx = -0.15;
                                ty = -0.15;
                                tz = 0.1;
                            } else if (marker_id == 2) {
                                tx = -0.05;
                                ty = -0.20;
                                tz = 0.12;
                            } else if (marker_id == 3) {
                                tx = 0.00;
                                ty = -0.22;
                                tz = 0.11;
                            } else if (marker_id == 4) {
                                tx = 0.05;
                                ty = -0.16;
                                tz = 0.12;
                            } else if (marker_id == 5) {
                                tx = 0.15;
                                ty = -0.15;
                                tz = 0.12;
                            } else if (marker_id == 6) {
                                tx = -0.2;
                                ty = -0.1;
                                tz = 0.12;       
                            } else {
                                // If the marker ID is entirely unknown
                                std::cout << "    [-] No fallback defined for Marker " << marker_id << ". Aborting move.\n";
                                is_busy = false;
                                return; 
                            }
                            
                            std::cout << "    [+] Fallback engaged. Target Coordinates: (" << tx << ", " << ty << ", " << tz << ")\n";
                        }
                    }
                } else {
                    std::cout << "[-] Invalid marker input. Example: 'M 2'\n";
                    is_busy = false;
                    return;
                }
            } else {
                // Standard XYZ coordinate parsing
                std::stringstream ss(line);
                if (!(ss >> tx >> ty >> tz)) {
                    std::cout << "[-] Invalid input. \n";
                    is_busy = false;
                    return;
                }
            }

            // PHASE 1: LIFT  — 2-Stage Smart Lift to fixed SAFE_Z_TARGET

            std::cout << "\n--- [PHASE 1] LIFT to Standard Z=" << SAFE_Z_TARGET << " ---\n";

            arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
            arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

            if (smartVerticalLift(arm_interface, SAFE_Z_TARGET, q_upright)) {
                waitForStateSettle(arm_interface);
            } else {
                std::cout << "[-] [PHASE 1] Cannot reach standard lift height Z=" << SAFE_Z_TARGET 
                          << ". Skipping the target.\n";
                is_busy = false;
                return;
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
                std::cout << "    [-] Cartesian detours failed. Attempting constrained OMPL fallback...\n";
                arm_interface.setStartStateToCurrentState();
                
                arm_interface.setPlanningPipelineId("ompl");
                arm_interface.setPlannerId("RRTConnectkConfigDefault");
                arm_interface.setPlanningTime(5.0);
                arm_interface.setPoseTarget(target_pose);

                moveit_msgs::msg::OrientationConstraint ocm;
                ocm.link_name = arm_interface.getEndEffectorLink();
                ocm.header.frame_id = arm_interface.getPlanningFrame();
                ocm.orientation.x = q_upright.x();
                ocm.orientation.y = q_upright.y();
                ocm.orientation.z = q_upright.z();
                ocm.orientation.w = q_upright.w();
                
                ocm.absolute_x_axis_tolerance = 3.14; 
                ocm.absolute_y_axis_tolerance = 0.2;
                ocm.absolute_z_axis_tolerance = 0.2; 
                ocm.weight = 1.0;

                moveit_msgs::msg::Constraints path_constraints;
                path_constraints.orientation_constraints.push_back(ocm);
                arm_interface.setPathConstraints(path_constraints);

                moveit::planning_interface::MoveGroupInterface::Plan plan;
                if (arm_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    if (arm_interface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                        waitForStateSettle(arm_interface);
                        moved = true;
                        std::cout << "    [+] OMPL horizontal transit successful.\n";
                    }
                }
                arm_interface.clearPathConstraints();
            }

            if (!moved) {
                std::cout << "[-] [PHASE 2] All horizontal attempts failed. Target unreachable upright.\n";
                is_busy = false;
                return;
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
            
            // Free the lock when the command finishes
            is_busy = false;
        }
    );

    // Keep the node alive so it can listen to the GUI
    spinner.join();
    return 0;
}
