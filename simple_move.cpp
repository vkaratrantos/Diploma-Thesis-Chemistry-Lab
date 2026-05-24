// Liquid Handling Robotic System

#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <vector>
#include <chrono>
#include <queue> 
#include <mutex>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> 
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

static constexpr double VEL_SCALE_TRANSIT  = 0.3; 
static constexpr double VEL_SCALE_LIQUID   = 0.3; 
static constexpr double ACC_SCALE_TRANSIT  = 0.1;
static constexpr double ACC_SCALE_LIQUID   = 0.1;
static constexpr double CARTESIAN_EEF_STEP = 0.005; 
static constexpr double MIN_CARTESIAN_FRACTION = 0.8;
static constexpr double JUMP_THRESHOLD     = 1.5;

// MOTION VARIABLES

static constexpr double SAFE_Z_TARGET      = 0.28;  
static constexpr double DROP_Z_RETRY_STEP  = 0.005;  
static constexpr double DROP_Z_MAX_RETRY   = 0.08;  
static constexpr int    STATE_SETTLE_MS    = 500;   

// QUEUE

std::queue<std::string> command_queue;
std::mutex queue_mutex;

// COLLISION OBJECTS

void setupCollisionObjects(const std::string& frame_id) {

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // TABLE SURFACE COLLISION OBJECT
    
    moveit_msgs::msg::CollisionObject table;
    table.id = "table_base";
    table.header.frame_id = frame_id;
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {1.0, 1.0, 0.02}; 
    table.primitive_poses.resize(1);
    table.primitive_poses[0].position.x = 0.0;
    table.primitive_poses[0].position.y = 0.0;
    table.primitive_poses[0].position.z = -0.02; 
    table.primitive_poses[0].orientation.w = 1.0;
    table.operation = table.ADD;
    collision_objects.push_back(table);

    // 1ST BOX OBJECT
    
    moveit_msgs::msg::CollisionObject wall;
    wall.id = "obstacle_box";
    wall.header.frame_id = frame_id;
    wall.primitives.resize(1);
    wall.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    wall.primitives[0].dimensions = { 0.5, 0.2, 0.05 };
    wall.primitive_poses.resize(1);
    wall.primitive_poses[0].position.x = 0.0;
    wall.primitive_poses[0].position.y = 0.18;
    wall.primitive_poses[0].position.z = 0.00;   
    wall.primitive_poses[0].orientation.w = 1.0;
    wall.operation = wall.ADD;
    collision_objects.push_back(wall);

    // 2ND BOX OBJECT
    
    moveit_msgs::msg::CollisionObject wall2;
    wall2.id = "obstacle_box_2";
    wall2.header.frame_id = frame_id;
    wall2.primitives.resize(1);
    wall2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    wall2.primitives[0].dimensions = { 0.2, 0.4, 0.15 };
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

// CARTESIAN MOVES FUNCTION

bool computeCartesianMove(
    moveit::planning_interface::MoveGroupInterface & iface,
    const geometry_msgs::msg::Pose & target_pose,
    const std::string & phase_name)
{
    std::cout << "    [" << phase_name << "] Computing Cartesian Path...\n";

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    
    // 1. CALCULATION OF THE GEOMETRIC PATH
    
    double fraction = iface.computeCartesianPath(waypoints, CARTESIAN_EEF_STEP, JUMP_THRESHOLD, trajectory);

    if (fraction >= MIN_CARTESIAN_FRACTION) {
        
        robot_trajectory::RobotTrajectory rt(iface.getRobotModel(), iface.getName());
        rt.setRobotTrajectoryMsg(*iface.getCurrentState(), trajectory);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        totg.computeTimeStamps(rt, VEL_SCALE_LIQUID, ACC_SCALE_LIQUID);        
        rt.getRobotTrajectoryMsg(trajectory); // Save the smoothed path back
        std::cout << "    [+] Cartesian path found and smoothed (" << (fraction * 100.0) << "%). Executing...\n";
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        return (iface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    } 
    std::cout << "    [-] Cartesian path failed at " << (fraction * 100.0) << "%. Collision detected.\n";
    return false;
}

// SMART DROP FUNCTION

bool smartVerticalDrop(
    moveit::planning_interface::MoveGroupInterface & iface,
    double target_z,
    const tf2::Quaternion & q_upright)
{
    geometry_msgs::msg::Pose start_pose = iface.getCurrentPose().pose;
    geometry_msgs::msg::Pose final_pose = start_pose;
    final_pose.position.z = target_z;

    if (start_pose.position.z <= target_z) {
        std::cout << "    [!] Warning: Arm is already at or below target Z.\n";
        return true;
    }

    // 1. CARTESIAN (VERTICAL DROP)

    if (computeCartesianMove(iface, final_pose, "DROP_DIRECT")) {
        waitForStateSettle(iface);
        return true;
    }
    std::cout << "    [-] Direct drop blocked.\n";

    // OMPL SETTINGS
    
    iface.setPlanningPipelineId("ompl");
    iface.setPlannerId("RRTConnectkConfigDefault");
    iface.setPlanningTime(10.0);

    moveit_msgs::msg::OrientationConstraint base_ocm;
    base_ocm.link_name = iface.getEndEffectorLink();
    base_ocm.header.frame_id = iface.getPlanningFrame();
    base_ocm.orientation.x = q_upright.x();
    base_ocm.orientation.y = q_upright.y();
    base_ocm.orientation.z = q_upright.z();
    base_ocm.orientation.w = q_upright.w();
    base_ocm.weight = 1.0;

    // 2. HYBRID MOVEMENT (OMPL to APPROACH_Z AND CARTESIAN TO TARGET_Z)

    double approach_z = target_z + 0.08; 
    
    // ATTEMPT THE HYBRID MOTION ONLY IF WE ARE ABOVE THE APPROACH HEIGHT
    
    if (start_pose.position.z > approach_z) {
        std::cout << "    [-] Attempting Hybrid: OMPL Approach -> Cartesian Insert...\n";
        
        geometry_msgs::msg::Pose approach_pose = start_pose;
        approach_pose.position.z = approach_z;
        iface.setPoseTarget(approach_pose);

        base_ocm.absolute_x_axis_tolerance = M_PI; 
        base_ocm.absolute_y_axis_tolerance = 0.4;
        base_ocm.absolute_z_axis_tolerance = 0.4; 
        
        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.orientation_constraints.push_back(base_ocm);
        iface.setPathConstraints(path_constraints);

        moveit::planning_interface::MoveGroupInterface::Plan hybrid_plan;
        bool approach_success = false;

        if (iface.plan(hybrid_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            
            robot_trajectory::RobotTrajectory rt(iface.getRobotModel(), iface.getName());
            rt.setRobotTrajectoryMsg(*iface.getCurrentState(), hybrid_plan.trajectory_);
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            totg.computeTimeStamps(rt, VEL_SCALE_LIQUID, ACC_SCALE_LIQUID);
            rt.getRobotTrajectoryMsg(hybrid_plan.trajectory_);

            if (iface.execute(hybrid_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                approach_success = true;
                waitForStateSettle(iface);
            }
        }
        
        iface.clearPathConstraints();

        if (approach_success) {
            std::cout << "    [+] Approach reached. Executing final vertical insertion...\n";
            iface.setStartStateToCurrentState();
            
            if (computeCartesianMove(iface, final_pose, "DROP_INSERT")) {
                waitForStateSettle(iface);
                return true;
            } else {
                std::cout << "    [-] Cartesian insertion failed from approach position.\n";
            }
        } else {
            std::cout << "    [-] OMPL approach to hover position failed.\n";
        }
    } else {
        std::cout << "    [-] Arm already below approach threshold. Skipping Hybrid phase.\n";
    }

    // 3. OMPL MOTION DIRECTLY TO TARGET_Z WITH STRICT CONSTRAINTS

    std::cout << "    [-] Attempting Pure Constrained OMPL directly to target Z...\n";
    iface.setStartStateToCurrentState();
    iface.setPoseTarget(final_pose);
    
    base_ocm.absolute_x_axis_tolerance = M_PI; 
    base_ocm.absolute_y_axis_tolerance = 0.4;
    base_ocm.absolute_z_axis_tolerance = 0.4; 
    
    moveit_msgs::msg::Constraints strict_constraints;
    strict_constraints.orientation_constraints.push_back(base_ocm);
    iface.setPathConstraints(strict_constraints);

    moveit::planning_interface::MoveGroupInterface::Plan strict_plan;
    if (iface.plan(strict_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        
        robot_trajectory::RobotTrajectory rt(iface.getRobotModel(), iface.getName());
        rt.setRobotTrajectoryMsg(*iface.getCurrentState(), strict_plan.trajectory_);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        totg.computeTimeStamps(rt, VEL_SCALE_LIQUID, ACC_SCALE_LIQUID);
        rt.getRobotTrajectoryMsg(strict_plan.trajectory_);

        if (iface.execute(strict_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            iface.clearPathConstraints();
            waitForStateSettle(iface);
            return true;
        }
    }
    iface.clearPathConstraints();

    // 4. OMPL MOTION DIRECTLY TO TARGET_Z WITH RELAXED CONSTRAINTS

    std::cout << "    [-] Strict OMPL failed. Relaxing constraints...\n";
    iface.setStartStateToCurrentState();
    iface.setPoseTarget(final_pose);
    
    // RELAXED CONSTRAINTS
    
    base_ocm.absolute_x_axis_tolerance = M_PI; 
    base_ocm.absolute_y_axis_tolerance = 0.8;
    base_ocm.absolute_z_axis_tolerance = 0.8; 
    
    moveit_msgs::msg::Constraints relaxed_constraints;
    relaxed_constraints.orientation_constraints.push_back(base_ocm);
    iface.setPathConstraints(relaxed_constraints);

    moveit::planning_interface::MoveGroupInterface::Plan relaxed_plan;
    if (iface.plan(relaxed_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        
        robot_trajectory::RobotTrajectory rt(iface.getRobotModel(), iface.getName());
        rt.setRobotTrajectoryMsg(*iface.getCurrentState(), relaxed_plan.trajectory_);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        totg.computeTimeStamps(rt, VEL_SCALE_LIQUID, ACC_SCALE_LIQUID);
        rt.getRobotTrajectoryMsg(relaxed_plan.trajectory_);

        if (iface.execute(relaxed_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            iface.clearPathConstraints();
            waitForStateSettle(iface);
            std::cout << "    [!] Warning: Executed drop with relaxed tolerances.\n";
            return true;
        }
    }
    iface.clearPathConstraints();

    // 5. ABORT

    std::cout << "    [!] CRITICAL: All drop attempts failed. Cannot reach target Z.\n";
    return false;
}

// SMART LIFT FUNCTION

bool smartVerticalLift(
    moveit::planning_interface::MoveGroupInterface & iface,
    double safe_z_target,
    const tf2::Quaternion & q_upright)
{
    geometry_msgs::msg::Pose start_pose = iface.getCurrentPose().pose;
    geometry_msgs::msg::Pose final_pose = start_pose;
    final_pose.position.z = safe_z_target;

    if (start_pose.position.z >= safe_z_target) return true;

    // 1. CARTESIAN (VERTICAL LIFT)

    if (computeCartesianMove(iface, final_pose, "LIFT_DIRECT")) {
        waitForStateSettle(iface);
        return true;
    }
    std::cout << "    [-] Direct lift blocked.\n";

    // OMPL SETTINGS
    
    iface.setPlanningPipelineId("ompl");
    iface.setPlannerId("RRTConnectkConfigDefault");
    iface.setPlanningTime(10.0);
    iface.setPoseTarget(final_pose);

    moveit_msgs::msg::OrientationConstraint base_ocm;
    base_ocm.link_name = iface.getEndEffectorLink();
    base_ocm.header.frame_id = iface.getPlanningFrame();
    base_ocm.orientation.x = q_upright.x();
    base_ocm.orientation.y = q_upright.y();
    base_ocm.orientation.z = q_upright.z();
    base_ocm.orientation.w = q_upright.w();
    base_ocm.weight = 1.0;

    // 2. HYBRID MOVEMENT (CARTESIAN TO EXTRACTION_Z AND OMPL TO SAFE_Z)

    double extraction_z = start_pose.position.z + 0.08; 
    if (extraction_z > safe_z_target) {
        extraction_z = safe_z_target;
    }

    geometry_msgs::msg::Pose extraction_pose = start_pose;
    extraction_pose.position.z = extraction_z;

    if (computeCartesianMove(iface, extraction_pose, "LIFT_EXTRACT")) {
        std::cout << "    [+] Extraction reached. Executing final constrained ascent...\n";
        iface.setStartStateToCurrentState();
        
        base_ocm.absolute_x_axis_tolerance = M_PI; 
        base_ocm.absolute_y_axis_tolerance = 0.4;
        base_ocm.absolute_z_axis_tolerance = 0.4; 
        
        moveit_msgs::msg::Constraints path_constraints;
        path_constraints.orientation_constraints.push_back(base_ocm);
        iface.setPathConstraints(path_constraints);

        moveit::planning_interface::MoveGroupInterface::Plan hybrid_plan;
        if (iface.plan(hybrid_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            
            robot_trajectory::RobotTrajectory rt(iface.getRobotModel(), iface.getName());
            rt.setRobotTrajectoryMsg(*iface.getCurrentState(), hybrid_plan.trajectory_);
            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            totg.computeTimeStamps(rt, VEL_SCALE_LIQUID, ACC_SCALE_LIQUID);
            rt.getRobotTrajectoryMsg(hybrid_plan.trajectory_);

            if (iface.execute(hybrid_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                iface.clearPathConstraints();
                waitForStateSettle(iface);
                return true;
            }
        }
        iface.clearPathConstraints();
        std::cout << "    [-] Hybrid ascent failed. Falling back to global OMPL...\n";
    } else {
        std::cout << "    [-] Extraction phase blocked. Falling back to global OMPL...\n";
    }

    // 3. OMPL MOTION DIRECTLY TO SAFE_Z WITH STRICT CONSTRAINTS

    std::cout << "    [-] Attempting Pure Constrained OMPL from current pose...\n";
    iface.setStartStateToCurrentState();
    
    base_ocm.absolute_x_axis_tolerance = M_PI; 
    base_ocm.absolute_y_axis_tolerance = 0.4;
    base_ocm.absolute_z_axis_tolerance = 0.4; 
    
    moveit_msgs::msg::Constraints strict_constraints;
    strict_constraints.orientation_constraints.push_back(base_ocm);
    iface.setPathConstraints(strict_constraints);

    moveit::planning_interface::MoveGroupInterface::Plan strict_plan;
    if (iface.plan(strict_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        
        robot_trajectory::RobotTrajectory rt(iface.getRobotModel(), iface.getName());
        rt.setRobotTrajectoryMsg(*iface.getCurrentState(), strict_plan.trajectory_);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        totg.computeTimeStamps(rt, VEL_SCALE_LIQUID, ACC_SCALE_LIQUID);
        rt.getRobotTrajectoryMsg(strict_plan.trajectory_);

        if (iface.execute(strict_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            iface.clearPathConstraints();
            waitForStateSettle(iface);
            return true;
        }
    }
    iface.clearPathConstraints();

    // 3. OMPL MOTION DIRECTLY TO TARGET_Z WITH RELAXED CONSTRAINTS

    std::cout << "    [-] Strict OMPL failed. Relaxing constraints (allowing slight tilt)...\n";
    iface.setStartStateToCurrentState();
    
    base_ocm.absolute_x_axis_tolerance = M_PI; 
    base_ocm.absolute_y_axis_tolerance = 0.8;
    base_ocm.absolute_z_axis_tolerance = 0.8; 
    
    moveit_msgs::msg::Constraints relaxed_constraints;
    relaxed_constraints.orientation_constraints.push_back(base_ocm);
    iface.setPathConstraints(relaxed_constraints);

    moveit::planning_interface::MoveGroupInterface::Plan relaxed_plan;
    if (iface.plan(relaxed_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        
        robot_trajectory::RobotTrajectory rt(iface.getRobotModel(), iface.getName());
        rt.setRobotTrajectoryMsg(*iface.getCurrentState(), relaxed_plan.trajectory_);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        totg.computeTimeStamps(rt, VEL_SCALE_LIQUID, ACC_SCALE_LIQUID);
        rt.getRobotTrajectoryMsg(relaxed_plan.trajectory_);

        if (iface.execute(relaxed_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            iface.clearPathConstraints();
            waitForStateSettle(iface);
            std::cout << "    [!] Warning: Executed lift with relaxed tolerances.\n";
            return true;
        }
    }
    iface.clearPathConstraints();

    // 5. ABORT

    std::cout << "    [!] CRITICAL: All lift attempts failed. Tube is trapped.\n";
    return false;
}

// MAIN FUNCTION

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
    arm_interface.setPlanningTime(10.0);
    arm_interface.setGoalPositionTolerance(0.01);
    arm_interface.setGoalOrientationTolerance(0.1);

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

    auto gui_sub = node->create_subscription<std_msgs::msg::String>(
        "/gui_commands", 10,
        [&](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(queue_mutex);
            command_queue.push(msg->data);
            std::cout << "[QUEUE] Received command: " << msg->data << " (Pending tasks: " << command_queue.size() << ")\n";
        }
    );

    // MAIN EXECUTION LOOP
    
    while (rclcpp::ok()) {
        std::string line = "";
        
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (!command_queue.empty()) {
                line = command_queue.front();
                command_queue.pop();
            }
        }

        if (line.empty()) {
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        std::cout << "\n[EXECUTING COMMAND]: " << line << "\n";

        if (line == "q" || line == "Q") break;

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

        if (line == "p" || line == "P") {
            std::cout << ">>> Pouring the liquid...\n";
            std::vector<double> joint_positions;
            moveit::core::RobotStatePtr current_state = arm_interface.getCurrentState();
            const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup("arm_group");
            current_state->copyJointGroupPositions(joint_model_group, joint_positions);

            if (!joint_positions.empty()) {
                double original_last_joint = joint_positions.back();
                joint_positions.back() = original_last_joint + (M_PI / 2.0);
                arm_interface.setJointValueTarget(joint_positions);
                
                if (arm_interface.move() == moveit::core::MoveItErrorCode::SUCCESS) {
                    waitForStateSettle(arm_interface);
                    std::cout << "    [+] Tube Rotated. Waiting...\n";
                    rclcpp::sleep_for(std::chrono::seconds(1));
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
            continue;
        }
        
        double tx, ty, tz;

        if (line[0] == 'm' || line[0] == 'M') {
            int marker_id;
            std::stringstream ss(line.substr(1));
            if (ss >> marker_id) {
                if (marker_id == 0) {
                    tx = 0.0; ty = -0.065; tz = 0.20;
                } else {
                    try {
                        std::string target_frame = "marker_" + std::to_string(marker_id);
                        geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(
                            "marker_base", target_frame, tf2::TimePointZero);
                        tx = t.transform.translation.x + 0.0;
                        ty = t.transform.translation.y + 0.11;
                        tz = 0.15; 
                    } catch (const tf2::TransformException & ex) {
                        std::cout << "    [-] Falling back to hardcoded positions\n";
                        if (marker_id == 1) { tx = -0.155; ty = -0.147; tz = 0.1; } 
                        else if (marker_id == 2) { tx = -0.051; ty = -0.201; tz = 0.1; } 
                        else if (marker_id == 3) { tx = 0.022; ty = -0.213; tz = 0.1; } 
                        else if (marker_id == 4) { tx = 0.117; ty = -0.192; tz = 0.1; } 
                        else if (marker_id == 5) { tx = 0.156; ty = -0.149; tz = 0.1; } 
                        else if (marker_id == 6) { tx = -0.205; ty = -0.112; tz = 0.1; } 
                        else {
                            continue; 
                        }
                    }
                }
            } else {
                continue;
            }
        } else {
            std::stringstream ss(line);
            if (!(ss >> tx >> ty >> tz)) continue;
        }

        // PHASE 1: LIFT
        
        std::cout << "\n--- [PHASE 1] LIFT to Standard Z=" << SAFE_Z_TARGET << " ---\n";
        arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
        arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

        if (smartVerticalLift(arm_interface, SAFE_Z_TARGET, q_upright)) {
            waitForStateSettle(arm_interface);
        } else {
            std::cout << "[-] [PHASE 1] Cannot reach standard lift height. Skipping.\n";
            continue;
        }

        // PHASE 2: HORIZONTAL MOVE
        
        std::cout << "\n--- [PHASE 2] HORIZONTAL MOVE to (" << tx << ", " << ty << ") ---\n";
        arm_interface.setStartStateToCurrentState();
        geometry_msgs::msg::Pose start_pose = arm_interface.getCurrentPose().pose;
        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.x = tx;
        target_pose.position.y = ty;
        target_pose.position.z = SAFE_Z_TARGET; 

        bool moved = false;

        // HORIZONTAL ATTEMPTS
        // 1. CARTESIAN PATHS
        
        std::cout << "    [!] Attempting Cartesian interpolation...\n";
        
        if (computeCartesianMove(arm_interface, target_pose, "HORIZ_DIRECT")) {
            waitForStateSettle(arm_interface);
            moved = true;
        } 

        // 2. STRICT OMPL
        
        if (!moved) {
            std::cout << "    [-] Cartesian path blocked. Falling back to Strict OMPL...\n";
            
            arm_interface.setStartStateToCurrentState();
            arm_interface.setPlanningPipelineId("ompl"); 
            arm_interface.setPlannerId("RRTConnectkConfigDefault"); 
            arm_interface.setPlanningTime(10.0); 
            arm_interface.setPoseTarget(target_pose);

            moveit_msgs::msg::OrientationConstraint strict_ocm;
            strict_ocm.link_name = arm_interface.getEndEffectorLink();
            strict_ocm.header.frame_id = arm_interface.getPlanningFrame();
            strict_ocm.orientation.x = q_upright.x();
            strict_ocm.orientation.y = q_upright.y();
            strict_ocm.orientation.z = q_upright.z();
            strict_ocm.orientation.w = q_upright.w();
            
            strict_ocm.absolute_x_axis_tolerance = M_PI; 
            strict_ocm.absolute_y_axis_tolerance = 0.4; 
            strict_ocm.absolute_z_axis_tolerance = 0.4; 
            strict_ocm.weight = 1.0;

            moveit_msgs::msg::Constraints strict_constraints;
            strict_constraints.orientation_constraints.push_back(strict_ocm);
            arm_interface.setPathConstraints(strict_constraints);

            moveit::planning_interface::MoveGroupInterface::Plan strict_plan;
            
            if (arm_interface.plan(strict_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                std::cout << "    [+] Valid OMPL detour found. Smoothing velocities...\n";
                
                robot_trajectory::RobotTrajectory rt(arm_interface.getRobotModel(), arm_interface.getName());
                rt.setRobotTrajectoryMsg(*arm_interface.getCurrentState(), strict_plan.trajectory_);
                
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                totg.computeTimeStamps(rt, VEL_SCALE_LIQUID, ACC_SCALE_LIQUID);
                
                rt.getRobotTrajectoryMsg(strict_plan.trajectory_);

                if (arm_interface.execute(strict_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    waitForStateSettle(arm_interface);
                    moved = true;
                }
            }
            arm_interface.clearPathConstraints();
        }
        
        // 3. Z-EXPLORATION

        if (!moved) {
            std::cout << "[-] Strict OMPL blocked. Exploring alternative Z-heights... \n";
            
            std::vector<double> z_offsets = {-0.01, 0.01, -0.02, 0.02, -0.03, 0.03, -0.04, 0.04};
            
            for (double offset : z_offsets) {
                std::cout <<"     [*] Trying Z-offset: " << (offset * 100.0) << " cm...\n";
            
                geometry_msgs::msg::Pose explore_target = target_pose;
                explore_target.position.z += offset;
            
                arm_interface.setStartStateToCurrentState();
                arm_interface.setPoseTarget(explore_target);
            
                moveit_msgs::msg::OrientationConstraint strict_ocm;
                strict_ocm.link_name = arm_interface.getEndEffectorLink();
                strict_ocm.header.frame_id = arm_interface.getPlanningFrame();
                strict_ocm.orientation.x = q_upright.x();
                strict_ocm.orientation.y = q_upright.y();
                strict_ocm.orientation.z = q_upright.z();
                strict_ocm.orientation.w = q_upright.w();
                strict_ocm.absolute_x_axis_tolerance = M_PI; 
                strict_ocm.absolute_y_axis_tolerance = 0.4; 
                strict_ocm.absolute_z_axis_tolerance = 0.4; 
                strict_ocm.weight = 1.0;
            
                moveit_msgs::msg::Constraints explore_constraints;
                explore_constraints.orientation_constraints.push_back(strict_ocm);
                arm_interface.setPathConstraints(explore_constraints);
            
                moveit::planning_interface::MoveGroupInterface::Plan explore_plan;
                if (arm_interface.plan(explore_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                
                    robot_trajectory::RobotTrajectory rt(arm_interface.getRobotModel(), arm_interface.getName());
                    rt.setRobotTrajectoryMsg(*arm_interface.getCurrentState(), explore_plan.trajectory_);
                    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                    totg.computeTimeStamps(rt, VEL_SCALE_LIQUID, ACC_SCALE_LIQUID);
                    rt.getRobotTrajectoryMsg(explore_plan.trajectory_);
                
                    if (arm_interface.execute(explore_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                        waitForStateSettle(arm_interface);
                    
                        std::cout << "    [+] Offset. Realigning to 0.28 m...\n";
                        arm_interface.setStartStateToCurrentState();
                    
                        if (computeCartesianMove(arm_interface, target_pose, "HORIZ_DROP")) {
                            waitForStateSettle(arm_interface);
                        } else {
                            std::cout << "    [-] Failed to recover to standard Z! Continuing from current height.\n";
                        }
                        
                        moved = true;
                        arm_interface.clearPathConstraints();
                        break;
                    }
                }
                arm_interface.clearPathConstraints();
            }
        }
        
        // 4. RELAXED OMPL

        if (!moved) {
            std::cout << "    [-] Elevated Hop blocked. Relaxing orientation constraints...\n";
            
            arm_interface.setStartStateToCurrentState();
            arm_interface.setPoseTarget(target_pose);

            moveit_msgs::msg::OrientationConstraint relaxed_ocm;
            relaxed_ocm.link_name = arm_interface.getEndEffectorLink();
            relaxed_ocm.header.frame_id = arm_interface.getPlanningFrame();
            relaxed_ocm.orientation.x = q_upright.x();
            relaxed_ocm.orientation.y = q_upright.y();
            relaxed_ocm.orientation.z = q_upright.z();
            relaxed_ocm.orientation.w = q_upright.w();
            relaxed_ocm.absolute_x_axis_tolerance = M_PI;
            relaxed_ocm.absolute_y_axis_tolerance = 0.8;
            relaxed_ocm.absolute_z_axis_tolerance = 0.8; 
            relaxed_ocm.weight = 1.0;

            moveit_msgs::msg::Constraints relaxed_constraints;
            relaxed_constraints.orientation_constraints.push_back(relaxed_ocm);
            arm_interface.setPathConstraints(relaxed_constraints);

            moveit::planning_interface::MoveGroupInterface::Plan relaxed_plan;
            if (arm_interface.plan(relaxed_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                
                robot_trajectory::RobotTrajectory rt(arm_interface.getRobotModel(), arm_interface.getName());
                rt.setRobotTrajectoryMsg(*arm_interface.getCurrentState(), relaxed_plan.trajectory_);
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                totg.computeTimeStamps(rt, VEL_SCALE_LIQUID, ACC_SCALE_LIQUID);
                rt.getRobotTrajectoryMsg(relaxed_plan.trajectory_); 

                if (arm_interface.execute(relaxed_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    waitForStateSettle(arm_interface);
                    moved = true;
                }
            }
            arm_interface.clearPathConstraints();
        }

        // 5. ABORT

        if (!moved) {
            std::cout << "[-] [PHASE 2] Target unreachable. All horizontal planning failed.\n";
            continue; 
        }
        
        // PHASE 3: DROP
        
        std::cout << "\n--- [PHASE 3] DROP to Z=" << tz << " ---\n";
        arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
        arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

        bool dropped = false;
        
        for (double z_try = tz; z_try <= tz + DROP_Z_MAX_RETRY && !dropped; z_try += DROP_Z_RETRY_STEP) {
        
            if (smartVerticalDrop(arm_interface, z_try, q_upright)) {
                dropped = true;
                waitForStateSettle(arm_interface);
                std::cout << "\n>>> Sequence complete! Target reached safely.\n";
            }
        }
    }

    std::cout << "\n>>> Shutting down.\n";
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
