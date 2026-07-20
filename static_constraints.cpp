// LIQUID HANDLING 7-DOF ROBOTIC ARM -- FINAL VERSION

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
#include <atomic>
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

// MOTION VARIABLES

static constexpr int    STATE_SETTLE_MS    = 500;   

// QUEUE

std::queue<std::string> command_queue;
std::mutex queue_mutex;

// COLLISION OBJECTS

// Modified to accept tf_buffer so it can spawn tubes relative to live markers
void setupCollisionObjects(const std::string& frame_id, tf2_ros::Buffer& tf_buffer) {

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // TABLE SURFACE COLLISION OBJECT
    
    moveit_msgs::msg::CollisionObject table;
    table.id = "table_base";
    table.header.frame_id = frame_id;
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = {1.0, 1.0, 0.04}; 
    table.primitive_poses.resize(1);
    table.primitive_poses[0].position.x = 0.0;
    table.primitive_poses[0].position.y = 0.0;
    table.primitive_poses[0].position.z = -0.03; 
    table.primitive_poses[0].orientation.w = 1.0;
    table.operation = table.ADD;
    collision_objects.push_back(table);

    // 1ST BOX OBJECT
    
    moveit_msgs::msg::CollisionObject wall;
    wall.id = "obstacle_box";
    wall.header.frame_id = frame_id;
    wall.primitives.resize(1);
    wall.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    wall.primitives[0].dimensions = { 0.7, 0.2, 0.05 };
    wall.primitive_poses.resize(1);
    wall.primitive_poses[0].position.x = 0.0;
    wall.primitive_poses[0].position.y = 0.18;
    wall.primitive_poses[0].position.z = 0.02;   
    wall.primitive_poses[0].orientation.w = 1.0;
    wall.operation = wall.ADD;
    collision_objects.push_back(wall);

    // 2ND BOX OBJECT
    
    moveit_msgs::msg::CollisionObject wall2;
    wall2.id = "obstacle_box_2";
    wall2.header.frame_id = frame_id;
    wall2.primitives.resize(1);
    wall2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    wall2.primitives[0].dimensions = { 0.2, 0.4, 0.2 };
    wall2.primitive_poses.resize(1);
    wall2.primitive_poses[0].position.x = -0.4;
    wall2.primitive_poses[0].position.y = 0.1;
    wall2.primitive_poses[0].position.z = 0.03;   
    wall2.primitive_poses[0].orientation.w = 1.0;
    wall2.operation = wall2.ADD;
    collision_objects.push_back(wall2);
    
    // MIXER OBJECT
    
    moveit_msgs::msg::CollisionObject mixer;
    mixer.id = "mixer";
    mixer.header.frame_id = frame_id;
    mixer.primitives.resize(1);
    mixer.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    mixer.primitives[0].dimensions = {0.17, 0.06}; // {HEIGHT, RADIUS}
    mixer.primitive_poses.resize(1);
    mixer.primitive_poses[0].position.x = -0.32;
    mixer.primitive_poses[0].position.y = -0.15;
    mixer.primitive_poses[0].position.z = 0.06; 
    mixer.primitive_poses[0].orientation.w = 1.0;
    mixer.operation = mixer.ADD;
    collision_objects.push_back(mixer);
    
    // 6 DYNAMIC TEST TUBES (13cm height, 1.2cm diameter)
    // We dynamically generate them 4cm behind markers 1-6
    // 5 DYNAMIC TEST TUBES (13cm height, 1.2cm diameter)
    for (int i = 1; i <= 5; ++i) { // <--- CHANGED TO 5
        double tube_x = 0.0, tube_y = 0.0, tube_z = 0.055;
        bool tf_found = false;
        
        try {
            std::string target_frame = "marker_" + std::to_string(i);
            geometry_msgs::msg::TransformStamped t = tf_buffer.lookupTransform(
                "marker_base", target_frame, tf2::TimePointZero);
                
            tube_x = t.transform.translation.x;
            tube_y = t.transform.translation.y + 0.02; // Offset behind the marker
            tf_found = true;
        } catch (...) {
            std::cout << "    [-] TF for marker_" << i << " not ready yet. Using hardcoded fallback.\n";
        }

        if (!tf_found) {
            if (i == 1) { tube_x = -0.19; tube_y = -0.24; }
            else if (i == 2) { tube_x = -0.10; tube_y = -0.28; }
            else if (i == 3) { tube_x =  0.00; tube_y = -0.32; }
            else if (i == 4) { tube_x =  0.10; tube_y = -0.28; }
            else if (i == 5) { tube_x =  0.19; tube_y = -0.24; }
        }

        moveit_msgs::msg::CollisionObject tube;
        tube.id = "tube_" + std::to_string(i);
        tube.header.frame_id = frame_id;
        tube.primitives.resize(1);
        tube.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        tube.primitives[0].dimensions = {0.13, 0.012}; 
        tube.primitive_poses.resize(1);
        tube.primitive_poses[0].position.x = tube_x;
        tube.primitive_poses[0].position.y = tube_y;
        tube.primitive_poses[0].position.z = tube_z; 
        tube.primitive_poses[0].orientation.w = 1.0;
        tube.operation = tube.ADD;
        collision_objects.push_back(tube);
    }

    // DYNAMIC MIXER (Tied to marker_6)
    double mixer_x = -0.19, mixer_y = -0.06; // Fallback
    try {
        geometry_msgs::msg::TransformStamped t = tf_buffer.lookupTransform(
            "marker_base", "marker_6", tf2::TimePointZero);
        mixer_x = t.transform.translation.x;
        mixer_y = t.transform.translation.y; // Assuming mixer is centered right on the marker
    } catch (...) {}

    moveit_msgs::msg::CollisionObject mixer;
    mixer.id = "mixer";
    mixer.header.frame_id = frame_id;
    mixer.primitives.resize(1);
    mixer.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    mixer.primitives[0].dimensions = {0.17, 0.06}; // {HEIGHT, RADIUS}
    mixer.primitive_poses.resize(1);
    mixer.primitive_poses[0].position.x = mixer_x;
    mixer.primitive_poses[0].position.y = mixer_y;
    // Perfect Z for a 17cm cylinder sitting on a -0.01m table surface (-0.01 + 0.085)
    mixer.primitive_poses[0].position.z = 0.075; 
    mixer.primitive_poses[0].orientation.w = 1.0;
    mixer.operation = mixer.ADD;
    collision_objects.push_back(mixer);
    
    planning_scene_interface.applyCollisionObjects(collision_objects);
    std::cout << ">>> [INIT] Collision objects loaded into scene based on Aruco markers.\n";
}

void waitForStateSettle(
    moveit::planning_interface::MoveGroupInterface & iface,
    int ms = STATE_SETTLE_MS)
{
    rclcpp::sleep_for(std::chrono::milliseconds(ms));
    iface.startStateMonitor(1.0); 
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
    
    std::cout << "\n>>> [INIT] Waiting 2 seconds for TF tree to populate from camera...\n";
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // Pass the populated TF buffer into the setup function
    setupCollisionObjects(arm_interface.getPlanningFrame(), *tf_buffer);

    tf2::Quaternion q_upright;
    q_upright.setRPY(0.0, -M_PI / 2.0, M_PI / 2.0);

    std::cout << "\n>>> [INIT] Moving to start position with OMPL...\n";

    arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_TRANSIT);
    arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_TRANSIT);
    arm_interface.setPlanningPipelineId("ompl");
    arm_interface.setPlannerId("RRTConnectkConfigDefault");
    arm_interface.setPlanningTime(10.0);
    
    arm_interface.setNumPlanningAttempts(5);
    arm_interface.setWorkspace(-0.5, -0.5, -0.05, 0.5, 0.5, 0.6);
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

    arm_interface.setGoalPositionTolerance(0.01);
    arm_interface.setGoalOrientationTolerance(0.01);

    std::cout << "\n>>> Ready. Waiting for GUI commands on '/gui_commands'...\n";

    auto gui_sub = node->create_subscription<std_msgs::msg::String>(
        "/gui_commands", 10,
        [&](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(queue_mutex);
            command_queue.push(msg->data);
            std::cout << "[QUEUE] Received command: " << msg->data << " (Pending tasks: " << command_queue.size() << ")\n";
        }
    );

    std::string attached_tube = "";
    int current_marker_id = -1;
    
    // Atomic variable for thread-safe access inside the background timer
    std::atomic<int> attached_marker_id{-1};

    // ---------------------------------------------------------
    // DYNAMIC COLLISION UPDATER
    // Runs automatically in the background executor thread
    // ---------------------------------------------------------
    moveit::planning_interface::PlanningSceneInterface dynamic_scene_interface;
    auto dynamic_updater = node->create_wall_timer(
        std::chrono::milliseconds(500), 
        [&]() {
            std::vector<moveit_msgs::msg::CollisionObject> update_objects;
            int currently_held_id = attached_marker_id.load();

            // Update Tubes 1-5
            for (int i = 1; i <= 5; ++i) { // <--- CHANGED TO 5
                if (i == currently_held_id) continue;
                try {
                    std::string target_frame = "marker_" + std::to_string(i);
                    geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(
                        "marker_base", target_frame, tf2::TimePointZero);

                    moveit_msgs::msg::CollisionObject tube;
                    tube.id = "tube_" + std::to_string(i);
                    tube.header.frame_id = arm_interface.getPlanningFrame();
                    tube.primitives.resize(1);
                    tube.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                    tube.primitives[0].dimensions = {0.13, 0.012}; 
                    tube.primitive_poses.resize(1);
                    tube.primitive_poses[0].position.x = t.transform.translation.x;
                    tube.primitive_poses[0].position.y = t.transform.translation.y + 0.02;
                    tube.primitive_poses[0].position.z = 0.055;
                    tube.primitive_poses[0].orientation.w = 1.0;
                    tube.operation = tube.ADD; 
                    update_objects.push_back(tube);
                } catch (...) { }
            }

            // Update Mixer (Marker 6)
            try {
                geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(
                    "marker_base", "marker_6", tf2::TimePointZero);

                moveit_msgs::msg::CollisionObject dyn_mixer;
                dynamic_mixer.id = "mixer";
                dynamic_mixer.header.frame_id = arm_interface.getPlanningFrame();
                dynamic_mixer.primitives.resize(1);
                dynamic_mixer.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                dynamic_mixer.primitives[0].dimensions = {0.17, 0.06}; 
                dynamic_mixer.primitive_poses.resize(1);
                dynamic_mixer.primitive_poses[0].position.x = t.transform.translation.x;
                dynamic_mixer.primitive_poses[0].position.y = t.transform.translation.y;
                dynamic_mixer.primitive_poses[0].position.z = 0.075;
                dynamic_mixer.primitive_poses[0].orientation.w = 1.0;
                dynamic_mixer.operation = dynamic_mixer.ADD; 
                update_objects.push_back(dynamic_mixer);
            } catch (...) { }
            }

            if (!update_objects.empty()) {
                dynamic_scene_interface.applyCollisionObjects(update_objects);
            }
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
            if (gripper_interface.move() == moveit::core::MoveItErrorCode::SUCCESS) {
                // DETACH THE OBJECT
                if (!attached_tube.empty()) {
                    arm_interface.detachObject(attached_tube);
                    std::cout << ">>> Detached " << attached_tube << " from the arm. It is now a static obstacle.\n";
                    attached_tube = "";
                    attached_marker_id.store(-1); // Allow the dynamic updater to start tracking it again
                }
            }
            continue; 
        }
    
        if (line == "c" || line == "C") {
            std::cout << ">>> Closing Gripper...\n";
            
            // 1. ATTACH THE OBJECT FIRST so the Allowed Collision Matrix ignores the fingers
            if (current_marker_id >= 1 && current_marker_id <= 5 && attached_tube.empty()) {
                std::string tube_id = "tube_" + std::to_string(current_marker_id);
                
                std::vector<std::string> touch_links; 
                const moveit::core::JointModelGroup* gripper_jmg = gripper_interface.getRobotModel()->getJointModelGroup("gripper");
                if (gripper_jmg) {
                    touch_links = gripper_jmg->getLinkModelNames();
                }
                
                // Pause dynamic updates for this tube
                attached_marker_id.store(current_marker_id); 
                
                // This updates the ACM so the fingers can intersect the tube's geometry
                arm_interface.attachObject(tube_id, arm_interface.getEndEffectorLink(), touch_links);
                attached_tube = tube_id;
                std::cout << ">>> Attached " << tube_id << " to " << arm_interface.getEndEffectorLink() << "!\n";
                
                // Crucial: Give the Planning Scene a moment to publish and update the ACM
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }

            // 2. NOW CLOSE THE GRIPPER
            gripper_interface.setNamedTarget("closed");
            if (gripper_interface.move() == moveit::core::MoveItErrorCode::SUCCESS) {
                std::cout << ">>> Gripper closed safely around the tube.\n";
            } else {
                std::cout << "    [-] Gripper failed to close.\n";
                // If closing fails for another reason, detach it to reset the state
                if (!attached_tube.empty()) {
                    arm_interface.detachObject(attached_tube);
                    attached_tube = "";
                    attached_marker_id.store(-1);
                }
            }
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
                current_marker_id = marker_id;
                if (marker_id == 7) {
                    tx = 0.0; ty = -0.065; tz = 0.20;
                } else {
                    try {
                        std::string target_frame = "marker_" + std::to_string(marker_id);
                        geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(
                            "marker_base", target_frame, tf2::TimePointZero);
                            
                        // Target exactly 10cm behind the marker location
                        tx = t.transform.translation.x;
                        ty = t.transform.translation.y + 0.02; 
                        
                        // Set arm target height safely above the table/marker for grabbing
                        tz = 0.13; 
                    } catch (const tf2::TransformException & ex) {
                        std::cout << "    [-] Falling back to hardcoded positions\n";
                        if (marker_id == 1) { tx = -0.19; ty = -0.20; tz = 0.15; } 
                        else if (marker_id == 2) { tx = -0.10; ty = -0.24; tz = 0.15; } 
                        else if (marker_id == 3) { tx =  0.00; ty = -0.28; tz = 0.15; } 
                        else if (marker_id == 4) { tx =  0.10; ty = -0.24; tz = 0.15; } 
                        else if (marker_id == 5) { tx =  0.19; ty = -0.20; tz = 0.15; } 
                        else if (marker_id == 6) { tx = -0.19; ty = -0.02; tz = 0.20; } 
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

        // ---------------------------------------------------------
        // THE NEW, SINGLE-STEP OMPL MOVEMENT WITH RECOVERY LOGIC
        // ---------------------------------------------------------
        
        std::cout << "\n--- MOVING TO TARGET Z=" << tz << " ---\n";
        
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = tx;
        target_pose.position.y = ty;
        target_pose.position.z = tz;
        target_pose.orientation.x = q_upright.x();
        target_pose.orientation.y = q_upright.y();
        target_pose.orientation.z = q_upright.z();
        target_pose.orientation.w = q_upright.w();

        // Prepare the orientation constraint object for the mixer
        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = arm_interface.getEndEffectorLink();
        ocm.header.frame_id = arm_interface.getPlanningFrame();
        ocm.orientation = target_pose.orientation;
        ocm.absolute_x_axis_tolerance = M_PI; 
        ocm.absolute_y_axis_tolerance = 0.3;
        ocm.absolute_z_axis_tolerance = 0.3; 
        ocm.weight = 1.0;

        moveit_msgs::msg::Constraints constraints;
        constraints.orientation_constraints.push_back(ocm);

        // Calculate appropriate speeds
        double current_vel_scale = attached_tube.empty() ? VEL_SCALE_TRANSIT : VEL_SCALE_LIQUID;
        double current_acc_scale = attached_tube.empty() ? ACC_SCALE_TRANSIT : ACC_SCALE_LIQUID;
        
        arm_interface.setMaxVelocityScalingFactor(current_vel_scale);
        arm_interface.setMaxAccelerationScalingFactor(current_acc_scale);

        bool target_reached = false;

        // RETRY LOOP: 2 Attempts maximum
        for (int attempt = 1; attempt <= 2; ++attempt) {
            
            arm_interface.setPoseTarget(target_pose);

            // 1. APPLY ORIENTATION CONSTRAINTS ONLY FOR THE MIXER (MARKER 6)
            if (!attached_tube.empty() && current_marker_id == 6) {
                if (attempt == 1) std::cout << "    [*] Holding tube & moving to mixer: Applying upright constraints.\n";
                arm_interface.setPathConstraints(constraints);
            } else {
                if (attempt == 1) std::cout << "    [*] Moving freely without orientation constraints.\n";
                arm_interface.clearPathConstraints();
            }

            moveit::planning_interface::MoveGroupInterface::Plan unified_plan;
            
            if (arm_interface.plan(unified_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                std::cout << "    [+] Plan found (Attempt " << attempt << ")! Smoothing trajectory...\n";
                
                robot_trajectory::RobotTrajectory rt(arm_interface.getRobotModel(), arm_interface.getName());
                rt.setRobotTrajectoryMsg(*arm_interface.getCurrentState(), unified_plan.trajectory_);
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                totg.computeTimeStamps(rt, current_vel_scale, current_acc_scale);
                rt.getRobotTrajectoryMsg(unified_plan.trajectory_);

                if (arm_interface.execute(unified_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    waitForStateSettle(arm_interface);
                    std::cout << "\n>>> Target reached safely.\n";
                    target_reached = true;
                    break; // Success! Exit the retry loop.
                } else {
                    std::cout << "    [-] Execution failed on Attempt " << attempt << ".\n";
                }
            } else {
                std::cout << "    [-] Planning failed on Attempt " << attempt << ".\n";
            }

            // 2. RECOVERY MECHANISM (Only triggers if Attempt 1 fails)
            if (attempt == 1) {
                std::cout << "\n    [!] RECOVERY: Moving to Safe Position (-0.15, -0.15, 0.20) to reset arm posture...\n";
                
                // Temporarily clear constraints to guarantee we can untwist and reach the safe spot
                arm_interface.clearPathConstraints(); 
                arm_interface.setPoseTarget(start_pose);
                
                moveit::planning_interface::MoveGroupInterface::Plan recovery_plan;
                if (arm_interface.plan(recovery_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    
                    robot_trajectory::RobotTrajectory rt(arm_interface.getRobotModel(), arm_interface.getName());
                    rt.setRobotTrajectoryMsg(*arm_interface.getCurrentState(), recovery_plan.trajectory_);
                    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                    totg.computeTimeStamps(rt, current_vel_scale, current_acc_scale);
                    rt.getRobotTrajectoryMsg(recovery_plan.trajectory_);

                    if (arm_interface.execute(recovery_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                        waitForStateSettle(arm_interface);
                        std::cout << "    [*] Safe position reached. Retrying target destination...\n\n";
                    } else {
                        std::cout << "    [-] Recovery execution failed! Arm might be physically stuck.\n";
                        break; // If recovery fails, abort entirely
                    }
                } else {
                    std::cout << "    [-] Could not plan path to safe position! Arm is trapped.\n";
                    break; // If recovery fails, abort entirely
                }
            }
        }

        if (!target_reached) {
            std::cout << "    [!] CRITICAL: Target unreachable or in collision even after recovery attempt.\n";
        }
        
        // Clean up constraints after every move just to be safe
        arm_interface.clearPathConstraints();
    }

    std::cout << "\n>>> Shutting down.\n";
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
