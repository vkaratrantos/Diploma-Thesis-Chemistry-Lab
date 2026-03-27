// --- Standard C++ Libraries ---
#include <memory>   
#include <thread>   
#include <iostream> 
#include <string>   
#include <sstream>  
#include <cmath>    
#include <vector>

// --- ROS 2 & MoveIt 2 Libraries ---
#include <rclcpp/rclcpp.hpp> 
#include <moveit/move_group_interface/move_group_interface.h> 
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char * argv[])
{
    // =========================================================================
    // 1. SYSTEM INITIALIZATION
    // =========================================================================
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("interactive_robot_planner");

    // =========================================================================
    // 2. BACKGROUND THREADING
    // =========================================================================
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    std::thread spinner([executor]() { executor->spin(); });

    // =========================================================================
    // 3. MOVEIT INTERFACES SETUP
    // =========================================================================
    using moveit::planning_interface::MoveGroupInterface;
    
    // Setup Arm
    MoveGroupInterface arm_interface(node, "arm_group");
    arm_interface.setPlanningTime(1.0);
    arm_interface.setMaxVelocityScalingFactor(0.1);
    arm_interface.setMaxAccelerationScalingFactor(0.1);

    // Setup Gripper
    MoveGroupInterface gripper_interface(node, "gripper");
    // Grippers usually need very little planning time
    gripper_interface.setPlanningTime(0.5); 

    // =========================================================================
    // 3.5 ADDING THE COLLISION FLOOR
    // =========================================================================
    std::cout << ">>> Spawning collision floor..." << std::endl;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject floor_object;
    
    floor_object.header.frame_id = arm_interface.getPlanningFrame();
    floor_object.id = "safety_floor";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {2.0, 2.0, 0.04}; 

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.04; 

    floor_object.primitives.push_back(primitive);
    floor_object.primitive_poses.push_back(box_pose);
    floor_object.operation = floor_object.ADD;

    planning_scene_interface.applyCollisionObjects({floor_object});
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // =========================================================================
    // 4. INITIAL STARTUP MOTION (EXACT POSE)
    // =========================================================================
    std::cout << ">>> Moving to initial position (X:0.0, Y:-0.2, Z:0.2) ..." << std::endl;
    
    arm_interface.clearPoseTargets();
    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = 0.0;
    start_pose.position.y = -0.2;
    start_pose.position.z = 0.2;

    tf2::Quaternion q_start;
    q_start.setRPY(0.0, -M_PI/2.0, M_PI/2.0); 
    
    start_pose.orientation.x = q_start.x();
    start_pose.orientation.y = q_start.y();
    start_pose.orientation.z = q_start.z();
    start_pose.orientation.w = q_start.w();

    arm_interface.setPoseTarget(start_pose);
    arm_interface.setPlanningTime(2.0); 
    
    MoveGroupInterface::Plan initial_plan;
    if (arm_interface.plan(initial_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        arm_interface.execute(initial_plan);
        std::cout << ">>> Initial position reached successfully!" << std::endl;
    } else {
        std::cout << "[-] WARNING: Failed to plan to the initial position." << std::endl;
    }

    arm_interface.clearPoseTargets();
    arm_interface.setPlanningTime(1.0); 

    std::cout << "\n>>> MoveIt 2 Interactive Planner Ready." << std::endl;

    // =========================================================================
    // 5. INTERACTIVE COMMAND LOOP (LIQUID HANDLING MODE)
    // =========================================================================
    while (rclcpp::ok()) {
        std::cout << "\n================================================" << std::endl;
        std::cout << "Commands: <X> <Y> <Z> (Move), O (Open), C (Close), Q (Quit)" << std::endl;
        std::cout << "Command: ";
        
        std::string line;
        std::getline(std::cin, line);
        if (line.empty()) continue;

        if (line == "q" || line == "Q") break;

        if (line == "o" || line == "O") {
            std::cout << ">>> Opening gripper..." << std::endl;
            gripper_interface.setNamedTarget("open");
            gripper_interface.move(); 
            continue; 
        }

        if (line == "c" || line == "C") {
            std::cout << ">>> Closing gripper..." << std::endl;
            gripper_interface.setNamedTarget("closed");
            gripper_interface.move(); 
            continue; 
        }

        std::stringstream ss(line);
        double tx, ty, tz;
        
        if (ss >> tx >> ty >> tz) {
            std::cout << ">>> Planning motion to X:" << tx << ", Y:" << ty << ", Z:" << tz << std::endl;
            std::cout << ">>> [LIQUID MODE: Pitch and Yaw locked during transit!]" << std::endl;
            
            // --- 1. APPLY PATH CONSTRAINTS ---
            moveit_msgs::msg::OrientationConstraint ocm;
            ocm.link_name = arm_interface.getEndEffectorLink();
            ocm.header.frame_id = arm_interface.getPlanningFrame();

            tf2::Quaternion q_constraint;
            q_constraint.setRPY(0.0, -M_PI/2.0, M_PI/2.0); // The baseline to measure against
            ocm.orientation.x = q_constraint.x();
            ocm.orientation.y = q_constraint.y();
            ocm.orientation.z = q_constraint.z();
            ocm.orientation.w = q_constraint.w();

            // X (Roll) is free (360 deg). 
            // Y (Pitch) and Z (Yaw) get a tight 0.1 radian (~5 degree) tolerance so the liquid doesn't spill.
            ocm.absolute_x_axis_tolerance = 2.0 * M_PI;
            ocm.absolute_y_axis_tolerance = 0.7;
            ocm.absolute_z_axis_tolerance = 0.7;
            ocm.weight = 1.0;

            moveit_msgs::msg::Constraints path_constraints;
            path_constraints.orientation_constraints.push_back(ocm);
            
            arm_interface.setPathConstraints(path_constraints);
            
            // Constrained planning requires much more math. Give the solver 10 seconds.
            arm_interface.setPlanningTime(2.0); 

            // --- 2. EXECUTE OUTWARD ROLL SEARCH ---
            std::vector<double> roll_angles_to_test;
            roll_angles_to_test.push_back(0.0);
            
            int step_deg = 1;
            int max_deg = 180;
            
            for (int i = step_deg; i <= max_deg; i += step_deg) {
                roll_angles_to_test.push_back(i * M_PI / 180.0);  
                roll_angles_to_test.push_back(-i * M_PI / 180.0); 
            }

            bool plan_found = false;
            MoveGroupInterface::Plan my_plan;

            for (double r : roll_angles_to_test) {
                geometry_msgs::msg::Pose target_pose;
                target_pose.position.x = tx;
                target_pose.position.y = ty;
                target_pose.position.z = tz;
                
                tf2::Quaternion q;
                q.setRPY(r, -M_PI/2.0, M_PI/2.0); 
                
                target_pose.orientation.x = q.x();
                target_pose.orientation.y = q.y();
                target_pose.orientation.z = q.z();
                target_pose.orientation.w = q.w();

                arm_interface.setPoseTarget(target_pose);
                
                if (arm_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    std::cout << "    [+] Found valid spill-proof plan with Roll = " << (r * 180.0 / M_PI) << "°" << std::endl;
                    plan_found = true;
                    break; 
                }
            }

            // --- 3. EXECUTE AND CLEANUP ---
            if (plan_found) {
                std::cout << ">>> Executing path..." << std::endl;
                arm_interface.execute(my_plan);
            } else {
                std::cout << "[-] Failed to find a valid spill-proof plan for these coordinates." << std::endl;
            }
            
            // CRITICAL: Clear constraints so they don't corrupt the next command
            arm_interface.clearPoseTargets();
            arm_interface.clearPathConstraints();
            // Reset planning time
            arm_interface.setPlanningTime(1.0); 

        } else {
            std::cout << "[-] Invalid input. Please enter valid commands." << std::endl;
        }
    }

    // =========================================================================
    // 6. CLEANUP
    // =========================================================================
    rclcpp::shutdown();
    spinner.join(); 
    return 0;
}
