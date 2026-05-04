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
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char * argv[])
{
    // =========================================================================
    // 1. SYSTEM INITIALIZATION
    // =========================================================================
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("liquid_handler_master");

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    std::thread spinner([executor]() { executor->spin(); });

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface arm_interface(node, "arm_group"); 

    // Χαμηλές ταχύτητες - Απαραίτητο για το Liquid Handling και τον Pilz
    arm_interface.setMaxVelocityScalingFactor(0.1);
    arm_interface.setMaxAccelerationScalingFactor(0.1);

    // =========================================================================
    // 2. INITIAL MOTION TO START POINT (OMPL PLANNER)
    // =========================================================================
    std::cout << ">>> Moving to start position using OMPL..." << std::endl;
    arm_interface.clearPoseTargets();
    arm_interface.clearPathConstraints();

    arm_interface.setPlanningPipelineId("ompl");
    arm_interface.setPlannerId("RRTConnectkConfigDefault");
    arm_interface.setPlanningTime(5.0); 
    
    // [ΝΕΟ] Δίνουμε λίγο μεγαλύτερη ανοχή (tolerance) για να βρει πιο εύκολα λύση
    arm_interface.setGoalPositionTolerance(0.01);    // 1cm ανοχή στη θέση
    arm_interface.setGoalOrientationTolerance(0.05); // Ελαφριά ανοχή στον προσανατολισμό

    geometry_msgs::msg::Pose start_pose;
    // Αλλάζουμε λίγο το Z για να είμαστε σίγουροι ότι είναι μακριά από τη βάση (αποφυγή self-collision)
    start_pose.position.x = -0.15;
    start_pose.position.y = -0.15;
    start_pose.position.z = 0.15; // Σηκώσαμε το Z στα 20cm 

    // The universal "Upright" mathematical state
    tf2::Quaternion q_upright;
    q_upright.setRPY(0.0, -M_PI/2.0, M_PI/2.0); 
    start_pose.orientation.x = q_upright.x();
    start_pose.orientation.y = q_upright.y();
    start_pose.orientation.z = q_upright.z();
    start_pose.orientation.w = q_upright.w();

    arm_interface.setPoseTarget(start_pose);
    MoveGroupInterface::Plan initial_plan;
    
    std::cout << ">>> Planning initial pose..." << std::endl;
    if (arm_interface.plan(initial_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        std::cout << ">>> Initial plan found! Executing..." << std::endl;
        arm_interface.execute(initial_plan);
    } else {
        std::cout << "[-] Failed to reach start position with OMPL!" << std::endl;
        rclcpp::shutdown();
        spinner.join();
        return -1;
    }
    
    // [ΝΕΟ] Επαναφέρουμε τις ανοχές στο μηδέν για τον Pilz (ώστε το υγρό να μην κουνηθεί!)
    arm_interface.setGoalPositionTolerance(0.001);   // 1mm
    arm_interface.setGoalOrientationTolerance(0.001);

    // =========================================================================
    // 3. PILZ PIPELINE SETUP (SWITCH TO LINEAR MOTION)
    // =========================================================================
    std::cout << ">>> Switching to Pilz Industrial Motion Planner..." << std::endl;
    // Αλλάζουμε το pipeline αποκλειστικά σε Pilz για τις γραμμικές κινήσεις
    arm_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_interface.setPlannerId("LIN"); // Γραμμική Κίνηση (Linear)
    arm_interface.setPlanningTime(2.0); 

// =========================================================================
    // 4. INTERACTIVE LOOP (LIFT - MOVE - DROP ARCHITECTURE)
    // =========================================================================

    // Χαμηλές και σταθερές ταχύτητες για να προλαβαίνει ο καρπός να στρίψει
    arm_interface.setMaxVelocityScalingFactor(0.05); 
    arm_interface.setMaxAccelerationScalingFactor(0.05);

    while (rclcpp::ok()) {
        std::cout << "\nEnter Final Target <X Y Z> : ";
        std::string line;
        std::getline(std::cin, line);
        if (line == "q" || line == "Q") break;

        std::stringstream ss(line);
        double tx, ty, tz;
        if (!(ss >> tx >> ty >> tz)) continue;

        double SAFE_Z = 0.33; // Το "ευέλικτο" ύψος που ανακαλύψαμε
        if (tz > SAFE_Z) SAFE_Z = tz + 0.05; // Αν του ζητήσεις να πάει πιο ψηλά από το 0.30, αναπροσαρμόζεται

        // ---------------------------------------------------------------------
        // ΦΑΣΗ 1: LIFT (Κάθετη Ανύψωση)
        // ---------------------------------------------------------------------
        std::cout << "\n[PHASE 1] Lifting straight up to safe height (Z=" << SAFE_Z << ")..." << std::endl;
        geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
        geometry_msgs::msg::Pose lift_pose = current_pose;
        lift_pose.position.z = SAFE_Z;
        
        arm_interface.setPoseTarget(lift_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan_lift;
        
        if (arm_interface.plan(plan_lift) == moveit::core::MoveItErrorCode::SUCCESS) {
            arm_interface.execute(plan_lift);
        } else {
            std::cout << "[-] Phase 1 Failed: Cannot lift straight up (Limit reached?)" << std::endl;
            continue; // Επιστροφή στην αναμονή νέου input
        }

        // ---------------------------------------------------------------------
        // ΦΑΣΗ 2: MOVE (Οριζόντια Μεταφορά + Smart Roll)
        // ---------------------------------------------------------------------
        std::cout << "[PHASE 2] Moving horizontally. Calculating optimal Roll..." << std::endl;
        
        // Παίρνουμε το νέο state ΤΩΡΑ που είμαστε ψηλά
        moveit::core::RobotStatePtr kinematic_state = arm_interface.getCurrentState();
        const moveit::core::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup("arm_group");
        
        bool found_valid_target = false;
        geometry_msgs::msg::Pose overhead_pose;

        // Ακτινωτή αναζήτηση για την ελάχιστη δυνατή περιστροφή
        std::vector<int> roll_angles;
        roll_angles.push_back(0); 
        for (int i = 0; i <= 180; i += 1) {
            roll_angles.push_back(i);
            roll_angles.push_back(-i);
        }

        for (int angle : roll_angles) {
            double roll = angle * (M_PI / 180.0);
            tf2::Quaternion q_rot;
            q_rot.setRotation(tf2::Vector3(1, 0, 0), roll); 
            tf2::Quaternion q_final = q_upright * q_rot;
            q_final.normalize();

            geometry_msgs::msg::Pose test_pose;
            test_pose.position.x = tx;
            test_pose.position.y = ty;
            test_pose.position.z = SAFE_Z; // Παραμένουμε στο ασφαλές ύψος
            test_pose.orientation.x = q_final.x();
            test_pose.orientation.y = q_final.y();
            test_pose.orientation.z = q_final.z();
            test_pose.orientation.w = q_final.w();

            if (kinematic_state->setFromIK(joint_model_group, test_pose, 0.05)) {
                overhead_pose = test_pose;
                found_valid_target = true;
                std::cout << "          Found valid path! Minimal Roll required: " << angle << " degrees." << std::endl;
                break; 
            }
        }

        if (!found_valid_target) {
            std::cout << "[-] Phase 2 Failed: Target X,Y is out of reach even at safe height." << std::endl;
            continue;
        }

        arm_interface.setPoseTarget(overhead_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan_move;
        
        if (arm_interface.plan(plan_move) == moveit::core::MoveItErrorCode::SUCCESS) {
            arm_interface.execute(plan_move);
        } else {
            std::cout << "[-] Phase 2 Failed: Could not draw a straight line to overhead target." << std::endl;
            continue;
        }

        // ---------------------------------------------------------------------
        // ΦΑΣΗ 3: DROP (Κάθετη Κάθοδος)
        // ---------------------------------------------------------------------
        std::cout << "[PHASE 3] Dropping straight down to final target (Z=" << tz << ")..." << std::endl;
        
        // Παίρνουμε την ΤΡΕΧΟΥΣΑ στάση (που έχει ήδη το σωστό X,Y και το σωστό Roll από τη Φάση 2)
        geometry_msgs::msg::Pose drop_pose = arm_interface.getCurrentPose().pose; 
        drop_pose.position.z = tz; // Το μόνο που αλλάζουμε είναι να κατέβουμε!

        arm_interface.setPoseTarget(drop_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan_drop;
        
        if (arm_interface.plan(plan_drop) == moveit::core::MoveItErrorCode::SUCCESS) {
            arm_interface.execute(plan_drop);
            std::cout << ">>> Sequence Complete! Glass delivered safely.\n" << std::endl;
        } else {
            std::cout << "[-] Phase 3 Failed: Cannot drop down (Collision or Limit)." << std::endl;
        }
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
