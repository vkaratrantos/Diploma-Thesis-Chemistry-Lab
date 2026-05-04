// =============================================================================
// simple_move.cpp — Liquid Handler Pick & Place
// ROS 2 Humble | MoveIt 2 | Pilz Industrial Motion Planner
//
// Architecture: LIFT (Cartesian/OMPL) → MOVE (Pilz LIN) → DROP (Cartesian/OMPL)
// Fixes applied:
//   1. State settling delay after each execute()
//   2. computeCartesianPath for lift/drop (straight-line + fraction check)
//   3. OMPL fallback when Cartesian path fraction is too low
//   4. Pilz LIN used ONLY for horizontal transfer (Phase 2)
//   5. Aggressive SAFE_Z margin
//   6. Retry loop with incremental Z for lift failures
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
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>

// =============================================================================
// CONSTANTS
// =============================================================================
static constexpr double VEL_SCALE_TRANSIT  = 0.10; // Speed for OMPL moves
static constexpr double VEL_SCALE_LIQUID   = 0.05; // Speed while carrying liquid
static constexpr double ACC_SCALE_TRANSIT  = 0.10;
static constexpr double ACC_SCALE_LIQUID   = 0.05;

static constexpr double CARTESIAN_EEF_STEP = 0.005; // 5 mm interpolation step
static constexpr double CARTESIAN_JUMP_THR = 0.0;   // Disable jump threshold

static constexpr double MIN_CARTESIAN_FRACTION = 0.95; // 95 % path completeness

static constexpr double SAFE_Z_DEFAULT     = 0.3;  // Default transit height [m]
static constexpr double SAFE_Z_MARGIN      = 0.12;  // Extra headroom above target
static constexpr double SAFE_Z_RETRY_STEP  = 0.03;  // Increment when lift fails
static constexpr double SAFE_Z_MAX_RETRY   = 0.15;  // Max extra Z to try

static constexpr int    STATE_SETTLE_MS    = 500;   // Wait after execute() [ms]

// =============================================================================
// HELPER: wait for robot state to settle after an execute() call
// =============================================================================
void waitForStateSettle(
    moveit::planning_interface::MoveGroupInterface & iface,
    int ms = STATE_SETTLE_MS)
{
    rclcpp::sleep_for(std::chrono::milliseconds(ms));
    iface.startStateMonitor(1.0); // force a fresh state read
}

// =============================================================================
// HELPER: attempt a straight Cartesian move; fall back to OMPL if needed.
//
//   Returns true on success, false if both methods fail.
//   The function temporarily switches planners as required and restores them.
// =============================================================================
bool cartesianMoveWithFallback(
    moveit::planning_interface::MoveGroupInterface & iface,
    const geometry_msgs::msg::Pose                & target,
    const std::string                             & phase_name)
{
    // --- Try Cartesian path first ---
    std::vector<geometry_msgs::msg::Pose> waypoints = {target};
    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = iface.computeCartesianPath(
        waypoints, CARTESIAN_EEF_STEP, CARTESIAN_JUMP_THR, trajectory);

    if (fraction >= MIN_CARTESIAN_FRACTION) {
        std::cout << "    [" << phase_name << "] Cartesian path: "
                  << static_cast<int>(fraction * 100) << "% complete. Executing...\n";
        auto result = iface.execute(trajectory);
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            return true;
        }
        std::cout << "    [" << phase_name << "] Cartesian execute failed. Falling back to OMPL...\n";
    } else {
        std::cout << "    [" << phase_name << "] Cartesian coverage too low ("
                  << static_cast<int>(fraction * 100) << "%). Falling back to OMPL...\n";
    }

    // --- OMPL fallback ---
    iface.setPlanningPipelineId("ompl");
    iface.setPlannerId("RRTConnectkConfigDefault");
    iface.setPlanningTime(5.0);

    iface.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (iface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        std::cout << "    [" << phase_name << "] OMPL plan found. Executing...\n";
        auto result = iface.execute(plan);
        return (result == moveit::core::MoveItErrorCode::SUCCESS);
    }

    std::cout << "[-] [" << phase_name << "] Both Cartesian and OMPL failed.\n";
    return false;
}

// =============================================================================
// HELPER: find IK-valid overhead pose by scanning roll angles
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

    // Radial scan: try 0° first, then ±1°, ±2°, ..., ±180°
    std::vector<int> roll_angles = {0};
    for (int i = 1; i <= 180; ++i) {
        roll_angles.push_back(i);
        roll_angles.push_back(-i);
    }

    for (int angle : roll_angles) {
        double roll = angle * (M_PI / 180.0);
        tf2::Quaternion q_rot;
        q_rot.setRotation(tf2::Vector3(1, 0, 0), roll);
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
            std::cout << "    Found valid IK with roll=" << angle << " deg.\n";
            return true;
        }
    }
    return false;
}

// =============================================================================
// MAIN
// =============================================================================
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

    // "Upright" orientation: tool pointing straight down
    tf2::Quaternion q_upright;
    q_upright.setRPY(0.0, -M_PI / 2.0, M_PI / 2.0);

    // =========================================================================
    // 2. INITIAL MOTION TO START POINT  (OMPL — no liquid yet)
    // =========================================================================
    std::cout << "\n>>> [INIT] Moving to start position with OMPL...\n";

    arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_TRANSIT);
    arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_TRANSIT);

    arm_interface.setPlanningPipelineId("ompl");
    arm_interface.setPlannerId("RRTConnectkConfigDefault");
    arm_interface.setPlanningTime(5.0);

    // Relaxed tolerances so OMPL finds a solution quickly
    arm_interface.setGoalPositionTolerance(0.01);
    arm_interface.setGoalOrientationTolerance(0.05);

    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x    = -0.15;
    start_pose.position.y    = -0.15;
    start_pose.position.z    =  0.20; // well above base to avoid self-collision
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

    // Tighten tolerances for precision liquid handling
    arm_interface.setGoalPositionTolerance(0.001);
    arm_interface.setGoalOrientationTolerance(0.001);

    // =========================================================================
    // 3. INTERACTIVE LOOP  (LIFT → MOVE → DROP)
    // =========================================================================
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

        // Compute safe transit height — always well above the target
        double safe_z = SAFE_Z_DEFAULT;
        if (tz + SAFE_Z_MARGIN > safe_z) {
            safe_z = tz + SAFE_Z_MARGIN;
        }

        // =================================================================
        // PHASE 1: LIFT  — straight up to safe_z
        // Planner: Cartesian path (preferred) with OMPL fallback
        // Speed:   transit (no liquid yet / liquid secured vertically)
        // =================================================================
        std::cout << "\n--- [PHASE 1] LIFT to Z=" << safe_z << " ---\n";

        arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
        arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

        geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
        geometry_msgs::msg::Pose lift_pose    = current_pose;

        bool lifted = false;

        // Retry with increasing Z if the straight-up path is blocked
        for (double z_try = safe_z;
             z_try <= safe_z + SAFE_Z_MAX_RETRY && !lifted;
             z_try += SAFE_Z_RETRY_STEP)
        {
            lift_pose.position.z = z_try;

            if (cartesianMoveWithFallback(arm_interface, lift_pose, "LIFT")) {
                safe_z = z_try; // remember the height that actually worked
                lifted = true;
                waitForStateSettle(arm_interface);
            } else {
                std::cout << "    Retrying with Z=" << (z_try + SAFE_Z_RETRY_STEP) << "...\n";
            }
        }

        if (!lifted) {
            std::cout << "[-] [PHASE 1] Cannot lift. Skipping this target.\n";
            continue;
        }

        // =================================================================
        // PHASE 2: MOVE  — horizontal transfer at safe_z
        // Planner: Pilz LIN (guarantees straight-line Cartesian motion)
        // Speed:   slow (carrying liquid)
        // =================================================================
        std::cout << "\n--- [PHASE 2] HORIZONTAL MOVE to (" << tx << ", " << ty << ") ---\n";

        arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
        arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

        // Switch to Pilz LIN for guaranteed straight-line motion
        arm_interface.setPlanningPipelineId("pilz_industrial_motion_planner");
        arm_interface.setPlannerId("LIN");
        arm_interface.setPlanningTime(2.0);

        geometry_msgs::msg::Pose overhead_pose;
        if (!findValidOverheadPose(arm_interface, tx, ty, safe_z, q_upright, overhead_pose)) {
            std::cout << "[-] [PHASE 2] Target X,Y is unreachable at safe height.\n";
            continue;
        }

        arm_interface.setPoseTarget(overhead_pose);
        MoveGroupInterface::Plan plan_move;

        bool moved = false;
        if (arm_interface.plan(plan_move) == moveit::core::MoveItErrorCode::SUCCESS) {
            if (arm_interface.execute(plan_move) == moveit::core::MoveItErrorCode::SUCCESS) {
                moved = true;
                waitForStateSettle(arm_interface);
            }
        }

        if (!moved) {
            // LIN failed: fall back to OMPL for the horizontal leg
            std::cout << "    [PHASE 2] Pilz LIN failed. Falling back to OMPL...\n";
            arm_interface.setPlanningPipelineId("ompl");
            arm_interface.setPlannerId("RRTConnectkConfigDefault");
            arm_interface.setPlanningTime(5.0);
            arm_interface.setPoseTarget(overhead_pose);

            MoveGroupInterface::Plan plan_ompl;
            if (arm_interface.plan(plan_ompl) == moveit::core::MoveItErrorCode::SUCCESS) {
                if (arm_interface.execute(plan_ompl) == moveit::core::MoveItErrorCode::SUCCESS) {
                    moved = true;
                    waitForStateSettle(arm_interface);
                }
            }
        }

        if (!moved) {
            std::cout << "[-] [PHASE 2] Both LIN and OMPL failed. Skipping.\n";
            continue;
        }

        // =================================================================
        // PHASE 3: DROP  — straight down to target Z
        // Planner: Cartesian path (preferred) with OMPL fallback
        // Speed:   slow (liquid in transit)
        // =================================================================
        std::cout << "\n--- [PHASE 3] DROP to Z=" << tz << " ---\n";

        arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
        arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

        // Build the drop target from the current (settled) pose so X,Y,orientation
        // exactly match what Phase 2 left us at — only Z changes.
        geometry_msgs::msg::Pose drop_pose = arm_interface.getCurrentPose().pose;
        drop_pose.position.z = tz;

        if (cartesianMoveWithFallback(arm_interface, drop_pose, "DROP")) {
            waitForStateSettle(arm_interface);
            std::cout << "\n>>> Sequence complete! Target reached safely.\n";
        } else {
            std::cout << "[-] [PHASE 3] Cannot drop. Check for collisions or joint limits.\n";
        }
    }

    // =========================================================================
    // 4. SHUTDOWN
    // =========================================================================
    std::cout << "\n>>> Shutting down. Goodbye!\n";
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
