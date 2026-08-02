// ============================================================================
//  LIQUID HANDLING 7-DOF ROBOTIC ARM
//  MoveIt 2 + MoveIt Task Constructor
//
//  Commands (published as std_msgs/String on /gui_commands):
//    TASK <1-5>   full MTC pick-and-carry pipeline for that tube
//    m<0-6>       manual hybrid move to a marker (OMPL transit + Cartesian descent)
//    <x> <y> <z>  manual hybrid move to raw coordinates
//    o            open gripper (detaches held tube)
//    c            close gripper (attaches tube at current marker)
//    p            pour
//    h            return to home pose
//    q            quit
// ============================================================================

// ---- BUILD NOTE ------------------------------------------------------------
// Uncomment the following line if you are building against MoveIt 2 Jazzy or
// newer, where computeCartesianPath() takes MaxEEFStep / JumpThreshold structs
// instead of plain doubles.
//
// #define MOVEIT_JAZZY_OR_NEWER 1
// ----------------------------------------------------------------------------

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // tf2::toMsg
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// ---- MoveIt Task Constructor ----------------------------------------------
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>

namespace mtc = moveit::task_constructor;
using MGI = moveit::planning_interface::MoveGroupInterface;

// ============================================================================
//  CONSTANTS
//  Every magic number lives here. In the original file the tube height and the
//  mixer radius were defined differently in setupCollisionObjects() and in the
//  dynamic updater, so the scene silently changed shape 500 ms after startup.
// ============================================================================

// ---- Speed scaling ---------------------------------------------------------
static constexpr double VEL_SCALE_TRANSIT = 0.30;
static constexpr double ACC_SCALE_TRANSIT = 0.10;
static constexpr double VEL_SCALE_LIQUID  = 0.15;
static constexpr double ACC_SCALE_LIQUID  = 0.05;

// ---- Table -----------------------------------------------------------------
static constexpr double TABLE_SIZE_XY   = 1.5;
static constexpr double TABLE_THICKNESS = 0.04;
static constexpr double TABLE_CENTER_Z  = -0.03;

// ---- Test tubes ------------------------------------------------------------
static constexpr double TUBE_HEIGHT = 0.13;
static constexpr double TUBE_RADIUS = 0.012;
// NOTE: geometrically this should be table_top + height/2 = -0.01 + 0.065 = 0.055.
// Your original code used 0.07 at init and 0.08 in the updater. 0.08 is kept
// here to preserve the behaviour you tuned against, but verify it in RViz --
// if it is wrong, the tube collision cylinder floats 2.5 cm above the table
// and the gripper can clip the real tube without the planner noticing.
static constexpr double TUBE_CENTER_Z = 0.08;
static constexpr double TUBE_Y_OFFSET = 0.02;       // marker -> tube body, in y

// ---- Mixer -----------------------------------------------------------------
static constexpr double MIXER_HEIGHT   = 0.17;
static constexpr double MIXER_RADIUS   = 0.06;      // updater used 0.05; unified
static constexpr double MIXER_CENTER_Z = 0.08;      // updater used 0.07; unified
static constexpr double MIXER_Y_OFFSET = 0.10;

// ---- Grasp geometry --------------------------------------------------------
static constexpr double GRASP_Y_OFFSET = 0.155;     // marker -> TCP, in y
static constexpr double GRASP_Z        = 0.11;      // TCP height at the tube
static constexpr double POUR_Z         = 0.20;      // TCP height above the mixer
static constexpr double APPROACH_DIST  = 0.10;      // vertical descent to grasp
static constexpr double LIFT_DIST      = 0.12;      // vertical retreat after grasp
static constexpr double STANDOFF_TUBE  = 0.10;
static constexpr double STANDOFF_MIXER = 0.12;

// ---- Misc ------------------------------------------------------------------
static constexpr int NUM_TUBES       = 5;
static constexpr int MIXER_MARKER    = 6;
static constexpr int STATE_SETTLE_MS = 500;

// ============================================================================
//  SHARED STATE
// ============================================================================

static std::queue<std::string> command_queue;
static std::mutex              queue_mutex;

// Which tube (if any) is currently on the gripper. The background updater skips
// republishing this one, otherwise a world copy of the tube would fight the
// attached copy.
static std::atomic<int> attached_marker_id{ -1 };

// Hard pause for the background scene updater. MTC plans the whole pipeline
// offline against a scene snapshot; if the updater keeps rewriting collision
// objects mid-plan, MTC is planning against a moving target.
static std::atomic<bool> scene_updates_paused{ false };

// RAII: pause scene updates for the lifetime of the guard.
class ScenePause
{
public:
    ScenePause() { scene_updates_paused.store(true); }
    ~ScenePause() { scene_updates_paused.store(false); }
};

// ============================================================================
//  SCENE HELPERS
// ============================================================================

// Look up a marker's XY position expressed in the planning frame.
//
// The original code looked up "marker_base" -> "marker_N" but then stamped the
// resulting CollisionObject with getPlanningFrame(). That is only correct if
// those two frames coincide. Asking TF for the transform directly into the
// planning frame is always correct, and fails loudly if the frames are not
// connected -- which is what you want.
static bool lookupMarkerXY(tf2_ros::Buffer & tf_buffer,
                           const std::string & planning_frame,
                           int marker_id,
                           double & x,
                           double & y)
{
    try
    {
        const std::string marker_frame = "marker_" + std::to_string(marker_id);
        const auto t = tf_buffer.lookupTransform(planning_frame, marker_frame, tf2::TimePointZero);
        x = t.transform.translation.x;
        y = t.transform.translation.y;
        return true;
    }
    catch (const tf2::TransformException &)
    {
        return false;
    }
}

static moveit_msgs::msg::CollisionObject makeTube(int index,
                                                  const std::string & frame_id,
                                                  double marker_x,
                                                  double marker_y)
{
    moveit_msgs::msg::CollisionObject tube;
    tube.id = "tube_" + std::to_string(index);
    tube.header.frame_id = frame_id;
    tube.primitives.resize(1);
    tube.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    tube.primitives[0].dimensions = { TUBE_HEIGHT, TUBE_RADIUS };
    tube.primitive_poses.resize(1);
    tube.primitive_poses[0].position.x = marker_x;
    tube.primitive_poses[0].position.y = marker_y + TUBE_Y_OFFSET;
    tube.primitive_poses[0].position.z = TUBE_CENTER_Z;
    tube.primitive_poses[0].orientation.w = 1.0;
    tube.operation = tube.ADD;
    return tube;
}

static moveit_msgs::msg::CollisionObject makeMixer(const std::string & frame_id,
                                                   double marker_x,
                                                   double marker_y)
{
    moveit_msgs::msg::CollisionObject mixer;
    mixer.id = "mixer";
    mixer.header.frame_id = frame_id;
    mixer.primitives.resize(1);
    mixer.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    mixer.primitives[0].dimensions = { MIXER_HEIGHT, MIXER_RADIUS };
    mixer.primitive_poses.resize(1);
    mixer.primitive_poses[0].position.x = marker_x;
    mixer.primitive_poses[0].position.y = marker_y + MIXER_Y_OFFSET;
    mixer.primitive_poses[0].position.z = MIXER_CENTER_Z;
    mixer.primitive_poses[0].orientation.w = 1.0;
    mixer.operation = mixer.ADD;
    return mixer;
}

static void setupCollisionObjects(const std::string & frame_id, tf2_ros::Buffer & tf_buffer)
{
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    // ---- Table -------------------------------------------------------------
    moveit_msgs::msg::CollisionObject table;
    table.id = "table_base";
    table.header.frame_id = frame_id;
    table.primitives.resize(1);
    table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    table.primitives[0].dimensions = { TABLE_SIZE_XY, TABLE_SIZE_XY, TABLE_THICKNESS };
    table.primitive_poses.resize(1);
    table.primitive_poses[0].position.z = TABLE_CENTER_Z;
    table.primitive_poses[0].orientation.w = 1.0;
    table.operation = table.ADD;
    objects.push_back(table);

    // ---- Static obstacle 1 -------------------------------------------------
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
    objects.push_back(wall);

    // ---- Static obstacle 2 -------------------------------------------------
    moveit_msgs::msg::CollisionObject wall2;
    wall2.id = "obstacle_box_2";
    wall2.header.frame_id = frame_id;
    wall2.primitives.resize(1);
    wall2.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    wall2.primitives[0].dimensions = { 0.4, 0.2, 0.3 };
    wall2.primitive_poses.resize(1);
    wall2.primitive_poses[0].position.x = -0.6;
    wall2.primitive_poses[0].position.y = 0.1;
    wall2.primitive_poses[0].position.z = 0.03;
    wall2.primitive_poses[0].orientation.w = 1.0;
    wall2.operation = wall2.ADD;
    objects.push_back(wall2);

    // ---- Tubes -------------------------------------------------------------
    // Fallback positions used only if the camera has not published TF yet.
    static const double FALLBACK_X[NUM_TUBES] = { -0.17, -0.085, 0.0, 0.085, 0.17 };
    static constexpr double FALLBACK_Y = -0.345;

    for (int i = 1; i <= NUM_TUBES; ++i)
    {
        double mx = 0.0, my = 0.0;
        if (!lookupMarkerXY(tf_buffer, frame_id, i, mx, my))
        {
            std::cout << "    [-] TF for marker_" << i << " not ready. Using fallback.\n";
            mx = FALLBACK_X[i - 1];
            my = FALLBACK_Y - TUBE_Y_OFFSET;   // so the tube lands on FALLBACK_Y
        }
        objects.push_back(makeTube(i, frame_id, mx, my));
    }

    // ---- Mixer -------------------------------------------------------------
    double mixer_x = -0.32, mixer_y = -0.33;   // fallback
    if (!lookupMarkerXY(tf_buffer, frame_id, MIXER_MARKER, mixer_x, mixer_y))
        std::cout << "    [-] TF for marker_6 not ready. Using fallback mixer pose.\n";
    objects.push_back(makeMixer(frame_id, mixer_x, mixer_y));

    psi.applyCollisionObjects(objects);
    std::cout << ">>> [INIT] Collision objects loaded.\n";
}

static void waitForStateSettle(MGI & iface, int ms = STATE_SETTLE_MS)
{
    rclcpp::sleep_for(std::chrono::milliseconds(ms));
    iface.startStateMonitor(1.0);
}

// ============================================================================
//  MOTION HELPERS (manual / non-MTC path)
// ============================================================================

// Convert a pose goal into a joint goal seeded from the current configuration.
//
// This is the fix for "the arm goes around itself". With 7 DOF, a pose goal is
// a goal *region* and OMPL samples IK solutions from it -- so consecutive plans
// can land in wildly different arm postures. Seeding IK from the current state
// keeps the solution in the same branch, and RRTConnect then only has to bridge
// two nearby configurations.
static bool setNearestJointGoal(MGI & mg,
                                const geometry_msgs::msg::Pose & pose,
                                int tries = 12,
                                double seed_spread = 0.6,
                                double ik_timeout = 0.05)
{
    auto current_ptr = mg.getCurrentState(2.0);
    if (!current_ptr)
    {
        std::cout << "    [-] Could not read current robot state.\n";
        return false;
    }

    moveit::core::RobotState current(*current_ptr);
    const auto * jmg = current.getJointModelGroup(mg.getName());
    if (!jmg) return false;

    std::vector<double> q_cur;
    current.copyJointGroupPositions(jmg, q_cur);

    std::vector<double> q_best;
    double best_cost = std::numeric_limits<double>::max();

    for (int i = 0; i < tries; ++i)
    {
        moveit::core::RobotState s(current);

        // Attempt 0 seeds from the exact current state (most likely to give a
        // natural nearby solution); later attempts perturb the seed so we do
        // not get stuck in one local branch.
        if (i > 0) s.setToRandomPositionsNearBy(jmg, current, seed_spread);

        if (!s.setFromIK(jmg, pose, mg.getEndEffectorLink(), ik_timeout))
            continue;

        std::vector<double> q;
        s.copyJointGroupPositions(jmg, q);

        // Weight proximal joints more: the same angle at the base sweeps far
        // more volume than at the wrist.
        double cost = 0.0;
        for (size_t k = 0; k < q.size(); ++k)
        {
            const double w = 1.0 + 0.5 * static_cast<double>(q.size() - k);
            cost += w * std::fabs(q[k] - q_cur[k]);
        }

        if (cost < best_cost) { best_cost = cost; q_best = q; }
    }

    if (q_best.empty()) return false;

    mg.setJointValueTarget(q_best);
    return true;
}

// Time-parameterise a raw trajectory (Cartesian paths come back untimed) and
// execute it.
static bool retimeAndExecute(MGI & mg,
                             moveit_msgs::msg::RobotTrajectory & traj_msg,
                             double vel_scale,
                             double acc_scale)
{
    auto current_ptr = mg.getCurrentState(2.0);
    if (!current_ptr) return false;

    robot_trajectory::RobotTrajectory rt(mg.getRobotModel(), mg.getName());
    rt.setRobotTrajectoryMsg(*current_ptr, traj_msg);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(rt, vel_scale, acc_scale))
    {
        std::cout << "    [-] Time parameterisation failed.\n";
        return false;
    }
    rt.getRobotTrajectoryMsg(traj_msg);

    return mg.execute(traj_msg) == moveit::core::MoveItErrorCode::SUCCESS;
}

// Straight-line Cartesian move. Refuses to execute a partial path.
static bool moveLinear(MGI & mg,
                       const geometry_msgs::msg::Pose & target,
                       double vel_scale,
                       double acc_scale,
                       double eef_step = 0.005,
                       double min_fraction = 0.98)
{
    mg.setStartStateToCurrentState();
    mg.clearPoseTargets();

    std::vector<geometry_msgs::msg::Pose> waypoints{ target };
    moveit_msgs::msg::RobotTrajectory traj_msg;

#ifdef MOVEIT_JAZZY_OR_NEWER
    const double fraction = mg.computeCartesianPath(
        waypoints,
        moveit::core::MaxEEFStep(eef_step),
        moveit::core::JumpThreshold::relative(5.0),
        traj_msg,
        true /* avoid_collisions */);
#else
    // jump_threshold = 5.0 (relative). Do NOT pass 0.0 on a redundant arm --
    // that disables the check and lets IK swap null-space branches mid-line,
    // producing a violent flick.
    const double fraction = mg.computeCartesianPath(
        waypoints, eef_step, 5.0, traj_msg, true /* avoid_collisions */);
#endif

    if (fraction < min_fraction)
    {
        std::cout << "    [-] Cartesian path only " << (fraction * 100.0)
                  << "% complete. Not executing.\n";
        return false;
    }

    return retimeAndExecute(mg, traj_msg, vel_scale, acc_scale);
}

// Free-space move with obstacle avoidance, via a seeded joint goal.
static bool moveFreeSpace(MGI & mg,
                          const geometry_msgs::msg::Pose & target,
                          double vel_scale,
                          double acc_scale)
{
    mg.setStartStateToCurrentState();
    mg.clearPoseTargets();
    mg.setMaxVelocityScalingFactor(vel_scale);
    mg.setMaxAccelerationScalingFactor(acc_scale);

    if (!setNearestJointGoal(mg, target))
    {
        std::cout << "    [!] Seeded IK failed; falling back to a pose goal "
                     "(path may be less direct).\n";
        mg.setPoseTarget(target);
    }

    MGI::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        return false;

    // No manual retiming needed here: the OMPL pipeline already applies
    // time-optimal parameterisation with the scaling factors set above.
    return mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

// Free-space transit to a standoff pose, then a pure vertical Cartesian descent.
static bool approachTarget(MGI & mg,
                           const geometry_msgs::msg::Pose & target,
                           double standoff_z,
                           double vel_scale,
                           double acc_scale)
{
    geometry_msgs::msg::Pose standoff = target;
    standoff.position.z += standoff_z;

    std::cout << "    [1/2] Transit to standoff (z+" << standoff_z << ")...\n";
    if (!moveFreeSpace(mg, standoff, vel_scale, acc_scale))
    {
        std::cout << "    [-] Transit failed.\n";
        return false;
    }
    waitForStateSettle(mg, 200);

    std::cout << "    [2/2] Cartesian descent...\n";
    if (!moveLinear(mg, target, vel_scale * 0.5, acc_scale * 0.5))
    {
        std::cout << "    [-] Descent blocked. Holding at standoff.\n";
        return false;
    }
    return true;
}

// ============================================================================
//  MOVEIT TASK CONSTRUCTOR PIPELINE
// ============================================================================

static bool executeLiquidTransferTask(int target_marker,
                                      tf2_ros::Buffer & tf_buffer,
                                      const std::string & planning_frame,
                                      const rclcpp::Node::SharedPtr & node,
                                      const std::string & tcp_link)
{
    const std::string tube_name = "tube_" + std::to_string(target_marker);
    std::cout << "\n>>> [MTC] Building pipeline for " << tube_name << "...\n";

    // Freeze the background updater for the whole plan+execute cycle. MTC plans
    // offline against a snapshot; letting the updater rewrite the scene
    // underneath it produces plans that are invalid before they even run.
    ScenePause pause;

    // ---- 1. TF lookups (fail fast before building anything) ----------------
    double tube_mx = 0.0, tube_my = 0.0, mixer_mx = 0.0, mixer_my = 0.0;
    if (!lookupMarkerXY(tf_buffer, planning_frame, target_marker, tube_mx, tube_my))
    {
        std::cout << "[-] MTC: TF lookup for marker_" << target_marker << " failed.\n";
        return false;
    }
    if (!lookupMarkerXY(tf_buffer, planning_frame, MIXER_MARKER, mixer_mx, mixer_my))
    {
        std::cout << "[-] MTC: TF lookup for marker_6 (mixer) failed.\n";
        return false;
    }

    const double grasp_x = tube_mx;
    const double grasp_y = tube_my + GRASP_Y_OFFSET;

    tf2::Quaternion q_upright;
    q_upright.setRPY(0.0, -M_PI / 2.0, M_PI / 2.0);
    const geometry_msgs::msg::Quaternion q_upright_msg = tf2::toMsg(q_upright);

    // ---- 2. Task and solvers -----------------------------------------------
    // Static so the Task outlives this function and the RViz "Motion Planning
    // Tasks" panel can still introspect the solution after execution.
    static std::unique_ptr<mtc::Task> task_holder;
    task_holder = std::make_unique<mtc::Task>();
    mtc::Task & task = *task_holder;

    task.stages()->setName("liquid transfer " + tube_name);
    task.loadRobotModel(node);

    task.setProperty("group", std::string("arm_group"));
    task.setProperty("ik_frame", tcp_link);

    // Your MTC version exposes only PipelinePlanner(node, pipeline_name). The
    // three-argument constructor that also takes a planner id was added later,
    // so the id goes in as a property instead.
    //
    // If setPlannerId() is missing on your version too, replace that line with:
    //     planner_ompl->setProperty("planner", std::string("RRTConnectkConfigDefault"));
    auto planner_ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node, "ompl");
    planner_ompl->setPlannerId("RRTConnectkConfigDefault");
    planner_ompl->setMaxVelocityScalingFactor(VEL_SCALE_TRANSIT);
    planner_ompl->setMaxAccelerationScalingFactor(ACC_SCALE_TRANSIT);
    planner_ompl->setTimeout(10.0);

    auto planner_cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    planner_cartesian->setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
    planner_cartesian->setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);
    planner_cartesian->setStepSize(0.005);
    // If your MTC version has deprecated setJumpThreshold, drop this line --
    // but do not set it to 0 on a redundant arm.
    planner_cartesian->setJumpThreshold(5.0);

    // Gripper open/close needs no obstacle avoidance; straight joint
    // interpolation is faster and cannot fail for spurious planner reasons.
    auto planner_gripper = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    const auto * gripper_jmg = task.getRobotModel()->getJointModelGroup("gripper");
    if (!gripper_jmg)
    {
        std::cout << "[-] MTC: no 'gripper' joint model group in the SRDF.\n";
        return false;
    }
    const std::vector<std::string> gripper_links = gripper_jmg->getLinkModelNames();

    // ---- 3. Stages ----------------------------------------------------------

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

    {
        auto s = std::make_unique<mtc::stages::MoveTo>("open gripper", planner_gripper);
        s->setGroup("gripper");
        s->setGoal("open");
        task.add(std::move(s));
    }

    // Pre-grasp: directly above the tube.
    geometry_msgs::msg::PoseStamped pregrasp;
    pregrasp.header.frame_id  = planning_frame;
    pregrasp.pose.position.x  = grasp_x;
    pregrasp.pose.position.y  = grasp_y;
    pregrasp.pose.position.z  = GRASP_Z + APPROACH_DIST;
    pregrasp.pose.orientation = q_upright_msg;
    {
        auto s = std::make_unique<mtc::stages::MoveTo>("move to pre-grasp", planner_ompl);
        s->setGroup("arm_group");
        s->setGoal(pregrasp);
        task.add(std::move(s));
    }

    // CRITICAL: the fingers are about to descend around a collision cylinder.
    // attachObject() alone does NOT create these ACM entries -- without this
    // stage the Cartesian approach returns a partial fraction and the task
    // fails at the first descent.
    {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper-tube contact");
        s->allowCollisions(tube_name, gripper_links, true);
        task.add(std::move(s));
    }

    // Descend a deterministic distance. A loose min/max bracket lets MoveRelative
    // take the full max whenever nothing blocks, which silently shifts the grasp
    // height by centimetres.
    {
        auto s = std::make_unique<mtc::stages::MoveRelative>("approach tube", planner_cartesian);
        s->setGroup("arm_group");
        s->setIKFrame(tcp_link);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = planning_frame;
        dir.vector.z = -1.0;
        s->setDirection(dir);
        s->setMinMaxDistance(APPROACH_DIST - 0.01, APPROACH_DIST);
        task.add(std::move(s));
    }

    // Attach before closing, so the ACM is already correct while the fingers
    // move into the tube geometry.
    {
        auto s = std::make_unique<mtc::stages::ModifyPlanningScene>("attach tube");
        s->attachObject(tube_name, tcp_link);
        task.add(std::move(s));
    }
    {
        auto s = std::make_unique<mtc::stages::MoveTo>("close gripper", planner_gripper);
        s->setGroup("gripper");
        s->setGoal("closed");
        task.add(std::move(s));
    }

    // Straight vertical lift clears the rack before the arm reorients.
    {
        auto s = std::make_unique<mtc::stages::MoveRelative>("lift tube", planner_cartesian);
        s->setGroup("arm_group");
        s->setIKFrame(tcp_link);
        geometry_msgs::msg::Vector3Stamped dir;
        dir.header.frame_id = planning_frame;
        dir.vector.z = 1.0;
        s->setDirection(dir);
        s->setMinMaxDistance(LIFT_DIST - 0.02, LIFT_DIST);
        task.add(std::move(s));
    }

    // Transit to the mixer, tube held upright.
    geometry_msgs::msg::PoseStamped pour_pose;
    pour_pose.header.frame_id  = planning_frame;
    pour_pose.pose.position.x  = mixer_mx;
    pour_pose.pose.position.y  = mixer_my;
    pour_pose.pose.position.z  = POUR_Z;
    pour_pose.pose.orientation = q_upright_msg;
    {
        auto s = std::make_unique<mtc::stages::MoveTo>("move to mixer", planner_ompl);
        s->setGroup("arm_group");
        s->setGoal(pour_pose);

        // VERIFY THIS: the free axis must be the one the tube is symmetric
        // about. Free the wrong axis and you either over-constrain (planning
        // stalls) or the tube tips during transit. Your two earlier versions
        // disagreed about which axis this is.
        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name                 = tcp_link;
        ocm.header.frame_id           = planning_frame;
        ocm.orientation               = q_upright_msg;
        ocm.absolute_x_axis_tolerance = M_PI;    // <-- free axis
        ocm.absolute_y_axis_tolerance = 0.25;
        ocm.absolute_z_axis_tolerance = 0.25;
        ocm.weight                    = 1.0;

        moveit_msgs::msg::Constraints c;
        c.orientation_constraints.push_back(ocm);
        s->setPathConstraints(c);

        task.add(std::move(s));
    }

    // ---- 4. Plan and execute ------------------------------------------------
    try
    {
        if (!task.plan(3))
        {
            std::cout << "[-] MTC found no valid pipeline.\n"
                         "    Open the 'Motion Planning Tasks' panel in RViz to see\n"
                         "    which stage failed and why.\n";
            return false;
        }
    }
    catch (const mtc::InitStageException & e)
    {
        std::cout << "[-] MTC init error: " << e << "\n";
        return false;
    }
    catch (const std::exception & e)
    {
        std::cout << "[-] MTC planning threw: " << e.what() << "\n";
        return false;
    }

    std::cout << ">>> [MTC] Plan found (" << task.solutions().size()
              << " solutions). Executing best...\n";

    const auto result = task.execute(*task.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        std::cout << "[-] MTC execution failed, error code " << result.val << "\n";
        return false;
    }

    return true;
}

// ============================================================================
//  MAIN
// ============================================================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("liquid_handler_master");

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    std::thread spinner([executor]() { executor->spin(); });

    MGI arm_interface(node, "arm_group");
    MGI gripper_interface(node, "gripper");

    const std::string planning_frame = arm_interface.getPlanningFrame();
    const std::string tcp_link       = arm_interface.getEndEffectorLink();

    std::cout << ">>> [INIT] Planning frame: " << planning_frame
              << " | TCP link: " << tcp_link << "\n";

    auto tf_buffer   = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    std::cout << ">>> [INIT] Waiting 2 s for the TF tree to populate...\n";
    rclcpp::sleep_for(std::chrono::seconds(2));

    setupCollisionObjects(planning_frame, *tf_buffer);

    tf2::Quaternion q_upright;
    q_upright.setRPY(0.0, -M_PI / 2.0, M_PI / 2.0);

    // ---- Planner defaults ---------------------------------------------------
    arm_interface.setPlanningPipelineId("ompl");
    arm_interface.setPlannerId("RRTConnectkConfigDefault");
    arm_interface.setPlanningTime(10.0);
    arm_interface.setNumPlanningAttempts(5);
    arm_interface.setWorkspace(-1.0, -1.0, -0.1, 1.0, 1.0, 1.0);
    // Tight tolerances throughout. The original 0.12 m position tolerance made
    // the goal a 12 cm blob, which is a large region for OMPL to sample from.
    arm_interface.setGoalPositionTolerance(0.005);
    arm_interface.setGoalOrientationTolerance(0.02);

    geometry_msgs::msg::Pose home_pose;
    home_pose.position.x    = -0.15;
    home_pose.position.y    = -0.15;
    home_pose.position.z    = 0.20;
    home_pose.orientation.x = q_upright.x();
    home_pose.orientation.y = q_upright.y();
    home_pose.orientation.z = q_upright.z();
    home_pose.orientation.w = q_upright.w();

    std::cout << ">>> [INIT] Moving to home pose...\n";
    if (!moveFreeSpace(arm_interface, home_pose, VEL_SCALE_TRANSIT, ACC_SCALE_TRANSIT))
    {
        std::cout << "[-] [INIT] Failed to reach the home pose. Aborting.\n";
        rclcpp::shutdown();
        spinner.join();
        return -1;
    }
    waitForStateSettle(arm_interface);

    // ---- Command intake -----------------------------------------------------
    auto gui_sub = node->create_subscription<std_msgs::msg::String>(
        "/gui_commands", 10,
        [](const std_msgs::msg::String::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            command_queue.push(msg->data);
            std::cout << "[QUEUE] " << msg->data
                      << " (pending: " << command_queue.size() << ")\n";
        });

    std::string attached_tube;
    int current_marker_id = -1;

    // ---- Background scene updater -------------------------------------------
    moveit::planning_interface::PlanningSceneInterface dynamic_scene_interface;
    auto dynamic_updater = node->create_wall_timer(
        std::chrono::milliseconds(500),
        [&]()
        {
            if (scene_updates_paused.load()) return;

            std::vector<moveit_msgs::msg::CollisionObject> updates;
            const int held = attached_marker_id.load();

            for (int i = 1; i <= NUM_TUBES; ++i)
            {
                if (i == held) continue;   // never fight the attached copy
                double mx = 0.0, my = 0.0;
                if (lookupMarkerXY(*tf_buffer, planning_frame, i, mx, my))
                    updates.push_back(makeTube(i, planning_frame, mx, my));
            }

            double mixer_x = 0.0, mixer_y = 0.0;
            if (lookupMarkerXY(*tf_buffer, planning_frame, MIXER_MARKER, mixer_x, mixer_y))
                updates.push_back(makeMixer(planning_frame, mixer_x, mixer_y));

            if (!updates.empty())
                dynamic_scene_interface.applyCollisionObjects(updates);
        });

    std::cout << "\n>>> Ready. Listening on /gui_commands.\n";

    // ========================================================================
    //  MAIN EXECUTION LOOP
    // ========================================================================
    while (rclcpp::ok())
    {
        std::string line;
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (!command_queue.empty())
            {
                line = command_queue.front();
                command_queue.pop();
            }
        }

        if (line.empty())
        {
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        std::cout << "\n[EXECUTING] " << line << "\n";

        // ---- Quit -----------------------------------------------------------
        if (line == "q" || line == "Q") break;

        // ---- MTC pipeline ---------------------------------------------------
        // This check MUST come before the coordinate parsing below. In the
        // previous version it sat after the "m<N>" / "x y z" block, so "TASK 3"
        // fell into the coordinate parser, failed to read three doubles, and
        // hit `continue` -- the MTC branch was unreachable dead code.
        if (line.rfind("TASK", 0) == 0 || line.rfind("task", 0) == 0)
        {
            int marker_id = -1;
            try
            {
                marker_id = std::stoi(line.substr(4));   // handles "TASK3" and "TASK 3"
            }
            catch (const std::exception &)
            {
                std::cout << "    [-] Bad command. Use: TASK <1-" << NUM_TUBES << ">\n";
                continue;
            }

            if (marker_id < 1 || marker_id > NUM_TUBES)
            {
                std::cout << "    [-] TASK marker must be 1-" << NUM_TUBES << ".\n";
                continue;
            }
            if (!attached_tube.empty())
            {
                std::cout << "    [-] Already holding " << attached_tube
                          << ". Send 'o' to release first.\n";
                continue;
            }

            const bool ok = executeLiquidTransferTask(
                marker_id, *tf_buffer, planning_frame, node, tcp_link);

            if (ok)
            {
                // Keep the MoveGroupInterface-side bookkeeping in sync, or the
                // 'o' command will not know there is anything to detach and the
                // updater will republish a world copy of the held tube.
                attached_tube     = "tube_" + std::to_string(marker_id);
                current_marker_id = marker_id;
                attached_marker_id.store(marker_id);
                std::cout << ">>> Task complete. Holding " << attached_tube << ".\n";
            }
            else
            {
                std::cout << "    [!] Task failed or aborted.\n";
            }
            waitForStateSettle(arm_interface);
            continue;
        }

        // ---- Home -----------------------------------------------------------
        if (line == "h" || line == "H")
        {
            const double v = attached_tube.empty() ? VEL_SCALE_TRANSIT : VEL_SCALE_LIQUID;
            const double a = attached_tube.empty() ? ACC_SCALE_TRANSIT : ACC_SCALE_LIQUID;
            if (moveFreeSpace(arm_interface, home_pose, v, a))
                std::cout << ">>> Home.\n";
            else
                std::cout << "    [-] Could not reach home.\n";
            waitForStateSettle(arm_interface);
            continue;
        }

        // ---- Open gripper ---------------------------------------------------
        if (line == "o" || line == "O")
        {
            std::cout << ">>> Opening gripper...\n";
            gripper_interface.setNamedTarget("open");
            if (gripper_interface.move() == moveit::core::MoveItErrorCode::SUCCESS)
            {
                if (!attached_tube.empty())
                {
                    // detachObject() returns the object to the world at its
                    // current attached pose. The updater will then snap it back
                    // to wherever its marker is.
                    arm_interface.detachObject(attached_tube);
                    std::cout << ">>> Detached " << attached_tube << ".\n";
                    attached_tube.clear();
                    attached_marker_id.store(-1);
                }
            }
            else
            {
                std::cout << "    [-] Gripper failed to open.\n";
            }
            continue;
        }

        // ---- Close gripper (manual grasp) -----------------------------------
        if (line == "c" || line == "C")
        {
            std::cout << ">>> Closing gripper...\n";

            if (current_marker_id >= 1 && current_marker_id <= NUM_TUBES && attached_tube.empty())
            {
                const std::string tube_id = "tube_" + std::to_string(current_marker_id);

                // Re-place the tube at the TCP before attaching, so the attached
                // geometry matches where the tube physically is.
                const auto eef = arm_interface.getCurrentPose().pose;
                moveit_msgs::msg::CollisionObject restored;
                restored.id = tube_id;
                restored.header.frame_id = planning_frame;
                restored.primitives.resize(1);
                restored.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                restored.primitives[0].dimensions = { TUBE_HEIGHT, TUBE_RADIUS };
                restored.primitive_poses.resize(1);
                restored.primitive_poses[0].position.x = eef.position.x;
                restored.primitive_poses[0].position.y = eef.position.y - GRASP_Y_OFFSET + TUBE_Y_OFFSET;
                restored.primitive_poses[0].position.z = TUBE_CENTER_Z;
                restored.primitive_poses[0].orientation.w = 1.0;
                restored.operation = restored.ADD;

                attached_marker_id.store(current_marker_id);   // stop the updater first
                dynamic_scene_interface.applyCollisionObject(restored);
                rclcpp::sleep_for(std::chrono::milliseconds(150));

                std::vector<std::string> touch_links;
                if (const auto * jmg = gripper_interface.getRobotModel()->getJointModelGroup("gripper"))
                    touch_links = jmg->getLinkModelNames();

                arm_interface.attachObject(tube_id, tcp_link, touch_links);
                attached_tube = tube_id;
                std::cout << ">>> Attached " << tube_id << " to " << tcp_link << ".\n";

                // Let the planning scene publish the new ACM before moving.
                rclcpp::sleep_for(std::chrono::milliseconds(200));
            }

            gripper_interface.setNamedTarget("closed");
            if (gripper_interface.move() == moveit::core::MoveItErrorCode::SUCCESS)
            {
                std::cout << ">>> Gripper closed.\n";
            }
            else
            {
                std::cout << "    [-] Gripper failed to close. Reverting attach.\n";
                if (!attached_tube.empty())
                {
                    arm_interface.detachObject(attached_tube);
                    attached_tube.clear();
                    attached_marker_id.store(-1);
                }
            }
            continue;
        }

        // ---- Pour -----------------------------------------------------------
        if (line == "p" || line == "P")
        {
            if (attached_tube.empty())
            {
                std::cout << "    [-] Nothing attached; refusing to pour.\n";
                continue;
            }

            std::cout << ">>> Pouring...\n";
            auto state = arm_interface.getCurrentState(2.0);
            if (!state)
            {
                std::cout << "    [-] Could not read robot state.\n";
                continue;
            }

            const auto * jmg = state->getJointModelGroup("arm_group");
            std::vector<double> joints;
            state->copyJointGroupPositions(jmg, joints);
            if (joints.empty()) continue;

            const double upright = joints.back();

            arm_interface.setMaxVelocityScalingFactor(VEL_SCALE_LIQUID);
            arm_interface.setMaxAccelerationScalingFactor(ACC_SCALE_LIQUID);

            joints.back() = upright + M_PI / 2.0;
            arm_interface.setJointValueTarget(joints);
            if (arm_interface.move() == moveit::core::MoveItErrorCode::SUCCESS)
            {
                waitForStateSettle(arm_interface);
                std::cout << "    [+] Tilted. Draining...\n";
                rclcpp::sleep_for(std::chrono::seconds(1));

                joints.back() = upright;
                arm_interface.setJointValueTarget(joints);
                if (arm_interface.move() == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    waitForStateSettle(arm_interface);
                    std::cout << ">>> Poured.\n";
                }
                else
                {
                    std::cout << "    [!] Failed to return upright -- tube may still be tilted.\n";
                }
            }
            else
            {
                std::cout << "    [-] Could not execute the pour.\n";
            }
            continue;
        }

        // ---- Manual move: "m<N>" or raw "x y z" ------------------------------
        double tx = 0.0, ty = 0.0, tz = 0.0;
        double standoff = STANDOFF_TUBE;

        if (line[0] == 'm' || line[0] == 'M')
        {
            int marker_id = -1;
            std::stringstream ss(line.substr(1));
            if (!(ss >> marker_id))
            {
                std::cout << "    [-] Bad marker command.\n";
                continue;
            }
            current_marker_id = marker_id;

            if (marker_id == 0)
            {
                tx = 0.0; ty = -0.065; tz = 0.20;
            }
            else
            {
                double mx = 0.0, my = 0.0;
                if (lookupMarkerXY(*tf_buffer, planning_frame, marker_id, mx, my))
                {
                    tx = mx;
                    if (marker_id == MIXER_MARKER)
                    {
                        ty = my;
                        tz = POUR_Z;
                        standoff = STANDOFF_MIXER;
                    }
                    else
                    {
                        ty = my + GRASP_Y_OFFSET;
                        tz = GRASP_Z;
                    }
                }
                else
                {
                    std::cout << "    [-] No TF for marker_" << marker_id
                              << "; using hardcoded fallback.\n";
                    static const double FB_X[] = { -0.17, -0.10, 0.00, 0.10, 0.17 };
                    if (marker_id >= 1 && marker_id <= NUM_TUBES)
                    {
                        tx = FB_X[marker_id - 1]; ty = -0.22; tz = 0.10;
                    }
                    else if (marker_id == MIXER_MARKER)
                    {
                        tx = -0.20; ty = -0.11; tz = POUR_Z;
                        standoff = STANDOFF_MIXER;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
        else
        {
            std::stringstream ss(line);
            if (!(ss >> tx >> ty >> tz))
            {
                std::cout << "    [-] Unrecognised command.\n";
                continue;
            }
        }

        // ---- Execute the manual hybrid move ----------------------------------
        std::cout << "--- Moving to (" << tx << ", " << ty << ", " << tz << ") ---\n";

        geometry_msgs::msg::Pose target;
        target.position.x    = tx;
        target.position.y    = ty;
        target.position.z    = tz;
        target.orientation.x = q_upright.x();
        target.orientation.y = q_upright.y();
        target.orientation.z = q_upright.z();
        target.orientation.w = q_upright.w();

        const double v = attached_tube.empty() ? VEL_SCALE_TRANSIT : VEL_SCALE_LIQUID;
        const double a = attached_tube.empty() ? ACC_SCALE_TRANSIT : ACC_SCALE_LIQUID;

        // Keep the tube upright for the WHOLE transit, not just near the mixer.
        if (!attached_tube.empty())
        {
            moveit_msgs::msg::OrientationConstraint ocm;
            ocm.link_name                 = tcp_link;
            ocm.header.frame_id           = planning_frame;
            ocm.orientation               = target.orientation;
            ocm.absolute_x_axis_tolerance = M_PI;    // free axis -- verify
            ocm.absolute_y_axis_tolerance = 0.25;
            ocm.absolute_z_axis_tolerance = 0.25;
            ocm.weight                    = 1.0;

            moveit_msgs::msg::Constraints c;
            c.orientation_constraints.push_back(ocm);
            arm_interface.setPathConstraints(c);
        }

        if (!approachTarget(arm_interface, target, standoff, v, a))
        {
            arm_interface.clearPathConstraints();
            std::cout << "    [!] RECOVERY: returning home...\n";
            if (!moveFreeSpace(arm_interface, home_pose, VEL_SCALE_TRANSIT, ACC_SCALE_TRANSIT))
                std::cout << "    [!] CRITICAL: could not reach home. Arm may be trapped.\n";
        }
        else
        {
            std::cout << ">>> Target reached.\n";
        }

        arm_interface.clearPathConstraints();
        waitForStateSettle(arm_interface);
    }   // end while

    std::cout << "\n>>> Shutting down.\n";
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
