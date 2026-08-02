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

#include <Eigen/Geometry>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
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
static constexpr double VEL_SCALE_LIQUID  = 0.15;   // slower when carrying
static constexpr double ACC_SCALE_LIQUID  = 0.05;

// ---- Table -----------------------------------------------------------------
static constexpr double TABLE_SIZE_XY   = 1.5;
static constexpr double TABLE_THICKNESS = 0.04;
static constexpr double TABLE_CENTER_Z  = -0.03;    // -> top surface at z = -0.01

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

// How much rotation about the tool axis the interpolator may invent when a
// strict straight line fails. A test tube is a cylinder, so spin about its long
// axis does not change the grasp -- that freedom is yours to spend.
// Set to 0.0 if your gripper fingers are asymmetric or a cable would wrap.
// IMPORTANT: CartesianOptions::free_axis defaults to the TCP frame's Z. Verify
// that Z is the axis running along the tube on your robot before enabling this.
static constexpr double TOOL_AXIS_FREEDOM = M_PI;
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
//
//  Contains a hand-written Cartesian interpolator. The short version of what
//  that is: your controller only accepts joint angles, but you want the tool
//  tip to travel in a straight line. There is no closed-form answer, so the
//  line gets chopped into small steps and IK is solved at each one.
//
//  MoveIt's computeCartesianPath() does the same thing but STOPS at the first
//  step where IK fails -- that is where the "90.4762%" came from. This version
//  retries with different seeds, tries small rotations about the tool axis,
//  and halves the step size before giving up.
// ============================================================================

// ---------------------------------------------------------------------------
// Collision-aware IK.
//
// setFromIK() checks kinematics and joint limits but NOT collisions. Handing
// OMPL a single joint goal that happens to be in collision produces:
//     "Unable to sample any valid states for goal tree"
// because no valid state satisfies the goal. This wrapper gives setFromIK a
// validity callback so it only ever returns collision-free solutions.
// ---------------------------------------------------------------------------
class IkValidator
{
public:
    explicit IkValidator(const rclcpp::Node::SharedPtr & node,
                         const std::string & robot_description = "robot_description")
    {
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, robot_description);
        psm_->startSceneMonitor("/monitored_planning_scene");
        psm_->startWorldGeometryMonitor();
        psm_->startStateMonitor();
        if (!psm_->requestPlanningSceneState("/get_planning_scene"))
            std::cout << "    [!] Could not fetch the initial planning scene; "
                         "IK collision checks may be unreliable at startup.\n";
    }

    moveit::core::GroupStateValidityCallbackFn callback()
    {
        auto psm = psm_;
        return [psm](moveit::core::RobotState * state,
                     const moveit::core::JointModelGroup * group,
                     const double * values) -> bool
        {
            state->setJointGroupPositions(group, values);
            state->update();
            planning_scene_monitor::LockedPlanningSceneRO scene(psm);
            if (!scene) return true;             // no scene yet: do not block IK
            return !scene->isStateColliding(*state, group->getName());
        };
    }

    bool isStateValid(const moveit::core::RobotState & state, const std::string & group)
    {
        planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
        if (!scene) return true;
        return !scene->isStateColliding(state, group);
    }

private:
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};

// ---------------------------------------------------------------------------
// Find a collision-free IK solution close to a seed configuration.
//
// On a 7-DOF arm a pose goal is a goal *region* -- OMPL samples IK solutions
// from it, so consecutive plans can land in wildly different arm postures and
// the arm appears to loop around itself. Pinning one nearby solution fixes
// that, provided the solution is collision-free (hence the validator).
// ---------------------------------------------------------------------------
static bool solveNearestIk(MGI & mg,
                           IkValidator & validator,
                           const geometry_msgs::msg::Pose & pose,
                           const moveit::core::RobotState & seed_state,
                           moveit::core::RobotState & out_state,
                           int tries = 15,
                           double seed_spread = 0.8,
                           double ik_timeout = 0.05)
{
    const auto * jmg = seed_state.getJointModelGroup(mg.getName());
    if (!jmg) return false;

    const std::string tip = mg.getEndEffectorLink();
    auto validity = validator.callback();

    std::vector<double> q_seed;
    seed_state.copyJointGroupPositions(jmg, q_seed);

    double best_cost = std::numeric_limits<double>::max();
    bool found = false;

    for (int i = 0; i < tries; ++i)
    {
        moveit::core::RobotState s(seed_state);
        if (i > 0) s.setToRandomPositionsNearBy(jmg, seed_state, seed_spread);

        if (!s.setFromIK(jmg, pose, tip, ik_timeout, validity)) continue;
        if (!s.satisfiesBounds(jmg)) continue;

        std::vector<double> q;
        s.copyJointGroupPositions(jmg, q);

        // Weight proximal joints more: the same angle at the base sweeps far
        // more volume than at the wrist.
        double cost = 0.0;
        for (size_t k = 0; k < q.size(); ++k)
        {
            const double w = 1.0 + 0.5 * static_cast<double>(q.size() - k);
            cost += w * std::fabs(q[k] - q_seed[k]);
        }

        if (cost < best_cost) { best_cost = cost; out_state = s; found = true; }
    }

    return found;
}

static bool setNearestJointGoal(MGI & mg,
                                IkValidator & validator,
                                const geometry_msgs::msg::Pose & pose)
{
    auto current = mg.getCurrentState(2.0);
    if (!current) return false;

    moveit::core::RobotState goal(*current);
    if (!solveNearestIk(mg, validator, pose, *current, goal)) return false;

    mg.setJointValueTarget(goal);
    return true;
}

// ---------------------------------------------------------------------------
// Time-parameterise a raw trajectory and execute it. Cartesian paths come back
// with no velocities or timing, so this step is mandatory for them.
//
// Kept because it is useful if you ever want to execute a hand-built
// trajectory. [[maybe_unused]] silences the warning while nothing calls it.
// ---------------------------------------------------------------------------
[[maybe_unused]] static bool retimeAndExecute(MGI & mg,
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

// ============================================================================
//  THE INTERPOLATOR
// ============================================================================

struct CartesianOptions
{
    double step               = 0.005;   // nominal spacing between waypoints, m
    double max_joint_step     = 0.30;    // rad; reject a waypoint that jumps more
    int    seeds_per_waypoint = 5;       // perturbed-seed retries per waypoint
    int    max_subdivisions   = 4;       // how many times to halve a failing step
    double ik_timeout         = 0.02;

    // ---- Free-axis relaxation ---------------------------------------------
    // A test tube is a cylinder: rotating the gripper about the tube's long
    // axis does not change the grasp. MoveIt's version demands a fully
    // specified 6-DOF pose at every waypoint and throws that freedom away.
    // Letting each waypoint pick its own rotation about free_axis turns a
    // 6-DOF-constrained problem into a 5-DOF one, which massively enlarges
    // the set of reachable paths.
    //
    // Set to 0.0 when the exact orientation genuinely matters.
    double          free_axis_tolerance = 0.0;               // rad, e.g. M_PI
    Eigen::Vector3d free_axis           = Eigen::Vector3d::UnitZ();  // in TCP frame
    int             free_axis_samples   = 9;
};

// Straight-line blend for position, slerp for orientation.
static Eigen::Isometry3d lerpPose(const Eigen::Isometry3d & a,
                                  const Eigen::Isometry3d & b,
                                  double t)
{
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = (1.0 - t) * a.translation() + t * b.translation();
    const Eigen::Quaterniond qa(a.rotation());
    const Eigen::Quaterniond qb(b.rotation());
    out.linear() = qa.slerp(t, qb).toRotationMatrix();
    return out;
}

static Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::msg::Pose & p)
{
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    out.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                      p.orientation.y, p.orientation.z)
                       .normalized().toRotationMatrix();
    return out;
}

static double maxJointDelta(const moveit::core::RobotState & a,
                            const moveit::core::RobotState & b,
                            const moveit::core::JointModelGroup * jmg)
{
    std::vector<double> qa, qb;
    a.copyJointGroupPositions(jmg, qa);
    b.copyJointGroupPositions(jmg, qb);
    double worst = 0.0;
    for (size_t i = 0; i < qa.size(); ++i)
        worst = std::max(worst, std::fabs(qa[i] - qb[i]));
    return worst;
}

// ---------------------------------------------------------------------------
// Solve a single waypoint. This is where all the retry logic lives -- the part
// MoveIt's built-in version does not have.
// ---------------------------------------------------------------------------
static bool solveWaypoint(const moveit::core::JointModelGroup * jmg,
                          const std::string & tip,
                          IkValidator & validator,
                          const Eigen::Isometry3d & goal_pose,
                          const moveit::core::RobotState & prev,
                          const CartesianOptions & opts,
                          moveit::core::RobotState & out)
{
    auto validity = validator.callback();

    auto attempt = [&](const Eigen::Isometry3d & pose,
                       const moveit::core::RobotState & seed) -> bool
    {
        moveit::core::RobotState s(seed);
        if (!s.setFromIK(jmg, pose, tip, opts.ik_timeout, validity)) return false;
        if (!s.satisfiesBounds(jmg)) return false;
        // Local continuity check: reject a solution that jumped to a different
        // part of the null space. This is the explicit version of MoveIt's
        // "jump threshold", but measured against the previous waypoint rather
        // than the average over the whole path.
        if (maxJointDelta(prev, s, jmg) > opts.max_joint_step) return false;
        out = s;
        return true;
    };

    // 1. Exact pose, seeded from the previous solution.
    if (attempt(goal_pose, prev)) return true;

    // 2. Rotations about the free axis, walking outwards from zero so the
    //    smallest deviation from the commanded orientation wins.
    if (opts.free_axis_tolerance > 1e-6 && opts.free_axis_samples > 1)
    {
        const int half = std::max(1, opts.free_axis_samples / 2);
        for (int k = 1; k <= half; ++k)
        {
            const double mag = opts.free_axis_tolerance *
                               static_cast<double>(k) / static_cast<double>(half);
            for (const double sign : { 1.0, -1.0 })
            {
                Eigen::Isometry3d r = Eigen::Isometry3d::Identity();
                r.linear() = Eigen::AngleAxisd(sign * mag, opts.free_axis.normalized())
                                 .toRotationMatrix();
                if (attempt(goal_pose * r, prev)) return true;
            }
        }
    }

    // 3. Perturbed seeds, exact pose.
    for (int i = 0; i < opts.seeds_per_waypoint; ++i)
    {
        moveit::core::RobotState seed(prev);
        seed.setToRandomPositionsNearBy(jmg, prev, 0.25);
        if (attempt(goal_pose, seed)) return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// The interpolator itself. Returns the fraction of the line achieved and fills
// `out` with whatever it managed to build.
// ---------------------------------------------------------------------------
static double planRobustCartesian(MGI & mg,
                                  IkValidator & validator,
                                  const geometry_msgs::msg::Pose & target,
                                  const CartesianOptions & opts,
                                  robot_trajectory::RobotTrajectory & out)
{
    auto current = mg.getCurrentState(2.0);
    if (!current) return 0.0;

    const auto * jmg = current->getJointModelGroup(mg.getName());
    if (!jmg) return 0.0;
    const std::string tip = mg.getEndEffectorLink();

    const Eigen::Isometry3d start_pose = current->getGlobalLinkTransform(tip);
    const Eigen::Isometry3d end_pose   = poseMsgToEigen(target);

    const double distance = (end_pose.translation() - start_pose.translation()).norm();
    if (distance < 1e-6) return 1.0;

    const double dt_nominal = std::min(1.0, opts.step / distance);

    out.clear();
    out.addSuffixWayPoint(*current, 0.0);

    moveit::core::RobotState prev(*current);
    double t_done = 0.0;
    int refinements = 0;
    int guard = 0;

    while (t_done < 1.0 - 1e-9 && guard++ < 4000)
    {
        double dt = dt_nominal;
        bool advanced = false;

        for (int sub = 0; sub <= opts.max_subdivisions; ++sub)
        {
            const double t_next = std::min(1.0, t_done + dt);
            const Eigen::Isometry3d goal_i = lerpPose(start_pose, end_pose, t_next);

            moveit::core::RobotState s(prev);
            if (solveWaypoint(jmg, tip, validator, goal_i, prev, opts, s))
            {
                out.addSuffixWayPoint(s, 0.0);
                prev = s;
                t_done = t_next;
                advanced = true;
                if (sub > 0) ++refinements;
                break;
            }

            dt *= 0.5;   // adaptive refinement: try a smaller move
        }

        if (!advanced) break;   // genuinely stuck
    }

    if (refinements > 0)
        std::cout << "    [cart] refined the step " << refinements
                  << " time(s) to get through tight spots.\n";

    return t_done;
}

// ---------------------------------------------------------------------------
// Pure joint-space interpolation between the current state and a target joint
// configuration. Calls IK exactly ONCE (at the endpoint) instead of once per
// waypoint, so it either works or fails cleanly -- never a mysterious 90%.
//
// The cost is that the tip bows slightly off the true straight line, so the
// deviation is measured and the move is refused if it exceeds max_deviation.
// Over a 10 cm approach expect 1-3 mm.
// ---------------------------------------------------------------------------
static bool moveJointInterpolated(MGI & mg,
                                  IkValidator & validator,
                                  const moveit::core::RobotState & goal_state,
                                  double vel_scale,
                                  double acc_scale,
                                  int steps = 30,
                                  double max_deviation = 0.006)
{
    auto current = mg.getCurrentState(2.0);
    if (!current) return false;

    const auto * jmg = current->getJointModelGroup(mg.getName());
    const std::string tip = mg.getEndEffectorLink();

    const Eigen::Vector3d p_start = current->getGlobalLinkTransform(tip).translation();
    const Eigen::Vector3d p_end   = goal_state.getGlobalLinkTransform(tip).translation();
    const Eigen::Vector3d line    = p_end - p_start;
    const double line_len = line.norm();

    robot_trajectory::RobotTrajectory rt(mg.getRobotModel(), mg.getName());
    double worst_dev = 0.0;

    for (int i = 0; i <= steps; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        moveit::core::RobotState s(*current);
        current->interpolate(goal_state, t, s, jmg);
        s.update();

        if (!validator.isStateValid(s, mg.getName()))
        {
            std::cout << "    [-] Joint interpolation hits a collision at t=" << t << ".\n";
            return false;
        }

        if (line_len > 1e-6)
        {
            const Eigen::Vector3d p = s.getGlobalLinkTransform(tip).translation();
            const Eigen::Vector3d v = p - p_start;
            const double proj = v.dot(line) / line_len;
            const double dev = (v - (proj / line_len) * line).norm();
            worst_dev = std::max(worst_dev, dev);
        }

        rt.addSuffixWayPoint(s, 0.0);
    }

    std::cout << "    [joint] max deviation from the straight line = "
              << (worst_dev * 1000.0) << " mm\n";

    if (worst_dev > max_deviation)
    {
        std::cout << "    [-] Deviation exceeds " << (max_deviation * 1000.0)
                  << " mm; refusing.\n";
        return false;
    }

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(rt, vel_scale, acc_scale)) return false;

    moveit_msgs::msg::RobotTrajectory msg;
    rt.getRobotTrajectoryMsg(msg);
    return mg.execute(msg) == moveit::core::MoveItErrorCode::SUCCESS;
}

// ---------------------------------------------------------------------------
// Straight-line move with a three-rung fallback ladder:
//   1. interpolator, exact orientation
//   2. interpolator, allowing rotation about the tool axis
//   3. joint interpolation between IK-verified endpoints
// ---------------------------------------------------------------------------
static bool moveLinear(MGI & mg,
                       IkValidator & validator,
                       const geometry_msgs::msg::Pose & target,
                       double vel_scale,
                       double acc_scale,
                       double free_axis_tolerance = 0.0,
                       double min_fraction = 0.995)
{
    CartesianOptions opts;
    opts.free_axis_tolerance = 0.0;

    robot_trajectory::RobotTrajectory traj(mg.getRobotModel(), mg.getName());

    double fraction = planRobustCartesian(mg, validator, target, opts, traj);
    std::cout << "    [cart] strict: " << (fraction * 100.0) << "%\n";

    if (fraction < min_fraction && free_axis_tolerance > 1e-6)
    {
        opts.free_axis_tolerance = free_axis_tolerance;
        robot_trajectory::RobotTrajectory relaxed(mg.getRobotModel(), mg.getName());
        const double f2 = planRobustCartesian(mg, validator, target, opts, relaxed);
        std::cout << "    [cart] free-axis relaxed: " << (f2 * 100.0) << "%\n";
        if (f2 > fraction) { fraction = f2; traj = relaxed; }
    }

    if (fraction >= min_fraction)
    {
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        if (!totg.computeTimeStamps(traj, vel_scale, acc_scale))
        {
            std::cout << "    [-] Time parameterisation failed.\n";
            return false;
        }
        moveit_msgs::msg::RobotTrajectory msg;
        traj.getRobotTrajectoryMsg(msg);
        return mg.execute(msg) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    std::cout << "    [*] Straight-line planning stalled; trying joint interpolation.\n";

    auto current = mg.getCurrentState(2.0);
    if (!current) return false;

    moveit::core::RobotState goal(*current);
    if (!solveNearestIk(mg, validator, target, *current, goal))
    {
        std::cout << "    [-] No collision-free IK at the target at all. The pose is\n"
                     "        unreachable, not merely hard to reach in a straight line.\n";
        return false;
    }

    return moveJointInterpolated(mg, validator, goal, vel_scale, acc_scale);
}

// ---------------------------------------------------------------------------
// Free-space move with obstacle avoidance, via a validated joint goal.
// ---------------------------------------------------------------------------
static bool moveFreeSpace(MGI & mg,
                          IkValidator & validator,
                          const geometry_msgs::msg::Pose & target,
                          double vel_scale,
                          double acc_scale)
{
    mg.setStartStateToCurrentState();
    mg.clearPoseTargets();
    mg.setMaxVelocityScalingFactor(vel_scale);
    mg.setMaxAccelerationScalingFactor(acc_scale);

    if (!setNearestJointGoal(mg, validator, target))
    {
        std::cout << "    [!] No collision-free IK; falling back to a pose goal.\n";
        mg.setPoseTarget(target);
    }

    MGI::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
        return false;

    // The OMPL pipeline already applies time-optimal parameterisation with the
    // scaling factors set above, so no manual retiming is needed here.
    return mg.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

// ---------------------------------------------------------------------------
// Backward-planned approach:
//   1. find a collision-free IK solution at the GRASP pose
//   2. seed the standoff IK from it, so both ends share an IK branch and the
//      arm cannot arrive at a standoff posture that runs out of joint range
//      part-way down
//   3. free-space transit to that standoff configuration
//   4. straight-line descent with the fallback ladder
// ---------------------------------------------------------------------------
static bool approachTarget(MGI & mg,
                           IkValidator & validator,
                           const geometry_msgs::msg::Pose & target,
                           double standoff_z,
                           double vel_scale,
                           double acc_scale,
                           double free_axis_tolerance = 0.0)
{
    geometry_msgs::msg::Pose standoff = target;
    standoff.position.z += standoff_z;

    auto current = mg.getCurrentState(2.0);
    if (!current)
    {
        std::cout << "    [-] Could not read robot state.\n";
        return false;
    }

    // ---- Step 1: is the FINAL pose reachable and collision-free? -----------
    // Failing here is much more useful than failing 90% of the way down.
    moveit::core::RobotState grasp_state(*current);
    if (!solveNearestIk(mg, validator, target, *current, grasp_state))
    {
        std::cout << "    [-] No collision-free IK at the target pose. "
                     "Aborting before moving anywhere.\n";
        return false;
    }

    // ---- Step 2: seed the standoff from the grasp solution ------------------
    moveit::core::RobotState standoff_state(grasp_state);
    if (!solveNearestIk(mg, validator, standoff, grasp_state, standoff_state, 8, 0.3))
    {
        std::cout << "    [!] Could not seed the standoff from the grasp solution; "
                     "retrying from the current state.\n";
        if (!solveNearestIk(mg, validator, standoff, *current, standoff_state))
        {
            std::cout << "    [-] No collision-free IK at the standoff pose.\n";
            return false;
        }
    }

    // ---- Step 3: transit ----------------------------------------------------
    mg.setStartStateToCurrentState();
    mg.clearPoseTargets();
    mg.setJointValueTarget(standoff_state);
    mg.setMaxVelocityScalingFactor(vel_scale);
    mg.setMaxAccelerationScalingFactor(acc_scale);

    std::cout << "    [1/2] Transit to standoff (z+" << standoff_z << ")...\n";
    MGI::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        std::cout << "    [-] Transit planning failed even with a validated goal. "
                     "The start state may itself be in collision -- check RViz.\n";
        return false;
    }
    if (mg.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        std::cout << "    [-] Transit execution failed.\n";
        return false;
    }
    waitForStateSettle(mg, 200);

    // ---- Step 4: descend ----------------------------------------------------
    std::cout << "    [2/2] Descent...\n";
    if (!moveLinear(mg, validator, target, vel_scale * 0.5, acc_scale * 0.5,
                    free_axis_tolerance))
    {
        std::cout << "    [-] Descent incomplete. Holding at standoff.\n";
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

    // Collision-aware IK. Must exist before any motion helper is called.
    IkValidator ik_validator(node);

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
    if (!moveFreeSpace(arm_interface, ik_validator, home_pose, VEL_SCALE_TRANSIT, ACC_SCALE_TRANSIT))
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
            if (moveFreeSpace(arm_interface, ik_validator, home_pose, v, a))
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

        if (!approachTarget(arm_interface, ik_validator, target, standoff, v, a,
                           TOOL_AXIS_FREEDOM))
        {
            arm_interface.clearPathConstraints();
            std::cout << "    [!] RECOVERY: returning home...\n";
            if (!moveFreeSpace(arm_interface, ik_validator, home_pose, VEL_SCALE_TRANSIT, ACC_SCALE_TRANSIT))
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
