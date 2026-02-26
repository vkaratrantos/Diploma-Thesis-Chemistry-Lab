#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h> 

void printPose(const geometry_msgs::msg::Pose& pose) {
    double r, p, y;
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(r, p, y);
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[ΤΡΕΧΟΥΣΑ ΘΕΣΗ]: X=" << pose.position.x 
              << ", Y=" << pose.position.y 
              << ", Z=" << pose.position.z 
              << " | RPY (deg): " << (r * 180.0 / M_PI) << ", " 
              << (p * 180.0 / M_PI) << ", " 
              << (y * 180.0 / M_PI) << std::endl;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("cartesian_cube_search_node");

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread([executor]() { executor->spin(); }).detach();

  using moveit::planning_interface::MoveGroupInterface;
  MoveGroupInterface arm_interface(node, "arm_group"); 
  MoveGroupInterface gripper_interface(node, "gripper"); 

  arm_interface.setMaxVelocityScalingFactor(0.3); 
  arm_interface.setMaxAccelerationScalingFactor(0.3);

  // --- ΠΡΟΣΘΗΚΗ ΠΑΤΩΜΑΤΟΣ ---
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = arm_interface.getPlanningFrame();
  collision_object.id = "floor";
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {2.0, 2.0, 0.04}; 
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.z = -0.04; // Η πάνω επιφάνεια του εμποδίου είναι στα -2cm
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  planning_scene_interface.applyCollisionObjects({collision_object});

  // --- ΚΑΡΤΕΣΙΑΝΗ ΚΙΝΗΣΗ ΜΕ ΑΝΑΖΗΤΗΣΗ ΣΕ ΚΥΒΟ 2cm ---
  auto try_direct_cartesian = [&](double target_x, double target_y, double target_z) -> bool {
      std::cout << "\n    [ΕΝΑΡΞΗ] Αναζήτηση Καρτεσιανής Ευθείας (με ανοχή 1cm στο XYZ)..." << std::endl;

      // Πιθανές αποκλίσεις για τον κύβο (0cm, +1cm, -1cm)
      std::vector<double> offsets = {0.0, 0.01, -0.01};
      moveit_msgs::msg::RobotTrajectory traj;

      // Σταθερός προσανατολισμός για την κίνηση (Roll=180, Pitch=-90)
      // Το Yaw παραμένει -90 (ή 270) όπως στην αρχική θέση
      tf2::Quaternion q_target;
      q_target.setRPY(M_PI, -M_PI/2.0, -M_PI/2.0);

      // Εξαντλητική αναζήτηση στα 27 σημεία του κύβου (3x3x3)
      for (double dx : offsets) {
          for (double dy : offsets) {
              for (double dz : offsets) {
                  geometry_msgs::msg::Pose target;
                  target.position.x = target_x + dx;
                  target.position.y = target_y + dy;
                  target.position.z = target_z + dz;
                  target.orientation.x = q_target.x();
                  target.orientation.y = q_target.y();
                  target.orientation.z = q_target.z();
                  target.orientation.w = q_target.w();

                  std::vector<geometry_msgs::msg::Pose> waypoints = {target};
                  
                  // Υπολογισμός Καρτεσιανής Διαδρομής
                  double fraction = arm_interface.computeCartesianPath(waypoints, 0.01, 0.0, traj);

                  // Αν βρει λύση έστω και για το 95% της διαδρομής, την εκτελεί
                  if (fraction >= 0.95) {
                      std::cout << "    [+] Βρέθηκε λύση! Απόκλιση (dx, dy, dz): (" 
                                << dx*100 << "cm, " << dy*100 << "cm, " << dz*100 << "cm)" << std::endl;
                      
                      moveit::planning_interface::MoveGroupInterface::Plan plan;
                      plan.trajectory_ = traj;
                      arm_interface.execute(plan);
                      return true;
                  }
              }
          }
      }

      std::cout << "    [-] Αποτυχία: Δεν βρέθηκε εφικτή Καρτεσιανή ευθεία εντός του κύβου των 2cm." << std::endl;
      return false;
  };

  RCLCPP_INFO(node->get_logger(), "Εκκίνηση διαδικασίας...");

  // --- ΑΡΧΙΚΟΠΟΙΗΣΗ ΣΤΗ ΝΕΑ ΘΕΣΗ HOME (-0.15, -0.2, 0.2) ---
  std::cout << ">>> Μετάβαση σε Home (-0.15, -0.2, 0.2) με ελεύθερο προσανατολισμό..." << std::endl;
  arm_interface.clearPoseTargets();
  arm_interface.setPositionTarget(-0.15, -0.2, 0.2); 
  
  if (arm_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Αποτυχία μετάβασης στην αρχική θέση!");
  }

  std::cout << ">>> Σύστημα Έτοιμο: Ενεργός Αλγόριθμος Cartesian Cube Search (2cm)." << std::endl;

  while (rclcpp::ok()) {
    std::cout << "\n================================================" << std::endl;
    arm_interface.setStartStateToCurrentState();
    printPose(arm_interface.getCurrentPose().pose);

    std::cout << "Εντολές: P x y z, O, C, ή απλά x y z\nΕντολή: ";
    std::string line;
    std::getline(std::cin, line);
    if (line.empty()) continue;

    if (line == "O" || line == "o") { gripper_interface.setNamedTarget("open"); gripper_interface.move(); continue; } 
    if (line == "C" || line == "c") { gripper_interface.setNamedTarget("closed"); gripper_interface.move(); continue; }

    if (line[0] == 'P' || line[0] == 'p') {
        std::stringstream ss(line.substr(1)); double px, py, pz;
        if (ss >> px >> py >> pz) {
            std::cout << "\n>>> ΕΚΤΕΛΕΣΗ PICK ΣΤΟ (" << px << ", " << py << ", " << pz << ")" << std::endl;
            if(!try_direct_cartesian(px, py, pz + 0.10)) continue;
            gripper_interface.setNamedTarget("open"); gripper_interface.move();
            if(!try_direct_cartesian(px, py, pz)) continue;
            gripper_interface.setNamedTarget("closed"); gripper_interface.move();
            try_direct_cartesian(px, py, pz + 0.10);
        }
    } else {
        std::stringstream ss(line); double tx, ty, tz;
        if (ss >> tx >> ty >> tz) try_direct_cartesian(tx, ty, tz);
    }
  }
  rclcpp::shutdown();
  return 0;
}
