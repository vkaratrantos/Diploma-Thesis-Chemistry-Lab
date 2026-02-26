#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h> 

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

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

  std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

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
  box_pose.position.z = -0.04; 
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  planning_scene_interface.applyCollisionObjects({collision_object});

  // --- ΚΑΡΤΕΣΙΑΝΗ ΚΙΝΗΣΗ ΜΕ ΑΝΑΖΗΤΗΣΗ ΣΕ ΚΥΒΟ 2cm ---
  auto try_direct_cartesian = [&](double target_x, double target_y, double target_z) -> bool {
      std::cout << "\n    [ΕΝΑΡΞΗ] Αναζήτηση Καρτεσιανής Ευθείας (με ανοχή 1cm στο XYZ)..." << std::endl;

      std::vector<double> offsets = {0.0, 0.01, -0.01};
      moveit_msgs::msg::RobotTrajectory traj;

      tf2::Quaternion q_target;
      q_target.setRPY(M_PI, -M_PI/2.0, -M_PI/2.0);

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
                  
                  double fraction = arm_interface.computeCartesianPath(waypoints, 0.01, 0.0, traj);

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

    std::cout << "Εντολές: M <id> (πχ M 1), P (Pick), O (Open), C (Close), ή x y z\nΕντολή: ";
    std::string line;
    std::getline(std::cin, line);
    if (line.empty()) continue;

    if (line == "O" || line == "o") { gripper_interface.setNamedTarget("open"); gripper_interface.move(); continue; } 
    if (line == "C" || line == "c") { gripper_interface.setNamedTarget("closed"); gripper_interface.move(); continue; }

    // --- ΕΝΤΟΛΗ: ΜΕΤΑΒΑΣΗ ΣΕ MARKER ---
    if (line[0] == 'M' || line[0] == 'm') {
        int marker_id;
        std::stringstream ss(line.substr(1));
        if (ss >> marker_id && marker_id >= 0 && marker_id <= 9) {
            
            std::string target_frame = (marker_id == 0) ? "marker_base" : "marker_" + std::to_string(marker_id);
            std::string planning_frame = arm_interface.getPlanningFrame();

            try {
                geometry_msgs::msg::TransformStamped t = tf_buffer->lookupTransform(
                    planning_frame, target_frame, tf2::TimePointZero, std::chrono::seconds(1)
                );

                double target_x = t.transform.translation.x + 0.02; // +2 cm στον άξονα X
                double target_y = t.transform.translation.y + 0.15; // +15 cm στον άξονα Y
                double target_z = 0.20;                             // 20 cm ύψος σταθερά (Ζ)

                std::cout << "\n>>> Εντοπίστηκε το Marker " << marker_id 
                          << ". Εφαρμογή Offsets (X+0.02, Y+0.15, Z=0.20)..." << std::endl;
                std::cout << ">>> Τελικός Στόχος: X=" << target_x << ", Y=" << target_y << ", Z=" << target_z << std::endl;
                
                try_direct_cartesian(target_x, target_y, target_z);

            } catch (const tf2::TransformException & ex) {
                std::cout << "\n[-] ΣΦΑΛΜΑ: Το Marker " << marker_id << " δεν φαίνεται στην κάμερα αυτή τη στιγμή." << std::endl;
                std::cout << "Λεπτομέρειες: " << ex.what() << std::endl;
            }
        } else {
            std::cout << "\n[-] Λάθος εισαγωγή. Η σωστή μορφή είναι 'M 1' για το Marker 1 (επιτρεπτά 0-9)." << std::endl;
        }
        continue;
    }

    // --- ΝΕΑ ΕΝΤΟΛΗ: PICK (Στην τρέχουσα θέση με βάθος 15cm) ---
    if (line == "P" || line == "p") {
        std::cout << "\n>>> ΕΚΤΕΛΕΣΗ PICK ΣΤΗΝ ΤΡΕΧΟΥΣΑ ΘΕΣΗ..." << std::endl;
        
        // Λήψη των ακριβών συντεταγμένων της τρέχουσας θέσης πριν κάνουμε οτιδήποτε
        geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
        double cur_x = current_pose.position.x;
        double cur_y = current_pose.position.y;
        double cur_z = current_pose.position.z;

        // 1. Άνοιγμα Gripper (ΠΡΙΝ ΚΑΤΕΒΕΙ)
        std::cout << ">>> 1. Άνοιγμα Gripper..." << std::endl;
        gripper_interface.setNamedTarget("open"); 
        gripper_interface.move();
        
        // 2. Κάθοδος κατά 15 cm (0.15)
        std::cout << ">>> 2. Κάθοδος κατά 15 cm..." << std::endl;
        if(try_direct_cartesian(cur_x, cur_y, cur_z - 0.15)) {
            
            // 3. Κλείσιμο Gripper (ΜΟΛΙΣ ΚΑΤΕΒΕΙ)
            std::cout << ">>> 3. Κλείσιμο Gripper..." << std::endl;
            gripper_interface.setNamedTarget("closed"); 
            gripper_interface.move();
            
            // 4. Άνοδος στην αρχική θέση (επαναφορά στα προηγούμενα +15 cm)
            std::cout << ">>> 4. Άνοδος 15 cm (Επιστροφή)..." << std::endl;
            try_direct_cartesian(cur_x, cur_y, cur_z);
            
            std::cout << ">>> Η διαδικασία Pick ολοκληρώθηκε επιτυχώς!" << std::endl;
        } else {
            std::cout << "[-] Αποτυχία καθόδου. Πιθανόν το βάθος Z=" << (cur_z - 0.15) << " να παραβιάζει το όριο του πατώματος (-2cm)!" << std::endl;
        }
        continue;
    }

    // --- ΕΝΤΟΛΗ: ΑΠΛΗ ΜΕΤΑΒΑΣΗ ΣΕ X Y Z ---
    std::stringstream ss(line); double tx, ty, tz;
    if (ss >> tx >> ty >> tz) try_direct_cartesian(tx, ty, tz);
  }
  rclcpp::shutdown();
  return 0;
}
