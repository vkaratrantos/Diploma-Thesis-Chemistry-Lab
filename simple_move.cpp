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

  // --- 1. ΕΥΕΛΙΚΤΗ ΜΕΤΑΒΑΣΗ (ΓΙΑ ΠΡΟΣΕΓΓΙΣΗ ΣΤΟΧΩΝ X, Y) ---
  auto try_direct_cartesian = [&](double target_x, double target_y, double target_z) -> bool {
      std::vector<double> offsets = {0.0, 0.01, -0.01};
      moveit_msgs::msg::RobotTrajectory traj;
      geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;

      for (double dx : offsets) {
          for (double dy : offsets) {
              for (double dz : offsets) {
                  geometry_msgs::msg::Pose target;
                  target.position.x = target_x + dx;
                  target.position.y = target_y + dy;
                  target.position.z = target_z + dz;
                  target.orientation = current_pose.orientation;

                  std::vector<geometry_msgs::msg::Pose> waypoints = {target};
                  double fraction = arm_interface.computeCartesianPath(waypoints, 0.01, 0.0, traj);

                  if (fraction >= 0.95) {
                      moveit::planning_interface::MoveGroupInterface::Plan plan;
                      plan.trajectory_ = traj;
                      arm_interface.execute(plan);
                      return true;
                  }
              }
          }
      }
      std::cout << "    [-] Αποτυχία μετακίνησης στο σημείο." << std::endl;
      return false;
  };

  // --- 2. ΑΥΣΤΗΡΗ ΚΑΘΕΤΗ ΚΙΝΗΣΗ ---
  auto strict_vertical_move = [&](double delta_z) -> bool {
      std::cout << "    [STRICT VERTICAL] Κίνηση Z κατά " << delta_z * 100 << " cm..." << std::endl;
      
      geometry_msgs::msg::Pose target_pose = arm_interface.getCurrentPose().pose;
      target_pose.position.z += delta_z; 

      std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
      moveit_msgs::msg::RobotTrajectory traj;
      
      double fraction = arm_interface.computeCartesianPath(waypoints, 0.005, 0.0, traj);

      if (fraction >= 0.99) { 
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          plan.trajectory_ = traj;
          arm_interface.execute(plan); 
          return true;
      } else {
          std::cout << "    [-] Αποτυχία αυστηρής κάθετης κίνησης (Ασφαλές Μπλοκάρισμα)." << std::endl;
          return false;
      }
  };

  // --- 3. ΝΕΑ ΣΥΝΑΡΤΗΣΗ: ΤΟΠΙΚΗ ΠΕΡΙΣΤΡΟΦΗ ΜΕ QUATERNIONS (ΑΠΟΣΟΒΗΣΗ GIMBAL LOCK) ---
  auto strict_rotation_move = [&](double delta_roll) -> bool {
      std::cout << "    [STRICT ROTATION] Τοπική περιστροφή Roll κατά " << delta_roll * 180.0 / M_PI << " μοίρες..." << std::endl;
      
      geometry_msgs::msg::Pose target_pose = arm_interface.getCurrentPose().pose;
      
      // Διαβάζουμε τον τρέχοντα προσανατολισμό κατευθείαν ως Quaternion
      tf2::Quaternion q_current(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
      
      std::vector<geometry_msgs::msg::Pose> waypoints;

      // Σπάμε τη μεγάλη περιστροφή σε 5 ομαλά ενδιάμεσα βήματα (waypoints)
      int steps = 5;
      for (int i = 1; i <= steps; ++i) {
          double step_delta = delta_roll * i / steps; 
          
          // Δημιουργούμε την "καθαρή" τοπική περιστροφή γύρω από τον άξονα X (Roll)
          tf2::Quaternion q_rot;
          q_rot.setRPY(step_delta, 0.0, 0.0);
          
          // Πολλαπλασιάζουμε τα Quaternions για να εφαρμόσουμε την περιστροφή ΣΤΟ ΤΟΠΙΚΟ ΣΥΣΤΗΜΑ της αρπάγης
          tf2::Quaternion q_step = q_current * q_rot;
          q_step.normalize(); // Κανονικοποίηση για αποφυγή μαθηματικών σφαλμάτων
          
          geometry_msgs::msg::Pose wp = target_pose;
          wp.orientation.x = q_step.x();
          wp.orientation.y = q_step.y();
          wp.orientation.z = q_step.z();
          wp.orientation.w = q_step.w();
          
          waypoints.push_back(wp);
      }

      moveit_msgs::msg::RobotTrajectory traj;
      
      // Υπολογισμός Καρτεσιανής Διαδρομής πάνω στα 5 βήματα
      double fraction = arm_interface.computeCartesianPath(waypoints, 0.005, 0.0, traj);

      if (fraction >= 0.99) { 
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          plan.trajectory_ = traj;
          arm_interface.execute(plan); 
          return true;
      } else {
          std::cout << "    [-] Αποτυχία. Ο καρπός μάλλον χτυπάει μηχανικό όριο!" << std::endl;
          return false;
      }
  };

  RCLCPP_INFO(node->get_logger(), "Εκκίνηση διαδικασίας...");

  std::cout << ">>> Μετάβαση σε Home (-0.15, -0.2, 0.2) με ΚΑΘΕΤΟ προσανατολισμό..." << std::endl;
  arm_interface.clearPoseTargets();
  
  geometry_msgs::msg::Pose home_pose;
  home_pose.position.x = -0.15;
  home_pose.position.y = -0.2;
  home_pose.position.z = 0.2;

  tf2::Quaternion q_home;
  q_home.setRPY(M_PI, -M_PI/2.0, -M_PI/2.0);
  home_pose.orientation.x = q_home.x();
  home_pose.orientation.y = q_home.y();
  home_pose.orientation.z = q_home.z();
  home_pose.orientation.w = q_home.w();

  arm_interface.setPoseTarget(home_pose);
  if (arm_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Αποτυχία μετάβασης στην αρχική θέση!");
  }

  std::cout << ">>> Σύστημα Έτοιμο." << std::endl;

  while (rclcpp::ok()) {
    std::cout << "\n================================================" << std::endl;
    arm_interface.setStartStateToCurrentState();
    printPose(arm_interface.getCurrentPose().pose);

    std::cout << "Εντολές: M <id>, P (Pick), D (Drop), pour (Pour), O (Open), C (Close)\nΕντολή: ";
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

                double target_x = t.transform.translation.x + 0.02; // Προεπιλογή +2 cm X
                double target_y = t.transform.translation.y + 0.15; // Προεπιλογή +15 cm Y
                double target_z = 0.20;                             // Προεπιλογή 20 cm Z

                // --- ΕΙΔΙΚΟΣ ΚΑΝΟΝΑΣ ΓΙΑ MARKER 4 ---
                if (marker_id == 4) {
                    target_x = t.transform.translation.x + 0.08; // Ειδικό offset X +8 cm
                    target_z = 0.20;                             // 20 cm πάνω από το έδαφος
                    std::cout << "\n>>> [Ειδικός Κανόνας ID 4] Εφαρμογή Offsets (X+0.08, Y+0.15, Z=0.20)..." << std::endl;
                } else {
                    std::cout << "\n>>> Εντοπίστηκε το Marker " << marker_id << ". Εφαρμογή Κανονικών Offsets..." << std::endl;
                }

                try_direct_cartesian(target_x, target_y, target_z);

            } catch (const tf2::TransformException & ex) {
                std::cout << "\n[-] ΣΦΑΛΜΑ: Το Marker " << marker_id << " δεν φαίνεται." << std::endl;
            }
        }
        continue;
    }

    // --- ΕΝΤΟΛΗ: POUR (Άδειασμα) ---
    if (line == "pour" || line == "POUR" || line == "Pour") {
        std::cout << "\n>>> ΕΚΤΕΛΕΣΗ POUR (ΠΕΡΙΣΤΡΟΦΗ END-EFFECTOR)..." << std::endl;
        
        std::cout << ">>> 1. Γύρισμα καρπού 90 μοίρες (Roll)..." << std::endl;
        // Το -M_PI/2.0 όπως είχες βρει ότι δουλεύει για τα όρια του δικού σου ρομπότ
        if(strict_rotation_move(-M_PI / 2.0)) { 
            
            std::cout << ">>> [Αναμονή 1.5 δευτερόλεπτο για άδειασμα του υγρού/υλικού]..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            
            std::cout << ">>> 2. Επαναφορά καρπού (Ίσιωμα)..." << std::endl;
            // Η επαναφορά γίνεται με το αντίθετο πρόσημο
            strict_rotation_move(M_PI / 2.0); 
            
            std::cout << ">>> Η διαδικασία Pour ολοκληρώθηκε επιτυχώς!" << std::endl;
        }
        continue;
    }

    // --- ΕΝΤΟΛΗ: ΑΥΣΤΗΡΟ PICK ---
    if (line == "P" || line == "p") {
        std::cout << "\n>>> ΕΚΤΕΛΕΣΗ PICK (ΣΤΗΝ ΤΡΕΧΟΥΣΑ ΘΕΣΗ X, Y)..." << std::endl;
        
        std::cout << ">>> 1. Άνοιγμα Gripper..." << std::endl;
        gripper_interface.setNamedTarget("open"); 
        gripper_interface.move(); 
        
        std::cout << ">>> 2. Αυστηρή Κάθοδος 10 cm..." << std::endl;
        if(strict_vertical_move(-0.10)) { 
            
            std::cout << ">>> [Αναμονή 1 δευτερόλεπτο για σταθεροποίηση]..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::cout << ">>> 3. Κλείσιμο Gripper..." << std::endl;
            gripper_interface.setNamedTarget("closed"); 
            gripper_interface.move(); 
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            std::cout << ">>> 4. Αυστηρή Άνοδος 10 cm..." << std::endl;
            strict_vertical_move(0.10); 
            
            std::cout << ">>> Το Pick ολοκληρώθηκε!" << std::endl;
        }
        continue;
    }

    // --- ΕΝΤΟΛΗ: ΑΥΣΤΗΡΟ DROP / PLACE ---
    if (line == "D" || line == "d") {
        std::cout << "\n>>> ΕΚΤΕΛΕΣΗ DROP/PLACE (ΣΤΗΝ ΤΡΕΧΟΥΣΑ ΘΕΣΗ X, Y)..." << std::endl;
        
        std::cout << ">>> 1. Αυστηρή Κάθοδος 10 cm..." << std::endl;
        if(strict_vertical_move(-0.10)) { 
            
            std::cout << ">>> [Αναμονή 1 δευτερόλεπτο για σταθεροποίηση]..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            std::cout << ">>> 2. Άνοιγμα Gripper (Απελευθέρωση)..." << std::endl;
            gripper_interface.setNamedTarget("open"); 
            gripper_interface.move(); 
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            std::cout << ">>> 3. Αυστηρή Άνοδος 10 cm..." << std::endl;
            strict_vertical_move(0.10); 

            std::cout << ">>> 4. Κλείσιμο Gripper..." << std::endl;
            gripper_interface.setNamedTarget("closed"); 
            gripper_interface.move();
            
            std::cout << ">>> Το Drop/Place ολοκληρώθηκε!" << std::endl;
        }
        continue;
    }

    // --- ΕΝΤΟΛΗ: ΑΠΛΗ ΜΕΤΑΒΑΣΗ ---
    std::stringstream ss(line); double tx, ty, tz;
    if (ss >> tx >> ty >> tz) try_direct_cartesian(tx, ty, tz);
  }
  rclcpp::shutdown();
  return 0;
}
