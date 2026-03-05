#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <chrono>
#include <algorithm> 
#include <functional> // ΥΠΟΧΡΕΩΤΙΚΟ ΓΙΑ THN ΑΝΑΔΡΟΜΗ (Recursion)

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
  
  arm_interface.setPlanningTime(1.0);
  arm_interface.setMaxVelocityScalingFactor(0.3); 
  arm_interface.setMaxAccelerationScalingFactor(0.3);

  // --- Adding the floor as collision ---
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

  // --- 1. COARSE-TO-FINE CARTESIAN SEARCH WITH UNIVERSAL BRIDGE (Recursion) ---
  std::function<bool(double, double, bool)> try_direct_cartesian;
  try_direct_cartesian = [&](double target_x, double target_y, bool allow_bridge) -> bool {
    geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
    double current_z = current_pose.position.z;
    moveit_msgs::msg::RobotTrajectory traj;

    std::vector<double> coarse_z = {0.20, 0.24, 0.28, 0.30};
    std::sort(coarse_z.begin(), coarse_z.end(), [current_z](double a, double b) {
        return std::abs(a - current_z) < std::abs(b - current_z);
    });

    std::vector<double> coarse_roll = {0.0, 0.261, -0.261, 0.523, -0.523}; 

    struct Offset { double dx; double dy; };
    std::vector<Offset> xy_offsets = {
        {0.0, 0.0}, {0.005, 0.0}, {-0.005, 0.0}, {0.0, 0.005}, {0.0, -0.005}
    };

    double best_fraction = 0.0;
    double best_z = current_z;
    double best_roll = 0.0;
    
    std::cout << "    [*] Ξεκινάει Γρήγορη Σάρωση (Coarse Search)..." << std::endl;

    for (double test_z : coarse_z) {
        for (double test_roll : coarse_roll) {
            for (const auto& offset : xy_offsets) {
                geometry_msgs::msg::Pose target;
                target.position.x = target_x + offset.dx;
                target.position.y = target_y + offset.dy;
                target.position.z = test_z; 

                tf2::Quaternion q_current(current_pose.orientation.x, current_pose.orientation.y, 
                                          current_pose.orientation.z, current_pose.orientation.w);
                tf2::Quaternion q_rot;
                q_rot.setRPY(test_roll, 0, 0); 
                tf2::Quaternion q_target = q_current * q_rot;
                q_target.normalize();

                target.orientation.x = q_target.x(); target.orientation.y = q_target.y();
                target.orientation.z = q_target.z(); target.orientation.w = q_target.w();

                std::vector<geometry_msgs::msg::Pose> waypoints = {target};
                
                double fraction = arm_interface.computeCartesianPath(waypoints, 0.04, 1.5, traj);

                if (fraction >= 0.95) { 
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = traj;
                    if (arm_interface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                        std::cout << "    [+] Βρέθηκε άμεσα! Z=" << test_z*100 
                                  << "cm, Roll=" << test_roll*180/M_PI << "°" << std::endl;
                        return true;
                    }
                }
                
                if (fraction > best_fraction) {
                    best_fraction = fraction;
                    best_z = test_z;
                    best_roll = test_roll;
                }
            }
        }
    }

    if (best_fraction > 0.3) { 
        std::cout << "    [*] Η γρήγορη σάρωση έφτασε στο " << best_fraction*100 
                  << "%. Βαθύτερος έλεγχος γύρω από Z=" << best_z*100 
                  << "cm και Roll=" << best_roll*180/M_PI << "°..." << std::endl;
        
        for (double fine_z = best_z - 0.02; fine_z <= best_z + 0.021; fine_z += 0.01) {
            if (fine_z < 0.20 || fine_z > 0.30) continue; 

            for (double fine_roll = best_roll - 0.174; fine_roll <= best_roll + 0.175; fine_roll += 0.087) {
                if (fine_roll < -0.524 || fine_roll > 0.524) continue;

                for (const auto& offset : xy_offsets) {
                    geometry_msgs::msg::Pose target;
                    target.position.x = target_x + offset.dx;
                    target.position.y = target_y + offset.dy;
                    target.position.z = fine_z; 

                    tf2::Quaternion q_current(current_pose.orientation.x, current_pose.orientation.y, 
                                              current_pose.orientation.z, current_pose.orientation.w);
                    tf2::Quaternion q_rot;
                    q_rot.setRPY(fine_roll, 0, 0); 
                    tf2::Quaternion q_target = q_current * q_rot;
                    q_target.normalize();

                    target.orientation.x = q_target.x(); target.orientation.y = q_target.y();
                    target.orientation.z = q_target.z(); target.orientation.w = q_target.w();

                    std::vector<geometry_msgs::msg::Pose> waypoints = {target};
                    
                    double fraction = arm_interface.computeCartesianPath(waypoints, 0.01, 1.5, traj);

                    if (fraction >= 0.95) {
                        moveit::planning_interface::MoveGroupInterface::Plan plan;
                        plan.trajectory_ = traj;
                        if (arm_interface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                            std::cout << "    [+] Επιτυχία (Fine Search)! Z=" << fine_z*100 
                                      << "cm, Roll=" << fine_roll*180/M_PI << "°" << std::endl;
                            return true;
                        }
                    }
                }
            }
        }
    }
    
    // =========================================================================
    // THE UNIVERSAL BRIDGE FALLBACK (Recursion logic)
    // =========================================================================
    if (allow_bridge) {
        std::cout << "    [-] Αποτυχία απευθείας μετάβασης. Δοκιμή μέσω Κεντρικής Γέφυρας (X=0.0, Y=-0.2)..." << std::endl;
        
        // 1. Προσωρινή μετάβαση στη γέφυρα (pass 'false' to prevent infinite loops)
        if (try_direct_cartesian(0.0, -0.2, false)) {
            std::cout << "    [UNIVERSAL BRIDGE] Το ρομπότ έφτασε στη γέφυρα. Προσπάθεια για τον τελικό στόχο..." << std::endl;
            
            // 2. Από τη γέφυρα, πάμε στον τελικό στόχο!
            if (try_direct_cartesian(target_x, target_y, false)) {
                std::cout << "    [UNIVERSAL BRIDGE] Επιτυχία! Ο τελικός στόχος προσεγγίστηκε μέσω της γέφυρας." << std::endl;
                return true;
            } else {
                std::cout << "    [UNIVERSAL BRIDGE] ΣΦΑΛΜΑ: Το ρομπότ έφτασε στη γέφυρα αλλά αδυνατεί να βρει τον στόχο." << std::endl;
                return false;
            }
        } else {
            std::cout << "    [UNIVERSAL BRIDGE] ΣΦΑΛΜΑ: Το ρομπότ απέτυχε να φτάσει ακόμα και στη Γέφυρα!" << std::endl;
        }
    }

    return false;
  };

  // --- 2. Strict Vertical Move for Pick/Drop ---
  auto strict_vertical_move = [&](double delta_z) -> bool {
      std::cout << "    [Vertical Move] Moving vertically " << delta_z * 100 << " cm..." << std::endl;
      
      geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
      std::vector<double> offsets = {0.0, 0.0025, -0.0025, 0.005, -0.005};
      moveit_msgs::msg::RobotTrajectory traj;

      for (double dx : offsets) {
          for (double dy : offsets) {
              for (double dz_offset : offsets) {
                  double distance = std::sqrt(dx*dx + dy*dy + dz_offset*dz_offset);
                  if (distance > 0.0051) continue;

                  geometry_msgs::msg::Pose target_pose = current_pose;
                  target_pose.position.x += dx;
                  target_pose.position.y += dy;
                  target_pose.position.z += (delta_z + dz_offset); 

                  std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
                  
                  double fraction = arm_interface.computeCartesianPath(waypoints, 0.005, 1.5, traj);

                  if (fraction >= 0.99) { 
                      moveit::planning_interface::MoveGroupInterface::Plan plan;
                      plan.trajectory_ = traj;
                      arm_interface.execute(plan); 
                      
                      if (distance > 0.0) {
                          std::cout << "    [+] Βρέθηκε κάθετη διαδρομή με μικρή απόκλιση: dx=" 
                                    << dx << "m, dy=" << dy << "m, dz_offset=" << dz_offset << "m" << std::endl;
                      }
                      return true;
                  }
              }
          }
      }

      std::cout << "    [-] Failed to move vertically, even with offsets." << std::endl;
      return false;
  };

  // --- 3. Rotating the last joint ---
  auto rotate_last_joint = [&](double delta_angle) -> bool {
      std::cout << "    [JOINT ROTATION] Rotating the last joint by " 
                << delta_angle * 180.0 / M_PI << " degrees." << std::endl;

      std::vector<double> joint_values = arm_interface.getCurrentJointValues();

      if (joint_values.empty()) {
          std::cout << "    [-] Error: Joint values not found!" << std::endl;
          return false;
      }

      int last_joint_index = joint_values.size() - 1;
      joint_values[last_joint_index] += delta_angle;

      arm_interface.setJointValueTarget(joint_values);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (arm_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success) {
          arm_interface.execute(plan);
          return true;
      } else {
          std::cout << "    [-] Error. The joint hits a joint limit!" << std::endl;
          return false;
      }
  };

  RCLCPP_INFO(node->get_logger(), "Starting the procedure...");

  std::cout << ">>> Heading to Home Position (-0.15, -0.2, 0.25) with vertical orientation..." << std::endl;
  arm_interface.clearPoseTargets();
  
  geometry_msgs::msg::Pose home_pose;
  home_pose.position.x = -0.15;
  home_pose.position.y = -0.2;
  home_pose.position.z = 0.25; 

  tf2::Quaternion q_home;
  q_home.setRPY(M_PI, -M_PI/2.0, -M_PI/2.0);
  home_pose.orientation.x = q_home.x();
  home_pose.orientation.y = q_home.y();
  home_pose.orientation.z = q_home.z();
  home_pose.orientation.w = q_home.w();

  arm_interface.setPoseTarget(home_pose);
  if (arm_interface.move() != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Failed to go to Home Position!");
  }

  std::cout << ">>> The system is ready." << std::endl;

  while (rclcpp::ok()) {
    std::cout << "\n================================================" << std::endl;
    arm_interface.setStartStateToCurrentState();
    printPose(arm_interface.getCurrentPose().pose);

    std::cout << "Commands: M <id>, P (Pick), D (Drop), pour (Pour), O (Open), C (Close), <X> <Y> (Go to coordinates)\nCommand: ";
    std::string line;
    std::getline(std::cin, line);
    if (line.empty()) continue;

    if (line == "O" || line == "o") { gripper_interface.setNamedTarget("open"); gripper_interface.move(); continue; } 
    if (line == "C" || line == "c") { gripper_interface.setNamedTarget("closed"); gripper_interface.move(); continue; }

    // --- Command: Going to Marker ---
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

                double target_x = t.transform.translation.x; 
                double target_y = t.transform.translation.y; 

                std::cout << "\n>>> Εντοπίστηκε το Marker " << marker_id 
                          << ". Στόχος: X=" << target_x << ", Y=" << target_y << std::endl;

                // Περνάμε true για να επιτρέψουμε τη χρήση της γενικής γέφυρας αν χρειαστεί
                if (!try_direct_cartesian(target_x, target_y, true)) {
                    std::cout << "    [-] Αποτυχία απευθείας μετάβασης. Δοκιμή μέσω ενδιάμεσων Markers (Bridge)..." << std::endl;
                    
                    std::vector<int> fallback_markers = {3, 0, 2}; 
                    bool success_via_bridge = false;

                    for (int bridge_id : fallback_markers) {
                        if (bridge_id == marker_id) continue; 

                        std::string bridge_frame = (bridge_id == 0) ? "marker_base" : "marker_" + std::to_string(bridge_id);
                        try {
                            geometry_msgs::msg::TransformStamped t_bridge = tf_buffer->lookupTransform(
                                planning_frame, bridge_frame, tf2::TimePointZero, std::chrono::milliseconds(200)
                            );
                            
                            double bridge_x = t_bridge.transform.translation.x;
                            double bridge_y = t_bridge.transform.translation.y;

                            std::cout << "    [MARKER BRIDGE] Δοκιμή μετάβασης στο Marker " << bridge_id << "..." << std::endl;
                            
                            // Εδώ βάζουμε false για να μην κάνει διπλή αναδρομή
                            if (try_direct_cartesian(bridge_x, bridge_y, false)) {
                                std::cout << "    [MARKER BRIDGE] Επιτυχία! Τώρα προσπάθεια για τον τελικό στόχο (Marker " << marker_id << ")..." << std::endl;
                                
                                if (try_direct_cartesian(target_x, target_y, false)) {
                                    success_via_bridge = true;
                                    break; 
                                } else {
                                    std::cout << "    [MARKER BRIDGE] Αποτυχία από τη γέφυρα προς τον στόχο. Θα δοκιμαστεί άλλη εναλλακτική." << std::endl;
                                }
                            }
                        } catch (const tf2::TransformException & ex) {
                            continue; 
                        }
                    }

                    if (!success_via_bridge) {
                        std::cout << "    [-] Τελική Αποτυχία. Δεν βρέθηκε διαδρομή ούτε μέσω ενδιάμεσων Markers." << std::endl;
                    }
                }

            } catch (const tf2::TransformException & ex) {
                std::cout << "\n[-] ΣΦΑΛΜΑ: Το Marker " << marker_id << " δεν φαίνεται στην κάμερα." << std::endl;
            }
        }
        continue;
    }

    // --- ΕΝΤΟΛΗ: POUR (Άδειασμα) ---
    if (line == "pour" || line == "POUR" || line == "Pour") {
        std::cout << "\n>>> ΕΚΤΕΛΕΣΗ POUR (ΠΕΡΙΣΤΡΟΦΗ ΜΟΝΟ ΤΕΛΕΥΤΑΙΑΣ ΑΡΘΡΩΣΗΣ)..." << std::endl;
        
        std::cout << ">>> 1. Γύρισμα καρπού 90 μοίρες (Αριστερόστροφα / CCW)..." << std::endl;
        if(rotate_last_joint(M_PI / 2.0)) { 
            
            std::cout << ">>> [Αναμονή 1.5 δευτερόλεπτο για άδειασμα του υγρού/υλικού]..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            
            std::cout << ">>> 2. Επαναφορά καρπού (Δεξιόστροφα / CW)..." << std::endl;
            rotate_last_joint(-M_PI / 2.0); 
            
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
        if(strict_vertical_move(-0.15)) { 
            
            std::cout << ">>> [Αναμονή 1 δευτερόλεπτο για σταθεροποίηση]..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::cout << ">>> 3. Κλείσιμο Gripper..." << std::endl;
            gripper_interface.setNamedTarget("closed"); 
            gripper_interface.move(); 
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            std::cout << ">>> 4. Αυστηρή Άνοδος 10 cm..." << std::endl;
            strict_vertical_move(0.15); 
            
            std::cout << ">>> Το Pick ολοκληρώθηκε!" << std::endl;
        }
        continue;
    }

    // --- ΕΝΤΟΛΗ: ΑΥΣΤΗΡΟ DROP / PLACE ---
    if (line == "D" || line == "d") {
        std::cout << "\n>>> ΕΚΤΕΛΕΣΗ DROP/PLACE (ΣΤΗΝ ΤΡΕΧΟΥΣΑ ΘΕΣΗ X, Y)..." << std::endl;
        
        std::cout << ">>> 1. Αυστηρή Κάθοδος 10 cm..." << std::endl;
        if(strict_vertical_move(-0.15)) { 
            
            std::cout << ">>> [Αναμονή 1 δευτερόλεπτο για σταθεροποίηση]..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            std::cout << ">>> 2. Άνοιγμα Gripper (Απελευθέρωση)..." << std::endl;
            gripper_interface.setNamedTarget("open"); 
            gripper_interface.move(); 
            
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            std::cout << ">>> 3. Αυστηρή Άνοδος 10 cm..." << std::endl;
            strict_vertical_move(0.15); 

            std::cout << ">>> 4. Κλείσιμο Gripper..." << std::endl;
            gripper_interface.setNamedTarget("closed"); 
            gripper_interface.move();
            
            std::cout << ">>> Το Drop/Place ολοκληρώθηκε!" << std::endl;
        }
        continue;
    }

    // --- ΕΝΤΟΛΗ: ΑΠΛΗ ΜΕΤΑΒΑΣΗ ΣΕ X, Y ΣΥΝΤΕΤΑΓΜΕΝΕΣ ---
    std::stringstream ss(line); 
    double tx, ty;
    if (ss >> tx >> ty) {
        std::cout << "\n>>> ΕΚΤΕΛΕΣΗ ΜΕΤΑΒΑΣΗΣ: Στόχος X=" << tx << "m, Y=" << ty << "m" << std::endl;
        
        // Περνάμε true για να επιτρέψουμε τη χρήση της γενικής γέφυρας
        if (!try_direct_cartesian(tx, ty, true)) {
            std::cout << "    [-] Αποτυχία εύρεσης διαδρομής για X=" << tx << ", Y=" << ty << " σε όλα τα πιθανά ύψη." << std::endl;
        }
        continue;
    } else {
        std::cout << "[-] Άγνωστη ή λανθασμένη εντολή. Δοκιμάστε ξανά." << std::endl;
    }
  }
  rclcpp::shutdown();
  return 0;
}
