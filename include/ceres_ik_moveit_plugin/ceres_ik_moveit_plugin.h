#ifndef CERES_IK_MOVEIT_PLUGIN_H
#define CERES_IK_MOVEIT_PLUGIN_H

#include <algorithm>

// ROS
#include <ros/ros.h>
#include <random_numbers/random_numbers.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <ceres_ik_moveit_plugin/error_functions/pose_error.h>
#include <ceres_ik_moveit_plugin/error_functions/pose_error_free_angle.h>
#include <ceres_ik_moveit_plugin/error_functions/position_error.h>
#include <ceres_ik_moveit_plugin/error_functions/joint_angle_regularization.h>

#include <ceres_ik_moveit_plugin/transforms.h>
#include <ceres_ik_moveit_plugin/urdf_loader.h>

namespace ceres_ik_moveit_plugin {

template<typename T>
std::string vecToString(const std::vector<T>& vec) {
  std::stringstream ss;
  ss << "[";
  for (unsigned int i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() -1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

template<typename T>
std::string arrayToString(const T* vec) {
  std::stringstream ss;
  ss << "[";
  for (unsigned int i = 0; i < vec.size(); ++i) {
    ss << vec[i];
    if (i != vec.size() -1) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

class CeresIkMoveitPlugin : public kinematics::KinematicsBase {

public:
  CeresIkMoveitPlugin();


  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& base_name, const std::string& tip_name, double search_discretization);


  virtual bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                             std::vector<geometry_msgs::Pose>& poses) const;

  virtual bool
  getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  virtual const std::vector<std::string>& getJointNames() const;

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  virtual const std::vector<std::string>& getLinkNames() const;


protected:
  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param check_consistency Set to true if consistency check needs to be performed
   * @param redundancy The index of the redundant joint
   * @param consistency_limit The returned solutuion will contain a value for the redundant joint in the range
   * [seed_state(redundancy_limit)-consistency_limit,seed_state(redundancy_limit)+consistency_limit]
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                        std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::MoveItErrorCodes& error_code, const std::vector<double>& consistency_limits,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

private:
  // chain model
  std::vector<Link> chain_;
  int num_actuated_joints_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;

  // optimisation params
  bool position_only_ik;
  std::string free_angle_;
  int ik_solver_attempts_;
  int max_iterations_;
  double orientation_weight_;
  bool joint_angle_regularization_;
  std::vector<double> regularization_factors_;
  double goal_tolerance_;


  bool active_;
};

}

#endif
