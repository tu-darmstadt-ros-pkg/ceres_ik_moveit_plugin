#include <ceres_ik_moveit_plugin/ceres_ik_moveit_plugin.h>

#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <class_loader/class_loader.h>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

#include <chrono>

// register KDLKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(ceres_ik_moveit_plugin::CeresIkMoveitPlugin, kinematics::KinematicsBase)

namespace ceres_ik_moveit_plugin {

CeresIkMoveitPlugin::CeresIkMoveitPlugin()
  : active_(false) {

}

bool CeresIkMoveitPlugin::initialize(const std::string& robot_description, const std::string& group_name,
                                     const std::string& base_frame, const std::string& tip_frame,
                                     double search_discretization)
{
  setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);

  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
  const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED("CeresIK", "URDF and SRDF must be loaded for Ceres IK solver to work.");
    return false;
  }

  robot_model::RobotModel robot_model(urdf_model, srdf);
  robot_model::JointModelGroup* joint_model_group = robot_model.getJointModelGroup(group_name);

  // Checks
  if (!joint_model_group) {
    ROS_ERROR_STREAM_NAMED("CeresIK", "Can't find joint model group with name " << group_name << ".");
    return false;
  }

  if (!joint_model_group->isChain())
  {
    ROS_ERROR_NAMED("CeresIK", "Group '%s' is not a chain", group_name.c_str());
    return false;
  }

  if (!joint_model_group->isSingleDOFJoints())
  {
    ROS_ERROR_NAMED("CeresIK", "Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
    return false;
  }

  if (!joint_model_group->hasLinkModel(getTipFrame()))
  {
    ROS_ERROR_STREAM_NAMED("CeresIK", "Could not find tip name '" <<  getTipFrame() << "' in joint group '" << group_name << "'.");
    return false;
  }

  // Load robot transforms into custom templated data structure
  UrdfLoader urdf_loader;
  chain_ = urdf_loader.buildChain(urdf_model->getRoot(), joint_model_group);
  num_actuated_joints_ = 0;
  for (unsigned int i = 0; i < chain_.size(); i++) {
    if (chain_[i].getJoint()->isActuated()) {
      num_actuated_joints_++;
    }
    joint_names_.push_back(chain_[i].getJoint()->getName());
    link_names_.push_back(chain_[i].getName());
  }

  // Load parameters
  ros::NodeHandle pnh("~/" + group_name_);
  ROS_INFO_STREAM("Loading params from " << pnh.getNamespace() << " .");
  pnh.param<std::string>("free_angle", free_angle_, "none");
  std::transform(free_angle_.begin(), free_angle_.end(), free_angle_.begin(), ::tolower);
  pnh.param("position_only_ik", position_only_ik, false);
  pnh.param("ik_solver_attempts", ik_solver_attempts_, 3);
  pnh.param("max_iterations", max_iterations_, -1);
  pnh.param("orientation_weight", orientation_weight_, 0.5);
  pnh.param<std::vector<double>>("regularization_factors", regularization_factors_, std::vector<double>(num_actuated_joints_, 0));
  if (regularization_factors_.size() != num_actuated_joints_) {
    ROS_ERROR_STREAM_NAMED("CeresIK", "Size of regularization factors doesn't match number of actuated joints");
    return false;
  }
  pnh.param("joint_angle_regularization", joint_angle_regularization_, false);
  pnh.param<std::vector<double>>("default_seed_state", default_seed_state_, std::vector<double>(num_actuated_joints_, 0));

  active_ = true;
  return true;
}

bool CeresIkMoveitPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                       const std::vector<double>& joint_angles,
                                       std::vector<geometry_msgs::Pose>& poses) const
{
  if (!active_) {
    ROS_ERROR_NAMED("CeresIK", "FK failed. Plugin not active");
    return false;
  }

  if (joint_angles.size() != num_actuated_joints_) {
    ROS_ERROR_STREAM_NAMED("CeresIK", "Can't compute FK. joint_angles size (" << joint_angles.size()
                           << ") doesn't match number of actuated joints (" << num_actuated_joints_ << ").");
    return false;
  }
  Transform<double> pose;
  poses.resize(link_names.size());

  int current_angle_idx = 0;
  for (unsigned int i = 0; i < chain_.size(); i++) {
    double q;
    if (chain_[i].getJoint()->isActuated()) {
      q = joint_angles[current_angle_idx];
      current_angle_idx++;
    } else {
      q = 0.0;
    }
    pose = pose * chain_[i].pose<double>(q);
    for (unsigned int j = 0; j < link_names.size(); j++) {
      if (chain_[i].getName() == link_names[j]) {
        poses[j] = transformToMsg(pose);
      }
    }
  }

  return true;
}


bool CeresIkMoveitPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const std::vector<double> &consistency_limits,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

  // Validity check
  if (!active_) {
    ROS_ERROR_NAMED("CeresIK", "IK failed. Plugin not active");
    error_code.val = error_code.NO_IK_SOLUTION;
  }

  if (ik_seed_state.size() != num_actuated_joints_) {
    ROS_ERROR_STREAM_NAMED("CeresIK", "Can't compute IK. ik_seed_state size (" << ik_seed_state.size()
                           << ") doesn't match number of actuated joints (" << num_actuated_joints_ << ").");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // copy seed vector to init joint array
  double joint_state[num_actuated_joints_];
  for (unsigned int i = 0; i < num_actuated_joints_; i++) {
    joint_state[i] = ik_seed_state[i];
  }

  // build the problem
  ceres::Problem problem;
  ceres::CostFunction* cost_function;
  if (position_only_ik) {
    // only position ik
    ROS_DEBUG_STREAM("Position Only IK");
    cost_function = PositionError::Create(chain_, msgToTransform(ik_pose), num_actuated_joints_);
  } else if (free_angle_ != "none") {
    // position and orientation ik except one free angle
    ROS_DEBUG_STREAM("Pose IK with free angle: " << free_angle_);
    cost_function = PoseErrorFreeAngle::Create(chain_, msgToTransform(ik_pose), num_actuated_joints_, free_angle_, orientation_weight_);
  } else {
    // full ik: position and orientation
    ROS_DEBUG_STREAM("Full IK");
    cost_function = PoseError::Create(chain_, msgToTransform(ik_pose), num_actuated_joints_, orientation_weight_);
  }
  problem.AddResidualBlock(cost_function, NULL, joint_state);

  if (joint_angle_regularization_) {
    ROS_DEBUG_STREAM("Adding joint angle regularisation");
    ceres::CostFunction* regularization_function = JointAngleRegularization::Create(ik_seed_state, regularization_factors_);
    problem.AddResidualBlock(regularization_function, NULL, joint_state);
  }

  // set joint angle limits
  int joint_state_idx = 0;
  for (unsigned int i = 0; i < chain_.size(); i++) {
    if (chain_[i].getJoint()->isActuated()) {
      problem.SetParameterLowerBound(joint_state, joint_state_idx, chain_[i].getJoint()->getLowerLimit());
      problem.SetParameterUpperBound(joint_state, joint_state_idx, chain_[i].getJoint()->getUpperLimit());
      joint_state_idx++;
    }
  }

  // Solve the NLE
  ceres::Solver::Options ceres_options;
  ceres_options.max_solver_time_in_seconds = timeout;
  ceres_options.linear_solver_type = ceres::DENSE_QR;
//    ceres_options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(ceres_options, &problem, &summary);
  std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
  ROS_INFO_STREAM("IK needed " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " ms.");

  // check if we got a valid solution
  if (summary.termination_type != ceres::TerminationType::CONVERGENCE) {
    ROS_WARN_STREAM("Ceres did not converge. No solution found.");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
  ROS_DEBUG_STREAM(summary.BriefReport());

  // copy solution to vector
  solution.resize(num_actuated_joints_);
  for (unsigned int i = 0; i < num_actuated_joints_; i++) {
    solution[i] = joint_state[i];
  }

  // use callback to check for collisions
  for (unsigned int current_try = 0; current_try < ik_solver_attempts_; current_try++) {
    if (!solution_callback.empty()) {
      solution_callback(ik_pose, solution, error_code);
      if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_INFO_STREAM_NAMED("CeresIK", "Solution passes callback");
        return true;
      } else {
        ROS_WARN_STREAM_NAMED("CeresIK", "Solution Callback: Solution has error code " << error_code.val);
        return false;
      }
    } else {
      // no callback provided
      error_code.val = error_code.SUCCESS;
      return true;
    }
  }

}

const std::vector<std::string>& CeresIkMoveitPlugin::getJointNames() const {
  return joint_names_;
}

const std::vector<std::string>& CeresIkMoveitPlugin::getLinkNames() const {
  return link_names_;
}


bool CeresIkMoveitPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                          const std::vector<double> &ik_seed_state,
                                          std::vector<double> &solution,
                                          moveit_msgs::MoveItErrorCodes &error_code,
                                          const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          default_timeout_,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool CeresIkMoveitPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool CeresIkMoveitPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           const std::vector<double> &consistency_limits,
                                           std::vector<double> &solution,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool CeresIkMoveitPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool CeresIkMoveitPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                           const std::vector<double> &ik_seed_state,
                                           double timeout,
                                           const std::vector<double> &consistency_limits,
                                           std::vector<double> &solution,
                                           const IKCallbackFn &solution_callback,
                                           moveit_msgs::MoveItErrorCodes &error_code,
                                           const kinematics::KinematicsQueryOptions &options) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

}

