#include <ceres_ik_moveit_plugin/ceres_ik_moveit_plugin.h>

#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <class_loader/class_loader.h>

// URDF, SRDF
#include <urdf_model/model.h>
#include <srdfdom/model.h>

#include <moveit/rdf_loader/rdf_loader.h>

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

    ros::NodeHandle private_handle("~");

    rdf_loader::RDFLoader rdf_loader(robot_description_);
    const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
    const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();

    if (!urdf_model || !srdf)
    {
      ROS_ERROR_NAMED("CeresIK", "URDF and SRDF must be loaded for Ceres IK solver to work.");
      return false;
    }

    robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

    robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);

    // Endless checks
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
    }

    for (unsigned int i = 0; i < chain_.size(); i++) {
      joint_names_.push_back(chain_[i].getJoint()->getName());
    }

    for (unsigned int i = 0; i < chain_.size(); i++) {
      link_names_.push_back(chain_[i].getName());
    }

    active_ = true;

//    Transform<double> pose1(Eigen::Quaterniond::Identity(), Eigen::Vector3d(1, 2, 2));
//    Transform<double> pose2(Eigen::Quaterniond::Identity(), Eigen::Vector3d(5, 3, 2));
//    Transform<double> pose3 = pose1 * pose2;
//    Transform<double> pose4;
//    convertTransform<double>(pose3, pose4);

//    ROS_INFO_STREAM("pose3: " << pose3.toString());
//    ROS_INFO_STREAM("pose4: " << pose4.toString());


//    std::vector<geometry_msgs::Pose> pose;
//    std::vector<double> angles = {0.0, 1.55, 2.94, 1.61, 0.0};
//    std::vector<std::string> names = {"arm_link_0", "arm_link_1", "arm_link_2", "arm_link_3", "arm_link_4"};
//    getPositionFK(names, angles, pose);
//    for (unsigned int i = 0; i < pose.size(); i++) {
//      ROS_INFO_STREAM("Translation: " << pose[i].position.x << ", " << pose[i].position.y << ", " << pose[i].position.z);
//      ROS_INFO_STREAM("Rotation: " << pose[i].orientation.w << ", " << pose[i].orientation.x << ", " << pose[i].orientation.y << ", " << pose[i].orientation.z);
//    }

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

    double joint_state[num_actuated_joints_];
    for (unsigned int i = 0; i < num_actuated_joints_; i++) {
      joint_state[i] = ik_seed_state[i];
    }

    ceres::Problem problem;
    ceres::CostFunction* cost_function = CartesianError::Create(chain_, msgToTransform(ik_pose), num_actuated_joints_);
    problem.AddResidualBlock(cost_function, NULL, joint_state);
    int joint_state_idx = 0;
    for (unsigned int i = 0; i < chain_.size(); i++) {
      if (chain_[i].getJoint()->isActuated()) {
        problem.SetParameterLowerBound(joint_state, joint_state_idx, chain_[i].getJoint()->getLowerLimit());
        problem.SetParameterLowerBound(joint_state, joint_state_idx, chain_[i].getJoint()->getUpperLimit());
        joint_state_idx++;
      }
    }

    ceres::Solver::Options ceres_options;
    ceres_options.max_solver_time_in_seconds = timeout;
    ceres_options.linear_solver_type = ceres::DENSE_QR;
    ceres_options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(ceres_options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    solution.resize(num_actuated_joints_);
    std::stringstream ss;
    ss << "[";
    for (unsigned int i = 0; i < num_actuated_joints_; i++) {
      solution[i] = joint_state[i];
      ss << solution[i] << ", ";
    }
    ss << "]";
    ROS_INFO_STREAM("Translation: " << ik_pose.position.x << ", " << ik_pose.position.y << ", " << ik_pose.position.z);
    ROS_INFO_STREAM("Rotation: " << ik_pose.orientation.w << ", " << ik_pose.orientation.x << ", " << ik_pose.orientation.y << ", " << ik_pose.orientation.z);
    ROS_INFO_STREAM("Solution: " << ss.str());

    if (!solution_callback.empty()) {
      solution_callback(ik_pose, solution, error_code);
      if(error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        ROS_DEBUG_STREAM_NAMED("CeresIK","Solution passes callback");
        return true;
      }
      else
      {
        ROS_WARN_STREAM_NAMED("CeresIK","Solution Callback: Solution has error code " << error_code);
        return false;
      }
    } else {
      // no callback provided
      return true;
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

