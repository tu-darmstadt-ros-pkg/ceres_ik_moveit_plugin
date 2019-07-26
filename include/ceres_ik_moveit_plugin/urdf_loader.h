#ifndef URDF_LOADER_H
#define URDF_LOADER_H

#include <ros/ros.h>

#include <ceres_ik_moveit_plugin/transforms.h>

#include <moveit/robot_model/robot_model.h>

namespace ceres_ik_moveit_plugin {

class UrdfLoader {
public:
  std::vector<Link> buildChain(std::shared_ptr<const urdf::Link> root, const robot_model::JointModelGroup* joint_group);


private:
  bool addToChain(urdf::LinkConstSharedPtr root, std::vector<Link>& chain);
  std::shared_ptr<Joint> toJoint(urdf::JointConstSharedPtr urdf_joint);

  Transform<double> toTransform(urdf::Pose p);
  Eigen::Quaterniond toRotation(urdf::Rotation r);
  Eigen::Vector3d toTranslation(urdf::Vector3 v);
};

}

#endif
