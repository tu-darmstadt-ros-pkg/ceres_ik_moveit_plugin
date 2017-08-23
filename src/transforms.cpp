#include <ceres_ik_moveit_plugin/transforms.h>

namespace ceres_ik_moveit_plugin {

// Transform
geometry_msgs::Pose transformToMsg(const Transform<double>& transform) {
  geometry_msgs::Pose msg;
  msg.position.x = transform.translation(0);
  msg.position.y = transform.translation(1);
  msg.position.z = transform.translation(2);

  msg.orientation.x = transform.rotation.x();
  msg.orientation.y = transform.rotation.y();
  msg.orientation.z = transform.rotation.z();
  msg.orientation.w = transform.rotation.w();

  return msg;
}

Transform<double> msgToTransform(const geometry_msgs::Pose& msg) {
  return Transform<double>(Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z),
                   Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z));
}

// FixedJoint
FixedJoint::FixedJoint(std::string name, Eigen::Vector3d origin)
  : Joint(name, origin) {}

bool FixedJoint::isActuated() const {
  return false;
}

// RevoluteJoint
RevoluteJoint::RevoluteJoint(std::string name, Eigen::Vector3d origin, Eigen::Vector3d axis, double upper_limit, double lower_limit)
  : Joint(name, origin, axis, upper_limit, lower_limit) {}

bool RevoluteJoint::isActuated() const {
  return true;
}

// Link
Link::Link(std::string name, const Transform<double>& tip_transform, boost::shared_ptr<Joint> joint)
  : name_(name), tip_transform_(joint->pose<double>(0).inverse() * tip_transform), joint_(joint) {}

boost::shared_ptr<Joint> Link::getJoint() const {
  return joint_;
}

std::string Link::getName() const {
  return name_;
}

Transform<double> Link::getTipTransform() const {
  return tip_transform_;
}

}
