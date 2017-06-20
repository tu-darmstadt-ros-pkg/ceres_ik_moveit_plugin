#include <ceres_ik_moveit_plugin/transforms.h>

namespace ceres_ik_moveit_plugin {

// Transform

template <typename T> void convertTransform(const Transform<double>& t1, Transform<T>& t2) {
  t2.rotation.x() = T(t1.rotation.x());
  t2.rotation.y() = T(t1.rotation.y());
  t2.rotation.z() = T(t1.rotation.z());
  t2.rotation.w() = T(t1.rotation.w());

  for (unsigned int i = 0; i < 3; i++) {
    t2.translation(i) = T(t1.translation(i));
  }
}

template <> void convertTransform(const Transform<double>& t1, Transform<double>& t2) {
  t2.rotation.x() = t1.rotation.x();
  t2.rotation.y() = t1.rotation.y();
  t2.rotation.z() = t1.rotation.z();
  t2.rotation.w() = t1.rotation.w();

  for (unsigned int i = 0; i < 3; i++) {
    t2.translation(i) = t1.translation(i);
  }
}

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

//Transform<double> RevoluteJoint::pose(double q) const {
//  return Transform<double>(Eigen::Quaterniond(Eigen::AngleAxisd(q, axis_)), origin_);
//}

bool RevoluteJoint::isActuated() const {
  return true;
}

// Link

Link::Link(std::string name, const Transform<double>& tip_transform, boost::shared_ptr<Joint> joint)
  : name_(name), tip_transform_(joint->pose<double>(0).inverse() * tip_transform), joint_(joint) {}

//Transform<double> Link::pose(double q) const {
//  return joint_->pose<double>(q) * tip_transform_;
//}



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
