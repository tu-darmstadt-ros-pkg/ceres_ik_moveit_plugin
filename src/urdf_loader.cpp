#include <ceres_ik_moveit_plugin/urdf_loader.h>

namespace ceres_ik_moveit_plugin {

std::vector<Link> UrdfLoader::buildChain(boost::shared_ptr<const urdf::Link> root, const robot_model::JointModelGroup* joint_group) {
  std::vector<Link> chain;
  std::vector<const robot_model::LinkModel*> link_models = joint_group->getLinkModels();

  boost::shared_ptr<const urdf::Link> current_link = root;
  for (std::vector<const robot_model::LinkModel*>::iterator it = link_models.begin(); it != link_models.end(); ++it) {
    std::string link_name = (*it)->getName();

    for (std::vector<boost::shared_ptr<urdf::Link>>::const_iterator it_childs = current_link->child_links.begin();
         it_childs != current_link->child_links.end();
         ++it_childs) {
      if ((*it_childs)->name == link_name) {
        addToChain(*it_childs, chain);
        current_link = *it_childs;
        break;
      }
    }

  }

  return chain;
}

bool UrdfLoader::addToChain(boost::shared_ptr<const urdf::Link> root, std::vector<Link>& chain) {
  boost::shared_ptr<Joint> joint = toJoint(root->parent_joint);
  ROS_INFO_STREAM("Origin: " << joint->getOrigin() << ", Axis: " << joint->getAxis() << ", Pose(0): " << joint->pose(0.0).toString());
  Link link(root->name, toTransform(root->parent_joint->parent_to_joint_origin_transform), joint);
  ROS_INFO_STREAM("Adding link " << root->name);
  ROS_INFO_STREAM("Tip transform: " << link.getTipTransform().toString());
  chain.push_back(link);
  return true;
}

boost::shared_ptr<Joint> UrdfLoader::toJoint(boost::shared_ptr<const urdf::Joint> urdf_joint) {
  boost::shared_ptr<Joint> joint;
  Transform<double> parent_transform = toTransform(urdf_joint->parent_to_joint_origin_transform);
  switch (urdf_joint->type) {
    case urdf::Joint::FIXED:
      joint.reset(new FixedJoint(urdf_joint->name, parent_transform.translation));
      ROS_INFO_STREAM("Adding fixed joint " << joint->getName());
      break;
    case urdf::Joint::REVOLUTE: {
      Eigen::Vector3d axis = toTranslation(urdf_joint->axis);
      joint.reset(new RevoluteJoint(urdf_joint->name, parent_transform.translation, parent_transform.rotation * axis,
                                    urdf_joint->limits->upper, urdf_joint->limits->lower));
      ROS_INFO_STREAM("Adding revolute joint " << joint->getName() << " with limits " << joint->getLowerLimit() << " to " << joint->getUpperLimit());
      break;
    }

      break;
    default:
      ROS_ERROR_STREAM_NAMED("CeresIK", "Unknown joint type in urdf.");
  }
  return joint;
}

Transform<double> UrdfLoader::toTransform(urdf::Pose p) {
  return Transform<double>(toRotation(p.rotation), toTranslation(p.position));
}

Eigen::Quaterniond UrdfLoader::toRotation(urdf::Rotation r) {
  return Eigen::Quaterniond(r.w, r.x, r.y, r.z);
}

Eigen::Vector3d UrdfLoader::toTranslation(urdf::Vector3 v) {
  return Eigen::Vector3d(v.x, v.y, v.z);
}

}
