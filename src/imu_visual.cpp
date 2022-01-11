#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>

#include <rviz/ogre_helpers/arrow.h>

#include "force_visualization/imu_visual.h"

namespace force_visualization {

ImuVisual::ImuVisual(Ogre::SceneManager *scene_manager,
                     Ogre::SceneNode *parent_node) {
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Imu's header frame
  // relative to the RViz fixed frame.
  frame_node_ = parent_node->createChildSceneNode();

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.
  x_axis_force_.reset(new rviz::Arrow(scene_manager_, frame_node_));
  y_axis_force_.reset(new rviz::Arrow(scene_manager_, frame_node_));
  z_axis_force_.reset(new rviz::Arrow(scene_manager_, frame_node_));
}

ImuVisual::~ImuVisual() {
  // Destroy the frame node since we don't need it anymore.
  scene_manager_->destroySceneNode(frame_node_);
}

void ImuVisual::setMessage(const geometry_msgs::WrenchStamped::ConstPtr &msg, //
                           const double &multiplier)                          //
{
  Ogre::Vector3 multiplier_scale(multiplier, multiplier, multiplier);

  Ogre::Vector3 x_axis(msg->wrench.force.x, 0.0, 0.0);
  Ogre::Vector3 x_scale(abs(msg->wrench.force.x), 1.0, 1.0);
  x_axis_force_->setScale(x_scale * multiplier_scale);
  x_axis_force_->setDirection(x_axis);

  Ogre::Vector3 y_axis(0.0, msg->wrench.force.y, 0.0);
  Ogre::Vector3 y_scale(abs(msg->wrench.force.y), 1.0, 1.0);
  y_axis_force_->setScale(y_scale * multiplier_scale);
  y_axis_force_->setDirection(y_axis);

  Ogre::Vector3 z_axis(0.0, 0.0, msg->wrench.force.z);
  Ogre::Vector3 z_scale(abs(msg->wrench.force.z), 1.0, 1.0);
  z_axis_force_->setScale(z_scale * multiplier_scale);
  z_axis_force_->setDirection(z_axis);
}

// Position and orientation are passed through to the SceneNode.
void ImuVisual::setFramePosition(const Ogre::Vector3 &position) {
  frame_node_->setPosition(position);
}

void ImuVisual::setFrameOrientation(const Ogre::Quaternion &orientation) {
  frame_node_->setOrientation(orientation);
}

void ImuVisual::setAxesColor(const std::vector<Ogre::ColourValue> &axes_color) {
  x_axis_force_->setColor(axes_color[0]);
  y_axis_force_->setColor(axes_color[1]);
  z_axis_force_->setColor(axes_color[2]);
}

} // end namespace force_visualization
