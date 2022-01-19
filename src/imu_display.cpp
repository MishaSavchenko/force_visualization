#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include "force_visualization/imu_visual.h"
#include <rviz/frame_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/visualization_manager.h>

#include "force_visualization/imu_display.h"

namespace force_visualization {

struct SIMetric {
  std::string prefix;
  std::string abbriviation;
  double multiplier;
  SIMetric(std::string prefix_, std::string abbriviation_, double multiplier_)
      : prefix(prefix_), abbriviation(abbriviation_), multiplier(multiplier_) {}
};

std::vector<SIMetric> SI_METRICS{SIMetric("", "", 1e0),        //
                                 SIMetric("Giga", "G", 1e9),   //
                                 SIMetric("Mega", "M", 1e6),   //
                                 SIMetric("Kilo", "k", 1e3),   //
                                 SIMetric("Hecto", "h", 1e2),  //
                                 SIMetric("Deca", "da", 1e1),  //
                                 SIMetric("Deci", "d", 1e-1),  //
                                 SIMetric("Centi", "c", 1e-2), //
                                 SIMetric("Milli", "m", 1e-3), //
                                 SIMetric("Micro", "μ", 1e-6), //
                                 SIMetric("Nano", "n", 1e-9),  //
                                 SIMetric("Pico", "p", 1e-12)};

ImuDisplayTest::ImuDisplayTest() {

  axes_colors_ = new rviz::BoolProperty("Axes Colors",                  //
                                        true,                           //
                                        "Colors for the axes of force", //
                                        this);

  x_axis_color_property_ =
      new rviz::ColorProperty("X Axis Color",                      //
                              QColor(255, 0.0, 0.0),               //
                              "Color to draw the X force arrows.", //
                              axes_colors_);
  y_axis_color_property_ =
      new rviz::ColorProperty("Y Axis Color",                      //
                              QColor(0.0, 255, 0.0),               //
                              "Color to draw the Y force arrows.", //
                              axes_colors_);

  z_axis_color_property_ =
      new rviz::ColorProperty("Z Axis Color",                      //
                              QColor(0.0, 0.0, 255),               //
                              "Color to draw the Z force arrows.", //
                              axes_colors_);
  alpha_property_ =
      new rviz::FloatProperty("Alpha",                                        //
                              1.0,                                            //
                              "0 is fully transparent, 1.0 is fully opaque.", //
                              this);

  enum_test_ = new rviz::EnumProperty("Units",       //
                                      "Newton (1)",  //
                                      "DESCRIPTION", //
                                      this);

  force_data_ =
      new rviz::VectorProperty("Force Axes",                                //
                               Ogre::Vector3::ZERO,                         //
                               "Numerical readings from of the force data", //
                               this);

  torque_data_ =
      new rviz::VectorProperty("Torque Axes",                               //
                               Ogre::Vector3::ZERO,                         //
                               "Numerical readings from of the force data", //
                               this);
  // torque_data_ = new rviz::VectorProperty("Force Axes");

  history_length_property_ = new rviz::IntProperty(
      "History Length", 1, "Number of prior measurements to display.", this,
      SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(100000);

  for (int i = 0; i < SI_METRICS.size(); i++) {
    std::stringstream option_ss;
    option_ss << SI_METRICS[i].prefix << "Newton "  //
              << SI_METRICS[i].abbriviation << " (" //
              << SI_METRICS[i].multiplier << ")";
    enum_test_->addOption(QString::fromUtf8(option_ss.str().c_str()), i);
  }
}

void ImuDisplayTest::onInitialize() {
  MFDClass::onInitialize();
  updateHistoryLength();
}

ImuDisplayTest::~ImuDisplayTest() {}

void ImuDisplayTest::reset() {
  MFDClass::reset();
  visuals_.clear();
}

void ImuDisplayTest::updateHistoryLength() {
  visuals_.rset_capacity(history_length_property_->getInt());
}

void ImuDisplayTest::processMessage(
    const geometry_msgs::WrenchStamped::ConstPtr &msg) {

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  boost::shared_ptr<ImuVisual> visual;
  if (visuals_.full()) {
    visual = visuals_.front();
  } else {
    visual.reset(new ImuVisual(context_->getSceneManager(), scene_node_));
  }

  visual->setMessage(msg, //
                     1.0 / SI_METRICS[enum_test_->getOptionInt()].multiplier);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  force_data_->setVector(Ogre::Vector3(msg->wrench.force.x, //
                                       msg->wrench.force.y, //
                                       msg->wrench.force.z));

  torque_data_->setVector(Ogre::Vector3(msg->wrench.torque.x, //
                                        msg->wrench.torque.y, //
                                        msg->wrench.torque.z));

  float alpha = alpha_property_->getFloat();

  std::vector<Ogre::ColourValue> axes_color{
      x_axis_color_property_->getOgreColor(),
      y_axis_color_property_->getOgreColor(),
      z_axis_color_property_->getOgreColor()};

  axes_color[0].a = alpha;
  axes_color[1].a = alpha;
  axes_color[2].a = alpha;

  visual->setAxesColor(axes_color);
  visuals_.push_back(visual);
}

} // end namespace force_visualization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(force_visualization::ImuDisplayTest, rviz::Display)
