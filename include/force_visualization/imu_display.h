#ifndef IMU_DISPLAY_H
#define IMU_DISPLAY_H

#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>

#include <geometry_msgs/WrenchStamped.h>
#include <rviz/message_filter_display.h>
#endif

namespace Ogre {
class SceneNode;
}

namespace rviz {
class ColorProperty;
class FloatProperty;
class IntProperty;
class StringProperty;
class EnumProperty;
class VectorProperty;
} // namespace rviz

namespace force_visualization {

class ImuVisual;

class ImuDisplayTest
    : public rviz::MessageFilterDisplay<geometry_msgs::WrenchStamped> {
  Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  ImuDisplayTest();
  virtual ~ImuDisplayTest();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();

  // These Qt slots get connected to signals indicating changes in the
  // user-editable properties.
private Q_SLOTS:
  // void updateColorAndAlpha();
  void updateHistoryLength();

  // Function to handle an incoming ROS message.
private:
  void processMessage(const geometry_msgs::WrenchStamped::ConstPtr &msg);

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  boost::circular_buffer<boost::shared_ptr<ImuVisual>> visuals_;

  rviz::EnumProperty *enum_test_;
  rviz::StringProperty *string_test_;
  rviz::BoolProperty *axes_colors_;

  rviz::VectorProperty *force_data_;
  rviz::VectorProperty *torque_data_;

  // User-editable property variables.
  rviz::ColorProperty *x_axis_color_property_;
  rviz::ColorProperty *y_axis_color_property_;
  rviz::ColorProperty *z_axis_color_property_;
  rviz::FloatProperty *alpha_property_;
  rviz::IntProperty *history_length_property_;
};

} // end namespace force_visualization

#endif // IMU_DISPLAY_H
