#ifndef RVIZ_POINT_CLOUD_DISPLAY_H
#define RVIZ_POINT_CLOUD_DISPLAY_H

#include <deque>
#include <queue>
#include <vector>

#include <cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <semantic_changes_visual/ChangedScene.h>

#include <rviz/message_filter_display.h>
#include <rviz/default_plugin/point_cloud_common.h>
#include <rviz/properties/int_property.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <ObjectDetectionBridge.h>
#include <bb_display.h>

namespace semantic_changes_visual
{

class ChangeDisplay: public rviz::MessageFilterDisplay<ChangedScene>
{
Q_OBJECT
public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> Cloud;
    typedef sensor_msgs::PointCloud2 CloudMsg;
    typedef rviz::Line Line;
    typedef boost::shared_ptr<Line> LinePtr;
    typedef boost::shared_ptr<rviz::MovableText> MovableTextPtr;
    typedef rviz::Arrow Arrow;
    typedef boost::shared_ptr<Arrow> ArrowPtr;

    class Change {
    public:
        typedef enum {
            ADD, REMOVE, MOVE_FROM, MOVE_TO, PRESERVE
        } Type;
        static void getColors(Type type, int &r, int &g, int &b) {
            switch(type) {
            case ADD:
                g = 255; r = b = -1; break;
            case REMOVE:
                r = 255; g = b = -1; break;
            case MOVE_FROM:
                r = 255; g = 100; b = -1; break;
            case MOVE_TO:
                r = 100; g = 255; b = -1; break;
            default:
                b = 255; r = g = -1; break;
            }
        }
    };

  ChangeDisplay();
  ~ChangeDisplay();

  virtual void reset();

  virtual void update( float wall_dt, float ros_dt );

private Q_SLOTS:
  void updateQueueSize();

protected:
  /** @brief Do initialization. Overridden from MessageFilterDisplay. */
  virtual void onInitialize();

  /** @brief Process a single message.  Overridden from MessageFilterDisplay. */
  virtual void processMessage( const ChangedSceneConstPtr& cloud );

  void initMessageProcessing( const ChangedSceneConstPtr& scene );

  void addSimpleChanges( const std::vector<SimpleChange>& changes, Change::Type type );

  void addMoveChanges( const std::vector<MoveChange>& changes);

  void addColoredCloud( Cloud& cloud, int r, int g, int b, int alpha = 255 );

  void addAnnotatedBB( const BBDisplay &bb, std::string label, int r, int g, int b );

  void addLine(Ogre::Vector3 from, Ogre::Vector3 to, int r, int g, int b);

  void addArrow(Ogre::Vector3 from, Ogre::Vector3 to, int r, int g, int b);

  void addText(std::string label, Ogre::Vector3 pos, int r, int g, int b);

  rviz::IntProperty* queue_size_property_;

  rviz::PointCloudCommon* point_cloud_common_;

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  // DISPLAY ELEMENTS:

  Cloud display_cloud;

  std::vector<LinePtr> bb_lines;

  std::vector<MovableTextPtr> annotations;

  std::vector<ArrowPtr> arrows;

  static const float TEXT_SIZE = 0.05;
  static const float TEXT_VERT_DIST = 0.01;
};

} // namespace rviz

#endif
