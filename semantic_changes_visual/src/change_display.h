/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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

namespace semantic_changes_visual
{

/**
 * \class PointCloudDisplay
 * \brief Displays a point cloud of type sensor_msgs::PointCloud
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */
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

  void addColoredCloud( Cloud& cloud, int r, int g, int b );

  void addAnnotatedBB( const Cloud& cloud, std::string label, int r, int g, int b );

  void addLine(Ogre::Vector3 from, Ogre::Vector3 to, int r, int g, int b);

  void addArrow(Ogre::Vector3 from, Ogre::Vector3 to, int r, int g, int b);

  void addText(std::string label, Ogre::Vector3 pos, int r, int g, int b);

  static void getMinMax3DInCloud(const Cloud &cloud, pcl::PointXYZ &min, pcl::PointXYZ &max) {
  	  min.x = min.y = min.z = INFINITY;
  	  max.x = max.y = max.z = -INFINITY;
  	  for(Cloud::const_iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
  		  min.x = MIN(min.x, pt->x);
  		  min.y = MIN(min.y, pt->y);
  		  min.z = MIN(min.z, pt->z);
  		  max.x = MAX(max.x, pt->x);
  		  max.y = MAX(max.y, pt->y);
  		  max.z = MAX(max.z, pt->z);
  	  }
    }

  template<class Num>
  static Num avg(Num n1, Num n2) {
	  return (n1 + n2) / 2.0;
  }

  static void getMiddle3DInCloud(const Cloud &cloud, pcl::PointXYZ &middle) {
	  pcl::PointXYZ min, max;
	  getMinMax3DInCloud(cloud, min, max);
	  middle.x = avg(min.x, max.x);
	  middle.y = avg(min.y, max.y);
	  middle.z = avg(min.z, max.z);
	}

  rviz::IntProperty* queue_size_property_;

  rviz::PointCloudCommon* point_cloud_common_;

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  // DISPLAY ELEMENTS:

  Cloud display_cloud;

  std::vector<LinePtr> bb_lines;

  std::vector<MovableTextPtr> annotations;

  std::vector<ArrowPtr> arrows;
};

} // namespace rviz

#endif
