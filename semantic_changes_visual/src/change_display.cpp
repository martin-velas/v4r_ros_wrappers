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

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <cv.h>

#include <tf/transform_listener.h>

#include "rviz/default_plugin/point_cloud_common.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/int_property.h"
#include "rviz/ogre_helpers/movable_text.h"

#include <v4r/changedet/Visualizer3D.h>

#include "change_display.h"

using namespace rviz;
using namespace std;

namespace semantic_changes_visual
{

ChangeDisplay::ChangeDisplay()
  : point_cloud_common_( new PointCloudCommon( this ))
{
  queue_size_property_ = new IntProperty( "Queue Size", 10,
                                          "Advanced: set the size of the incoming PointCloud message queue. "
                                          " Increasing this is useful if your incoming TF data is delayed significantly "
                                          "from your PointCloud data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));

  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.
  update_nh_.setCallbackQueue( point_cloud_common_->getCallbackQueue() );
}

ChangeDisplay::~ChangeDisplay()
{
  delete point_cloud_common_;
}

void ChangeDisplay::onInitialize()
{
  MFDClass::onInitialize();
  point_cloud_common_->initialize( context_, scene_node_ );
}

void ChangeDisplay::updateQueueSize()
{
  tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
}

void ChangeDisplay::processMessage( const ChangedSceneConstPtr& scene )
{
  initMessageProcessing(scene);

  Cloud cloud;
  pcl::fromROSMsg(scene->scene_cloud, cloud);
  addColoredCloud(cloud, -1, -1, -1);

  addSimpleChanges(scene->removed, Change::REMOVE);
  addSimpleChanges(scene->added, Change::ADD);
  addSimpleChanges(scene->preserved, Change::PRESERVE);

  addMoveChanges(scene->moved);

  sensor_msgs::PointCloud2Ptr to_display(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(display_cloud, *to_display);

  to_display->header = scene->header;

  point_cloud_common_->addMessage(to_display);
}

void ChangeDisplay::initMessageProcessing( const ChangedSceneConstPtr& scene ) {
  if( !context_->getFrameManager()->getTransform( scene->header.frame_id,
		  scene->header.stamp,
												  position, orientation ))
  {
	ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
			scene->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
	return;
  }
  bb_lines.clear();
  annotations.clear();
  display_cloud.clear();
  arrows.clear();
}

void ChangeDisplay::addSimpleChanges( const std::vector<SimpleChange>& changes, Change::Type type ) {
    int r, g, b;
    Change::getColors(type, r, g, b);
    for(std::vector<SimpleChange>::const_iterator ch = changes.begin(); ch < changes.end(); ch++) {
        Cloud cloud;
        pcl::fromROSMsg(ch->cloud, cloud);
        int alpha = (type == Change::REMOVE) ? 100 : 255;
        addColoredCloud(cloud, r, g, b);
    	pcl::PointXYZ min, max;
        getMinMax3DInCloud(cloud, min, max);
        addAnnotatedBB(min, max, ch->label, MAX(r,0), MAX(g,0), MAX(b,0));
    }
}

void ChangeDisplay::addMoveChanges( const std::vector<MoveChange>& changes) {
	int r_from, g_from, b_from;
	Change::getColors(Change::MOVE_FROM, r_from, g_from, b_from);
	int r_to, g_to, b_to;
	Change::getColors(Change::MOVE_TO, r_to, g_to, b_to);

	for(std::vector<MoveChange>::const_iterator ch = changes.begin(); ch < changes.end(); ch++) {
		Cloud cloud_from, cloud_to;
		pcl::fromROSMsg(ch->cloud_from, cloud_from);
		pcl::fromROSMsg(ch->cloud_to, cloud_to);
		addColoredCloud(cloud_from, r_from, g_from, b_from, 100);
		addColoredCloud(cloud_to, r_to, g_to, b_to);

		pcl::PointXYZ min_from, max_from, min_to, max_to;
		getMinMax3DInCloud(cloud_from, min_from, max_from);
		addAnnotatedBB(min_from, max_from, ch->label, MAX(r_from,0), MAX(g_from,0), MAX(b_from,0));
		getMinMax3DInCloud(cloud_to, min_to, max_to);
		addAnnotatedBB(min_to, max_to, ch->label, MAX(r_to,0), MAX(g_to,0), MAX(b_to,0));

		Ogre::Vector3 arrow_from, arrow_to;
		getClosestPointsOfBB(min_from, max_from, min_to, max_to, arrow_from, arrow_to);
		addArrow(arrow_from, arrow_to,
				avg(r_from, r_to), avg(g_from, g_to), avg(b_from, b_from));
	}
}

void ChangeDisplay::addColoredCloud( Cloud& cloud, int r, int g, int b, int alpha) {
    for(Cloud::iterator pt = cloud.begin(); pt < cloud.end(); pt++) {
        pt->r = (r > 0) ? r : pt->r;
        pt->g = (g > 0) ? g : pt->g;
        pt->b = (b > 0) ? b : pt->b;
        pt->a = alpha;
    }
    display_cloud += cloud;
}

void ChangeDisplay::addLine(Ogre::Vector3 from, Ogre::Vector3 to, int r, int g, int b) {
	Ogre::SceneNode* frame_node = scene_node_->createChildSceneNode();
	LinePtr line(new Line(context_->getSceneManager(), frame_node));
	line->setPoints(from, to);
	line->setColor(r/255.0, g/255.0, b/255.0, 1.0);
	line->setVisible(true);

	frame_node->setPosition(position);
	frame_node->setOrientation(orientation);

	bb_lines.push_back(line);
}

void ChangeDisplay::addArrow(Ogre::Vector3 from, Ogre::Vector3 to, int r, int g, int b) {
	Ogre::SceneNode* frame_node = scene_node_->createChildSceneNode();
	ArrowPtr arrow(new Arrow(context_->getSceneManager(), frame_node));
	arrow->setColor(255.0, 0, 0, 1.0);
	arrow->setPosition(from);
	Ogre::Vector3 direct = to - from;
	float length = direct.length();
	arrow->setDirection(direct);
	arrow->setScale(Ogre::Vector3(length*0.9, length*2, length*2));

	frame_node->setPosition(position);
	frame_node->setOrientation(orientation);

	arrows.push_back(arrow);
}

void ChangeDisplay::addText(std::string label, Ogre::Vector3 pos, int r, int g, int b) {
	MovableText* text = new MovableText(label, "Arial", TEXT_SIZE);
	MovableTextPtr annot(text);
	annot->setTextAlignment(MovableText::H_CENTER, MovableText::V_ABOVE);
	Ogre::SceneNode* frame_node = scene_node_->createChildSceneNode();
	frame_node->attachObject(text);
	annot->setColor(Ogre::ColourValue(r/255.0, g/255.0, b/255.0, 1.0));
	annot->getParentNode()->setPosition(pos);
	annot->setVisible(true);

	annotations.push_back(annot);
}

void ChangeDisplay::addAnnotatedBB( const pcl::PointXYZ &min, const pcl::PointXYZ &max, std::string label, int r, int g, int b ) {

    float x_coords[] = {min.x, max.x};
    float y_coords[] = {min.y, max.y};
    float z_coords[] = {min.z, max.z};

    for(int ix1 = 0; ix1 < 2; ix1++) {
        for(int ix2 = 0; ix2 < 2; ix2++) {
            for(int iy1 = 0; iy1 < 2; iy1++) {
                for(int iy2 = 0; iy2 < 2; iy2++) {
                    for(int iz1 = 0; iz1 < 2; iz1++) {
                        for(int iz2 = 0; iz2 < 2; iz2++) {
                        	int eq = 0;
                        	eq += (ix1 == ix2);
                        	eq += (iy1 == iy2);
                        	eq += (iz1 == iz2);
                        	if(eq > 0) {
                        		addLine(Ogre::Vector3(x_coords[ix1], y_coords[iy1], z_coords[iz1]),
                        				Ogre::Vector3(x_coords[ix2], y_coords[iy2], z_coords[iz2]),
                        				r, g, b);
                        	}
                        }
                    }
                }
            }
        }
    }

    Ogre::Vector3 text_pos((min.x+max.x)/2, min.y-TEXT_VERT_DIST, (min.z+max.z)/2);
    addText(label, text_pos, r, g, b);
}


void ChangeDisplay::update( float wall_dt, float ros_dt )
{
  point_cloud_common_->update( wall_dt, ros_dt );
}

void ChangeDisplay::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( semantic_changes_visual::ChangeDisplay, rviz::Display )
