/*
 * bbdisplay.cpp
 *
 *  Created on: 1 Sep 2015
 *      Author: martin
 */

#include "semantic_changes_visual/bb_display.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

using namespace std;
using namespace pcl;

BBDisplay::BBDisplay(const pcl::PointCloud<PointT>::Ptr cloud_orig, const Eigen::Affine3f &pose) :
	cloud(new pcl::PointCloud<PointT>()) {

	transformPointCloud(*cloud_orig, *cloud, pose);

	pcl::PointXYZ min, max;
	getMinMax3DInCloud(*cloud_orig, min, max);

	corners.push_back(Ogre::Vector3(min.x, min.y, min.z));
	corners.push_back(Ogre::Vector3(max.x, min.y, min.z));
	corners.push_back(Ogre::Vector3(max.x, max.y, min.z));
	corners.push_back(Ogre::Vector3(min.x, max.y, min.z));

	corners.push_back(Ogre::Vector3(min.x, min.y, max.z));
	corners.push_back(Ogre::Vector3(max.x, min.y, max.z));
	corners.push_back(Ogre::Vector3(max.x, max.y, max.z));
	corners.push_back(Ogre::Vector3(min.x, max.y, max.z));

	for(vector<Ogre::Vector3>::iterator c = corners.begin(); c < corners.end(); c++) {
		pcl::PointXYZ pt(c->x, c->y, c->z);
		pt = transformPoint(pt, pose);
		c->x = pt.x;
		c->y = pt.y;
		c->z = pt.z;
	}
}

// above the middle of top face
Ogre::Vector3 BBDisplay::getAnnotationPos(float above_dist) const {
	Ogre::Vector3 pos = avg(corners[0], corners[6]);
	pos.y = INFINITY;

	for(vector<Ogre::Vector3>::const_iterator c = corners.begin(); c < corners.end(); c++) {
		pos.y = MIN(pos.y, c->y);
	}
	pos.y -= above_dist;

	return pos;
}

vector<LineSegment> BBDisplay::getLines() const {
	vector<LineSegment> lines;
	for(int i = 0; i < 4; i++) {
		lines.push_back(LineSegment(corners[i], corners[(i+1)%4]));	// front face edge
		lines.push_back(LineSegment(corners[i], corners[(i+2)%4]));	// front face diagonal

		lines.push_back(LineSegment(corners[i+4], corners[(i+1)%4+4])); // back face edge
		lines.push_back(LineSegment(corners[i+4], corners[(i+2)%4+4])); // back face diagonal

		lines.push_back(LineSegment(corners[i], corners[i+4]));
		lines.push_back(LineSegment(corners[i], corners[(i+1)%4+4]));
		lines.push_back(LineSegment(corners[i], corners[(i+3)%4+4]));
	}
	return lines;
}

LineSegment BBDisplay::getArrrowTo(const BBDisplay &other) const {
	LineSegment shortest;
	float min_len = INFINITY;
	for(vector<Ogre::Vector3>::const_iterator tc = corners.begin(); tc < corners.end(); tc++) {
		for(vector<Ogre::Vector3>::const_iterator oc = other.corners.begin(); oc < other.corners.end(); oc++) {
			float len = (*tc - *oc).length();
			if(len < min_len) {
				shortest.from = *tc;
				shortest.to = *oc;
			}
		}
	}
	return shortest;
}
