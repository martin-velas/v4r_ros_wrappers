/*
 * bbdisplay.h
 *
 *  Created on: 1 Sep 2015
 *      Author: martin
 */

#ifndef BBDISPLAY_H_
#define BBDISPLAY_H_

#include <cv.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <OgreVector3.h>

class LineSegment {
public:

	LineSegment() {
	}

	LineSegment(const Ogre::Vector3 &from, const Ogre::Vector3 &to) :
		from(from), to(to) {
	}

	Ogre::Vector3 from, to;
};

class BBDisplay {
public:

	typedef pcl::PointXYZRGB PointT;

	BBDisplay(const pcl::PointCloud<PointT>::Ptr cloud_orig, const Eigen::Affine3f &pose);

	Ogre::Vector3 getAnnotationPos(float above_dist = 0.01) const;

	std::vector<LineSegment> getLines() const;

	LineSegment getArrrowTo(const BBDisplay &other) const;

	const pcl::PointCloud<PointT>::Ptr cloud;

	static void getMinMax3DInCloud(const pcl::PointCloud<PointT> &cloud,
			pcl::PointXYZ &min, pcl::PointXYZ &max) {
		min.x = min.y = min.z = INFINITY;
		max.x = max.y = max.z = -INFINITY;
		for (pcl::PointCloud<PointT>::const_iterator pt = cloud.begin();
				pt < cloud.end(); pt++) {
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

	static void getMiddle3DInCloud(const pcl::PointCloud<PointT> &cloud,
			pcl::PointXYZ &middle) {
		pcl::PointXYZ min, max;
		getMinMax3DInCloud(cloud, min, max);
		middle.x = avg(min.x, max.x);
		middle.y = avg(min.y, max.y);
		middle.z = avg(min.z, max.z);
	}

private:

	// clockwise of two opposite faces
	std::vector<Ogre::Vector3> corners;
};

#endif /* BBDISPLAY_H_ */
