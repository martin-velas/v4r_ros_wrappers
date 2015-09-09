/*
 * ObjectDetectionBridge.h
 *
 *  Created on: 26 Aug 2015
 *      Author: martin
 */

#ifndef OBJECTDETECTIONBRIDGE_H_
#define OBJECTDETECTIONBRIDGE_H_

#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

#include <v4r/changedet/ObjectDetection.h>
#include <semantic_changes_visual/ObjectDetection.h>

class ObjectDetectionBridge {
public:
	template<class PointType>
	static v4r::ObjectDetection<PointType>
	fromROSMsg(const semantic_changes_visual::ObjectDetection &msg) {
		int id = msg.id;
		std::string label = msg.label;
		typename pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
		pcl::fromROSMsg(msg.cloud, *cloud);
		Eigen::Affine3f t;
		transformationFromROSMsg(msg.pose, t);
		return v4r::ObjectDetection<PointType>(label, id, cloud, t);
	}

	template<class PoinType>
	static void toROSMsg(const v4r::ObjectDetection<PoinType> obj,
			semantic_changes_visual::ObjectDetection &msg) {
		msg.id = obj.getId();
		msg.label = obj.getClass();
		pcl::toROSMsg(*(obj.getCloud(false)), msg.cloud);
		transformationToROSMsg(obj.getPose(), msg.pose);
	}

	static void transformationFromROSMsg(const geometry_msgs::Transform &msg,
			Eigen::Affine3f &t) {
		Eigen::Translation3f translation(Eigen::Vector3f(
				msg.translation.x,
				msg.translation.y,
				msg.translation.z));
		Eigen::Quaternionf rotation(msg.rotation.w,
				msg.rotation.x,
				msg.rotation.y,
				msg.rotation.z);
		t = Eigen::Affine3f(translation * rotation);
	}

	static void transformationToROSMsg(const Eigen::Affine3f &t, geometry_msgs::Transform &msg) {
		Eigen::Matrix4f matrix_t = t.matrix();
		Eigen::Affine3d double_t(matrix_t.cast<double>());
		tf::transformEigenToMsg(double_t, msg);
	}
};


#endif /* OBJECTDETECTIONBRIDGE_H_ */
