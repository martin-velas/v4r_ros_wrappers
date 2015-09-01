/*
 * ChangeDetectionBridge.h
 *
 *  Created on: 27 Aug 2015
 *      Author: martin
 */

#ifndef CHANGEDETECTIONBRIDGE_H_
#define CHANGEDETECTIONBRIDGE_H_

#include <pcl_conversions/pcl_conversions.h>

#include <v4r/changedet/ObjectHistory.h>

#include <semantic_changes_visual/SimpleChange.h>
#include <semantic_changes_visual/MoveChange.h>

class ChangeDetectionBridge {
public:
	template<class PointType>
	static void toROSMsg(const v4r::ObjectChangeForVisual<PointType> &change,
			semantic_changes_visual::SimpleChange &msg) {
		msg.id = change.id;
		msg.label = change.label;
		pcl::toROSMsg(*(change.cloud), msg.cloud);
		ObjectDetectionBridge::transformationToROSMsg(change.pose, msg.pose);
	}

	template<class PointType>
	static void toROSMsg(const v4r::ObjectChangeForVisual<PointType> &change,
			semantic_changes_visual::MoveChange &msg) {
		msg.id = change.id;
		msg.label = change.label;
		pcl::toROSMsg(*(change.cloud), msg.cloud);
		ObjectDetectionBridge::transformationToROSMsg(change.pose, msg.pose_to);
		ObjectDetectionBridge::transformationToROSMsg(change.pose_previous, msg.pose_from);
	}
};

#endif /* CHANGEDETECTIONBRIDGE_H_ */
