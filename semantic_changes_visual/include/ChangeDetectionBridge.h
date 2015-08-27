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
	}

	template<class PointType>
	static void fromROSMsg(const semantic_changes_visual::SimpleChange &msg,
			v4r::ObjectChangeForVisual<PointType> &change) {
		change.id = msg.id;
		change.label = msg.label;
		pcl::fromROSMsg(msg.cloud, *(change.cloud));
	}

	template<class PointType>
	static void toROSMsg(const v4r::ObjectChangeForVisual<PointType> &change,
			semantic_changes_visual::MoveChange &msg) {
		msg.id = change.id;
		msg.label = change.label;
		pcl::toROSMsg(*(change.cloud), msg.cloud_to);
		pcl::toROSMsg(*(change.cloud_previous), msg.cloud_from);
	}

	template<class PointType>
	static void fromROSMsg(const semantic_changes_visual::MoveChange &msg,
			v4r::ObjectChangeForVisual<PointType> &change) {
		change.id = msg.id;
		change.label = msg.label;
		pcl::fromROSMsg(msg.cloud_to, *(change.cloud));
		pcl::fromROSMsg(msg.cloud_from, *(change.cloud_previous));
	}
};

#endif /* CHANGEDETECTIONBRIDGE_H_ */
