#include <cstdlib>
#include <iostream>

#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>

#include <v4r/changedet/Visualizer3D.h>
#include <v4r/changedet/change_detection.h>
#include <v4r/changedet/ObjectDetection.h>
#include <v4r/changedet/ObjectHistory.h>
#include <v4r/changedet/miscellaneous.h>

#include <ObjectDetectionBridge.h>
#include <semantic_changes_visual/get_removed_objects.h>

using namespace std;
using namespace pcl;

v4r::ObjectsHistory<PointXYZRGB> history;
pcl::PointCloud<PointXYZRGB>::Ptr scene(new pcl::PointCloud<PointXYZRGB>());

bool detect_removed_objects(semantic_changes_visual::get_removed_objects::Request & req,
		semantic_changes_visual::get_removed_objects::Response & response) {

	PointCloud<PointXYZRGB>::Ptr observation(new PointCloud<PointXYZRGB>());
	pcl::fromROSMsg(req.observation, *observation);
	observation = v4r::downsampleCloud<PointXYZRGB>(observation);
	vector< v4r::ObjectDetection<PointXYZRGB> > detected_objects;
	for(int i = 0; i < req.objects.size(); i++) {
		detected_objects.push_back(
				ObjectDetectionBridge::fromROSMsg<PointXYZRGB>(req.objects[i]));
	}
	Eigen::Affine3f camera_pose;
	ObjectDetectionBridge::transformationFromROSMsg(req.camera_pose, camera_pose);

	history.add(detected_objects);

	vector<v4r::ObjectLabel> removed_labels;
	if(scene->empty()) {
		v4r::ChangeDetector<PointXYZRGB> detector;
		detector.detect(scene, observation, camera_pose);

		v4r::ChangeDetector<PointXYZRGB>::removePointsFrom(scene, detector.getRemoved());

		PCL_INFO("======================== CHANGES: ==========================\n");
		vector<v4r::ObjectLabel> changed = history.getRemovedObjects(detector);
		for(unsigned i = 0; i < changed.size(); i++) {
			v4r::ObjectLabel label = changed[i];
			PCL_INFO("Object event: %s has been [REMOVED]\n", label.c_str());
			v4r::ChangeDetector<PointXYZRGB>::removePointsFrom(scene, history.getLastCloudOf(label));
		}
		PCL_INFO("============================================================\n");

		removed_labels = history.getRemovedObjects(detector);
		*scene += *(detector.getAdded());
	} else {
		*scene += *observation;
	}

	for(vector<v4r::ObjectLabel>::iterator rl = removed_labels.begin();
			rl < removed_labels.end(); rl++) {
		semantic_changes_visual::ObjectLabel rl_msg;
		rl_msg.id = -1;
		rl_msg.label = *rl;
		response.removed_labels.push_back(rl_msg);
	}

	// TODO build the ChangedScene msg to publish

	return true;
}


int main(int argc, char *argv[]) {

	string node_name = "semantic_changes_visual";
	string service_name = "removal_detection_service";

	ros::init (argc, argv, node_name);
	ros::NodeHandle nh("~");
	ros::ServiceServer removal_detection_service =
			nh.advertiseService(node_name + "/" + service_name, detect_removed_objects);;

	ros::spin();

	return EXIT_SUCCESS;
}
