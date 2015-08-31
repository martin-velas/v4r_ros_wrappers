#include <cstdlib>
#include <iostream>

#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <v4r/changedet/Visualizer3D.h>
#include <v4r/changedet/change_detection.h>
#include <v4r/changedet/ObjectDetection.h>
#include <v4r/changedet/ObjectHistory.h>
#include <v4r/changedet/miscellaneous.h>

#include <ObjectDetectionBridge.h>
#include <ChangeDetectionBridge.h>
#include <semantic_changes_visual/get_removed_objects.h>
#include <semantic_changes_visual/ChangedScene.h>

using namespace std;
using namespace pcl;

const string NODE_NAME = "semantic_changes_visual";
const string SERVICE_NAME = "removal_detection_service";
const string VIS_TOPIC_NAME = "semantic_changes";
const string VIS_FRAME_ID = "sem_changes";

class ChangeDetectionROS {

private:
	v4r::ObjectsHistory<PointXYZRGB> history;
	pcl::PointCloud<PointXYZRGB>::Ptr scene;
	ros::NodeHandle nodeHandler;
	ros::ServiceServer removal_detection_service;
	ros::Publisher vis_changes_publisher;
	tf::TransformBroadcaster t_broadcaster;

public:
	ChangeDetectionROS() :
		scene(new pcl::PointCloud<PointXYZRGB>()),
		nodeHandler("~") {
		removal_detection_service = nodeHandler.advertiseService(
				SERVICE_NAME, &ChangeDetectionROS::detect_removed_objects, this);
		vis_changes_publisher =
				nodeHandler.advertise<semantic_changes_visual::ChangedScene>(VIS_TOPIC_NAME, 10);
	}

	bool detect_removed_objects(semantic_changes_visual::get_removed_objects::Request & req,
			semantic_changes_visual::get_removed_objects::Response & response) {

		PointCloud<PointXYZRGB>::Ptr observation(new PointCloud<PointXYZRGB>());
		pcl::fromROSMsg(req.observation, *observation);
		observation = v4r::downsampleCloud<PointXYZRGB>(observation);
		Eigen::Affine3f camera_pose;
		ObjectDetectionBridge::transformationFromROSMsg(req.camera_pose, camera_pose);

		vector< v4r::ObjectDetection<PointXYZRGB> > detected_objects;
		for(int i = 0; i < req.objects.size(); i++) {
			detected_objects.push_back(
					ObjectDetectionBridge::fromROSMsg<PointXYZRGB>(req.objects[i]));
		}

		if(!scene->empty()) {
			v4r::ChangeDetector<PointXYZRGB> detector;
			detector.detect(scene, observation, camera_pose);

			v4r::ChangeDetector<PointXYZRGB>::removePointsFrom(scene, detector.getRemoved());

			PCL_INFO("======================== CHANGES: ==========================\n");
			vector<v4r::ObjectIdLabeled> removed = history.markRemovedObjects(detector);
			for(unsigned i = 0; i < removed.size(); i++) {
				v4r::ObjectLabel label = removed[i].label;
				int id = removed[i].id;
				PCL_INFO("Object: %s(%d) has been [REMOVED]\n", label.c_str(), id);
				v4r::ChangeDetector<PointXYZRGB>::removePointsFrom(scene,
						history.getLastCloudOf(label));
				semantic_changes_visual::ObjectLabel label_msg;
				label_msg.id = id;
				label_msg.label = label;
				response.removed_labels.push_back(label_msg);
			}
			PCL_INFO("============================================================\n");

			*scene += *(detector.getAdded());

			/*v4r::Visualizer3D vis;
			vis.addColorPointCloud(scene);
			vis.setColor(0, 255, 0).addPointCloud(*detector.getAdded());
			vis.setColor(255, 0, 0).addPointCloud(*detector.getRemoved());
			Eigen::Vector3f viewport = (camera_pose).translation();
			vis.getViewer()->addSphere(PointXYZ(viewport(0), viewport(1), viewport(2)),
								0.1, 1.0, 0, 0, "cam-pos");
			for(int i = 0; i < detected_objects.size(); i++) {
				vis.addPointCloud(*(history.getLastCloudOf(detected_objects[i].getClass())));
			}
			vis.show();*/
		} else {
			ROS_INFO_STREAM("First observation");
			*scene += *observation;
		}

		history.add(detected_objects);

		publishChangesVisual(camera_pose);

		return true;
	}

	void publishChangesVisual(const Eigen::Affine3f &camera_pose) {
		ros::Time now = ros::Time::now();

		tf::Transform transform;
		Eigen::Matrix4f camera_pose_matrix = camera_pose.matrix();
		Eigen::Affine3d camera_pose_double(camera_pose_matrix.cast<double>());
		tf::transformEigenToTF(camera_pose_double.inverse(), transform);
		t_broadcaster.sendTransform(tf::StampedTransform(transform, now, VIS_FRAME_ID, "map"));

		semantic_changes_visual::ChangedScene sceneMsg;
		sceneMsg.header.frame_id = VIS_FRAME_ID;
		sceneMsg.header.stamp = now;
		toROSMsg(*scene, sceneMsg.scene_cloud);

		fillChanges(v4r::ObjectState::ADDED, sceneMsg.added);
		fillChanges(v4r::ObjectState::REMOVED, sceneMsg.removed);
		fillChanges(v4r::ObjectState::PRESERVED, sceneMsg.preserved);
		fillChanges(sceneMsg.moved);

		vis_changes_publisher.publish(sceneMsg);
	}

	void fillChanges(const v4r::ObjectState::EventT event,
			vector<semantic_changes_visual::SimpleChange> &changes_msg) {
		vector<v4r::ObjectChangeForVisual<pcl::PointXYZRGB> > changes = history.getChanges(event);
		changes_msg.resize(changes.size());
		for(int i = 0; i < changes.size(); i++) {
			ROS_INFO_STREAM("Object " << changes[i].label << " has " << event << endl);
			ChangeDetectionBridge::toROSMsg(changes[i], changes_msg[i]);
		}
	}

	void fillChanges(vector<semantic_changes_visual::MoveChange> &changes_msg) {
		vector<v4r::ObjectChangeForVisual<pcl::PointXYZRGB> > changes = history.getChanges(
			v4r::ObjectState::MOVED);
		changes_msg.resize(changes.size());
		for(int i = 0; i < changes.size(); i++) {
			ROS_INFO_STREAM("Object " << changes[i].label << " has MOVED" << endl);
			ChangeDetectionBridge::toROSMsg(changes[i], changes_msg[i]);
		}
	}
};



int main(int argc, char *argv[]) {

	ros::init (argc, argv, NODE_NAME);
	ChangeDetectionROS chdet;
	ros::spin();

	return EXIT_SUCCESS;
}
