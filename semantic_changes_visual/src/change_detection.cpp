#include <cstdlib>
#include <iostream>

#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <v4r/changedet/Visualizer3D.h>
#include <v4r/changedet/change_detection.h>
#include <v4r/changedet/ObjectDetection.h>
#include <v4r/changedet/ObjectHistory.h>
#include <v4r/changedet/miscellaneous.h>

#include <semantic_changes_visual/ObjectDetectionBridge.h>
#include <semantic_changes_visual/ChangeDetectionBridge.h>
#include <semantic_changes_visual/get_removed_objects.h>
#include <semantic_changes_visual/reset_change_detection.h>
#include <semantic_changes_visual/ChangedScene.h>

using namespace std;
using namespace pcl;

const string NODE_NAME = "semantic_changes_visual";
const string SERVICE_NAME = "removal_detection_service";
const string SERVICE_RESET_NAME = "removal_detection_reset";
const string CHANGES_TOPIC_NAME = "semantic_changes";
const string VIS_FRAME_ID = "sem_changes";

const string SCENE_TOPIC_NAME = CHANGES_TOPIC_NAME + "/scene_reconstructed";
const string ADDED_TOPIC_NAME = CHANGES_TOPIC_NAME + "/added_points";
const string REMOVED_TOPIC_NAME = CHANGES_TOPIC_NAME + "/removed_points";

class ChangeDetectionROS {
public:
	typedef PointXYZRGB PointT;
	typedef PointCloud<PointT> Cloud;

private:
	v4r::ObjectsHistory<PointT>::Ptr history;
	Cloud::Ptr scene, removed, added;
	Eigen::Affine3f first_camera_pose;
	ros::Time now;

	ros::NodeHandle nodeHandler;
	ros::ServiceServer removal_detection_service;
	ros::ServiceServer removal_detection_reset;
	ros::Publisher vis_changes_publisher;
	ros::Publisher scene_publisher;
	ros::Publisher added_publisher;
	ros::Publisher removed_publisher;
	tf::TransformBroadcaster t_broadcaster;

public:
	ChangeDetectionROS() :
		scene(new Cloud()), added(new Cloud()), removed(new Cloud()),
		history(new v4r::ObjectsHistory<PointT>()),
		nodeHandler("~") {
		removal_detection_service = nodeHandler.advertiseService(
				SERVICE_NAME, &ChangeDetectionROS::detect_removed_objects, this);
		removal_detection_reset = nodeHandler.advertiseService(
				SERVICE_RESET_NAME, &ChangeDetectionROS::reset, this);
		vis_changes_publisher =
				nodeHandler.advertise<semantic_changes_visual::ChangedScene>(CHANGES_TOPIC_NAME, 10);
		scene_publisher =
				nodeHandler.advertise<sensor_msgs::PointCloud2>(SCENE_TOPIC_NAME, 10);
		added_publisher =
				nodeHandler.advertise<sensor_msgs::PointCloud2>(ADDED_TOPIC_NAME, 10);
		removed_publisher =
				nodeHandler.advertise<sensor_msgs::PointCloud2>(REMOVED_TOPIC_NAME, 10);
	}

	bool detect_removed_objects(semantic_changes_visual::get_removed_objects::Request & req,
			semantic_changes_visual::get_removed_objects::Response & response) {

		Cloud::Ptr observation(new Cloud());
		pcl::fromROSMsg(req.observation, *observation);
		observation = v4r::downsampleCloud<PointXYZRGB>(observation);
		Eigen::Affine3f camera_pose;
		ObjectDetectionBridge::transformationFromROSMsg(req.camera_pose, camera_pose);

		vector< v4r::ObjectDetection<PointT> > detected_objects;
		for(int i = 0; i < req.objects.size(); i++) {
			detected_objects.push_back(
					ObjectDetectionBridge::fromROSMsg<PointXYZRGB>(req.objects[i]));
		}

		PCL_INFO("======================================================================\n");
		added->clear();
		removed->clear();
		if(!scene->empty()) {
			v4r::ChangeDetector<PointT> detector;
			detector.detect(scene, observation, camera_pose, 0.03);
			v4r::ChangeDetector<PointT>::removePointsFrom(scene, detector.getRemoved());

			vector<v4r::ObjectIdLabeled> removed_objects = history->markRemovedObjects(detector);
			for(unsigned i = 0; i < removed_objects.size(); i++) {
				v4r::ObjectLabel label = removed_objects[i].label;
				int id = removed_objects[i].id;
				v4r::ChangeDetector<PointT>::removePointsFrom(scene,
						history->getLastCloudOf(label));
				semantic_changes_visual::ObjectLabel label_msg;
				label_msg.id = id;
				label_msg.label = label;
				response.removed_labels.push_back(label_msg);
			}
			*added += *(detector.getAdded());
			*removed += *(detector.getRemoved());
			/*static v4r::Visualizer3D vis;
			vis.keepOnlyClouds(0).setColor(255, 0, 255).addPointCloud(*removed)
					.addColorPointCloud(scene)
					.addColorPointCloud(observation).show();*/
			*scene += *added;
			/*
			v4r::Visualizer3D().addColorPointCloud(scene)
					.setColor(255, 0, 0).addPointCloud(*removed)
					.setColor(0, 255, 0).addPointCloud(*added).show();
			 */
			toROSMsg(*removed, response.removed_points);
			toROSMsg(*added, response.added_points);
		} else {
			ROS_INFO_STREAM("First observation");
			*scene += *observation;
			first_camera_pose = camera_pose;
			*added += *observation;
			toROSMsg(*added, response.added_points);
		}

		history->add(detected_objects);

		Cloud::Ptr table(new Cloud());
		Eigen::VectorXf coeff;
		if(findPlane(table, coeff)) {
			publishChangesVisual(getTransformationPlaneToHorizontal(table, coeff));
		} else {
			publishChangesVisual(first_camera_pose);
		}

		v4r::ObjectsHistory<PointXYZRGB>::incrementTime();

		return true;
	}

	bool reset(semantic_changes_visual::reset_change_detection::Request & req,
				semantic_changes_visual::reset_change_detection::Response & response) {
		scene.reset(new pcl::PointCloud<PointXYZRGB>());
		history.reset(new v4r::ObjectsHistory<PointXYZRGB>());
		return true;
	}

	void publishChangesVisual(const Eigen::Affine3f &pose) {
		now = ros::Time::now();

		tf::Transform transform;
		Eigen::Matrix4f pose_matrix = pose.matrix();
		Eigen::Affine3d pose_double(pose_matrix.cast<double>());
		tf::transformEigenToTF(pose_double.inverse(), transform);
		t_broadcaster.sendTransform(tf::StampedTransform(transform, now, VIS_FRAME_ID, "map"));

		semantic_changes_visual::ChangedScene changesMsg;
		changesMsg.header.frame_id = VIS_FRAME_ID;
		changesMsg.header.stamp = now;

		fillChanges(v4r::ObjectState::ADDED, changesMsg.added);
		fillChanges(v4r::ObjectState::REMOVED, changesMsg.removed);
		fillChanges(v4r::ObjectState::PRESERVED, changesMsg.preserved);
		fillChanges(changesMsg.moved);

		vis_changes_publisher.publish(changesMsg);

		publishCloud(scene, scene_publisher);
		publishCloud(added, added_publisher);
		publishCloud(removed, removed_publisher);
	}

	void publishCloud(Cloud::Ptr cloud, ros::Publisher &publisher) {
		sensor_msgs::PointCloud2 cloudMsg;
		toROSMsg(*cloud, cloudMsg);
		cloudMsg.header.frame_id = VIS_FRAME_ID;
		cloudMsg.header.stamp = now;
		publisher.publish(cloudMsg);
	}

	void fillChanges(const v4r::ObjectState::EventT event,
			vector<semantic_changes_visual::SimpleChange> &changes_msg) {
		vector<v4r::ObjectChangeForVisual<pcl::PointXYZRGB> > changes = history->getChanges(event);
		changes_msg.resize(changes.size());
		for(int i = 0; i < changes.size(); i++) {
			ROS_INFO_STREAM("Object " << changes[i].label << " was [" <<
					boost::to_upper_copy(v4r::ObjectState::string(event)) << "]");
			ChangeDetectionBridge::toROSMsg(changes[i], changes_msg[i]);
		}
	}

	void fillChanges(vector<semantic_changes_visual::MoveChange> &changes_msg) {
		vector<v4r::ObjectChangeForVisual<pcl::PointXYZRGB> > changes = history->getChanges(
			v4r::ObjectState::MOVED);
		changes_msg.resize(changes.size());
		for(int i = 0; i < changes.size(); i++) {
			ROS_INFO_STREAM("Object " << changes[i].label << " has [MOVED]");
			ChangeDetectionBridge::toROSMsg(changes[i], changes_msg[i]);
		}
	}

	bool findPlane(Cloud::Ptr plane, Eigen::VectorXf &coeff) {
		SampleConsensusModelPlane<PointT>::Ptr model_p(
				new SampleConsensusModelPlane<PointT>(scene));
	    std::vector<int> inliers;
		pcl::RandomSampleConsensus<PointT> ransac(model_p);
	    ransac.setDistanceThreshold(.01);
	    if(!ransac.computeModel()) {
	    	return false;
	    }
	    ransac.getInliers(inliers);
	    copyPointCloud<PointT>(*scene, inliers, *plane);
	    ransac.getModelCoefficients(coeff);
	    return true;
	}

	// coeff = [nx, nx, nz, d] of the plane
	Eigen::Affine3f getTransformationPlaneToHorizontal(Cloud::ConstPtr plane,
			const Eigen::VectorXf &coeff, float above_ground = 0.0) {

		Eigen::Vector3f src_norm(coeff(0), coeff(1), coeff(2));
		Eigen::Vector3f dest_norm(0, coeff(1), 0);
		Eigen::Quaternionf R = Eigen::Quaternionf().setFromTwoVectors(src_norm, dest_norm);

		Eigen::Vector4f centroidH;
		compute3DCentroid(*plane, centroidH);
		Eigen::Vector3f centroid(centroidH(0), centroidH(1), centroidH(2));
		Eigen::Affine3f t = R*Eigen::Translation3f::Identity();
		pcl::transformPoint(centroid, centroid, t);

		t = R*Eigen::Translation3f(0, -above_ground-centroid(1), 0);
		return t;
	}
};

int main(int argc, char *argv[]) {

	ros::init (argc, argv, NODE_NAME);
	ChangeDetectionROS chdet;
	ros::spin();

	return EXIT_SUCCESS;
}
