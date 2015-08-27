/*
 * test_change_detection.cpp
 *
 *  Created on: 26 Aug 2015
 *      Author: martin
 */

#include <cstdlib>
#include <iostream>

#include <ros/ros.h>

#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>

#include <v4r/changedet/Visualizer3D.h>
#include <v4r/changedet/change_detection.h>
#include <v4r/changedet/ObjectDetection.h>
#include <v4r/changedet/ObjectHistory.h>
#include <v4r/changedet/miscellaneous.h>

#include <semantic_changes_visual/get_removed_objects.h>
#include <ObjectDetectionBridge.h>

using namespace std;
using namespace pcl;

int main(int argc, char *argv[]) {

	if(argc < 2) {
		PCL_INFO("Insufficient number of arguments, expected <pcd-cloud>+\n");
		return EXIT_FAILURE;
	}

    ros::init (argc, argv, "TestSemanticChangeDetection");
    ros::NodeHandle node("~");

    std::string service_name_sv_rec = "/semantic_changes_visual/removal_detection_service";
    ros::ServiceClient service_client =
    		node.serviceClient<semantic_changes_visual::get_removed_objects>(service_name_sv_rec);

    while(true) {
		for(int i = 1; i < argc; i++) {
			PointCloud<PointXYZRGB>::Ptr observation(new PointCloud<PointXYZRGB>());
			::io::loadPCDFile(argv[i], *observation);
			Eigen::Affine3f new_pos = v4r::resetViewpoint<PointXYZRGB>(observation);
			vector< v4r::ObjectDetection<PointXYZRGB> > objects = v4r::loadObjects<PointXYZRGB>(
						string(argv[i]) + "_results", new_pos);

			v4r::Visualizer3D().addColorPointCloud(observation)
					.addPointCloud(*(objects[0].getCloud(false))).show();

			semantic_changes_visual::get_removed_objects service_call;
			toROSMsg(*observation, service_call.request.observation);
			service_call.request.objects.resize(objects.size());
			for(int i = 0; i < objects.size(); i++) {
				ObjectDetectionBridge::toROSMsg(objects[i], service_call.request.objects[i]);
			}
			ObjectDetectionBridge::transformationToROSMsg(new_pos, service_call.request.camera_pose);

			if (!service_client.call(service_call)) {
				std::stringstream mm;
				ROS_ERROR("Error calling removal detection service.\n");
				return false;
			} else {
				ROS_INFO_STREAM("Following object has been removed from the scene:\n");
				for(int i = 0; i < service_call.response.removed_labels.size(); i++) {
					ROS_INFO_STREAM("\t" << service_call.response.removed_labels[i] << "\n");
				}
			}
			sleep(3);
		}
    }

	return EXIT_SUCCESS;
}


