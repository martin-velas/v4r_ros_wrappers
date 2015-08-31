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

typedef PointXYZRGB PointT;

PointCloud<PointXYZRGB>::Ptr observation;

v4r::ObjectDetection<PointT> loadObjectFromFile(const std::string &filename,
		const Eigen::Affine3f &sensor_pose) {
	typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	pcl::io::loadPCDFile(filename, *cloud);

	boost::filesystem::path file(filename);
	std::string filename_only = file.stem().string();
	std::string t_filename = file.parent_path().string() + "/" + filename_only + ".transformation";
	std::ifstream t_file(t_filename.c_str());
	float tx, ty, tz;
	float qx, qy, qz, qw;
	t_file >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
	Eigen::Quaternionf q(qw, qx, qy, qz);
	Eigen::Translation3f translation(tx, ty, tz);
	Eigen::Affine3f inter_pose(translation * q);

	pcl::transformPointCloud(*cloud, *cloud, inter_pose.inverse());

	std::string claz = filename_only.substr(0, filename_only.find("."));
	static int gen_id = 0;
	return v4r::ObjectDetection<PointT>(claz, gen_id++, cloud, sensor_pose*inter_pose );
}

std::vector< v4r::ObjectDetection<PointT> >
loadObjectsFromDirectory(const std::string &dirname, const Eigen::Affine3f &pose) {
	std::vector< v4r::ObjectDetection<PointT> > objects;

	std::vector<std::string> object_filenames;
	v4r::io::getFilesInDirectory(dirname, object_filenames, "", ".*.pcd", false);
	for(std::vector<std::string>::iterator filename = object_filenames.begin();
			filename < object_filenames.end(); filename++) {
		objects.push_back(
				loadObjectFromFile(dirname + "/" + *filename, pose));
	}
	return objects;
}

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

    map<string, PointCloud<PointXYZRGB>::Ptr > db_clouds;

    while(true) {
		for(int i = 1; i < argc; i++) {
			sleep(3);

			observation.reset(new PointCloud<PointXYZRGB>());
			::io::loadPCDFile(argv[i], *observation);
			Eigen::Affine3f new_pos = v4r::resetViewpoint<PointXYZRGB>(observation);
			vector< v4r::ObjectDetection<PointXYZRGB> > objects = loadObjectsFromDirectory(
						string(argv[i]) + "_results", new_pos);

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
		}
    }

	return EXIT_SUCCESS;
}


