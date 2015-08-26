#include <cstdlib>
#include <iostream>

#include <pcl/common/eigen.h>
#include <pcl/filters/voxel_grid.h>

#include <v4r/changedet/Visualizer3D.h>
#include <v4r/changedet/change_detection.h>
#include <v4r/changedet/ObjectDetection.h>
#include <v4r/changedet/ObjectHistory.h>
#include <v4r/changedet/miscellaneous.h>

using namespace std;
using namespace pcl;

int main(int argc, char *argv[]) {

	if(argc < 2) {
		PCL_INFO("Insufficient number of arguments, expected <pcd-cloud>+\n");
		return EXIT_FAILURE;
	}

	PointCloud<PointXYZRGB>::Ptr scene(new PointCloud<PointXYZRGB>());
	::io::loadPCDFile(argv[1], *scene);
	scene = downsampleCloud<PointXYZRGB>(scene);
	Eigen::Affine3f scene_pos = resetViewpoint<PointXYZRGB>(scene);
	vector< ObjectDetection<PointXYZRGB> > objects = loadObjects<PointXYZRGB>(
			string(argv[1]) + "_results", scene_pos);
	Visualizer3D().addColorPointCloud(scene).addObjects(objects).show();
	ObjectsHistory<PointXYZRGB> history;
	history.add(objects);

	for(int i = 2; i < argc; i++) {
		PointCloud<PointXYZRGB>::Ptr observation(new PointCloud<PointXYZRGB>());
		::io::loadPCDFile(argv[i], *observation);
		observation = downsampleCloud<PointXYZRGB>(observation);
		Eigen::Affine3f new_pos = resetViewpoint<PointXYZRGB>(observation);

		ChangeDetector<PointXYZRGB> detector;
		detector.detect(scene, observation, new_pos);

		ChangeDetector<PointXYZRGB>::removePointsFrom(scene, detector.getRemoved());

		PCL_INFO("======================== CHANGES: ==========================\n");
		vector<ObjectLabel> changed = history.getRemovedObjects(detector);
		for(unsigned i = 0; i < changed.size(); i++) {
			ObjectLabel label = changed[i];
			PCL_INFO("Object event: %s has been [REMOVED]\n", label.c_str());
			ChangeDetector<PointXYZRGB>::removePointsFrom(scene, history.getLastCloudOf(label));
		}
		PCL_INFO("============================================================\n");

		*scene += *(detector.getAdded());

		Visualizer3D vis;
		vis.addColorPointCloud(scene);
		vis.setColor(0, 255, 0).addPointCloud(*detector.getAdded());
		vis.setColor(255, 0, 0).addPointCloud(*detector.getRemoved());
		Eigen::Vector3f viewport_scene = (scene_pos).translation();
		Eigen::Vector3f viewport_obser = (new_pos).translation();
		vis.getViewer()->addSphere(PointXYZ(viewport_scene(0), viewport_scene(1), viewport_scene(2)),
						0.1, 1.0, 0, 0, "source-pos");
		vis.getViewer()->addSphere(PointXYZ(viewport_obser(0), viewport_obser(1), viewport_obser(2)),
						0.1, 0, 1.0, 0, "target-pos");
		vis.show();

		Visualizer3D().addColorPointCloud(scene).show();
	}

	return EXIT_SUCCESS;
}
