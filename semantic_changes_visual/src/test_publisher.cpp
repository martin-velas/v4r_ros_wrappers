/*
 * test_publisher.cpp
 *
 *  Created on: 19.8.2015
 *      Author: ivelas
 */

#include <cstdlib>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <v4r/changedet/Visualizer3D.h>

#include <semantic_changes_visual/ChangedScene.h>
#include <ObjectDetectionBridge.h>

using namespace pcl;
using namespace std;
using namespace semantic_changes_visual;

typedef PointXYZRGB PointT;
typedef PointCloud<PointT> Cloud;

Cloud::Ptr genColorSphere(Eigen::Vector3f center, float radius, int points) {
  Cloud::Ptr sphere(new Cloud());

  // 1. Generate cloud data
  sphere->width = points;
  sphere->height = 1;
  sphere->points.resize (sphere->width * sphere->height);
    // 1.2 Add sphere:
  double rand_x1, rand_x2;
    for (size_t i = 0; i < points; i++)
    {
      // See: http://mathworld.wolfram.com/SpherePointPicking.html
      do {
        rand_x1 = (rand () % 100) / (50.0f) - 1;
        rand_x2 = (rand () % 100) / (50.0f) - 1;
      } while (pow (rand_x1, 2) + pow (rand_x2, 2) >= 1);

      double pre_calc = sqrt (radius - pow (rand_x1, 2) - pow (rand_x2, 2));
      sphere->points[i].x = 2 * rand_x1 * pre_calc;
      sphere->points[i].y = 2 * rand_x2 * pre_calc;
      sphere->points[i].z = - 2 * (pow (rand_x1, 2) + pow (rand_x2, 2));
      sphere->points[i].r = rand() % 255;
      sphere->points[i].g = rand() % 255;
      sphere->points[i].b = rand() % 255;
    }

  Eigen::Translation3f translation(center);
  transformPointCloud(*sphere, *sphere, Eigen::Affine3f(translation));
  return sphere;
}

Cloud::Ptr genGround(float width, float depth, int points) {
	Cloud::Ptr ground(new Cloud());
	ground->width = points;
	ground->height = 1;
	ground->points.resize(ground->width * ground->height);
	double rand_x1, rand_x2;
	for (size_t i = 0; i < points; i++) {
		float x = (rand() / (float)RAND_MAX) * width*2 - width;
		float y = (rand() / (float)RAND_MAX) * depth*2 - width;
		ground->points[i].x = x;
		ground->points[i].y = y;
		ground->points[i].z = 0;
		int gray = 100 + rand() % 60;
		ground->points[i].r = gray;
		ground->points[i].g = gray;
		ground->points[i].b = gray;
	}
	return ground;
}

int main(int argc, char *argv[]) {

  const int POINTS_PER_SPHERE = 10000;
  const float SPHERE_RADIUS = 0.2;
  const float DEG_TO_RAD = M_PI / 180.0;
  const int SPHERES = 6;
  const float ANGLE_DELTA = 360 / SPHERES;

  //Visualizer3D vis;
  float angle = 0;
  vector<Eigen::Affine3f> transforms(SPHERES);
  Cloud::Ptr scene(new Cloud());
  Eigen::Vector3f center(1, 0, sqrt(SPHERE_RADIUS));
  Cloud::Ptr sphere = genColorSphere(center, SPHERE_RADIUS, POINTS_PER_SPHERE);

  for(int i = 0; i < SPHERES; i++, angle+=ANGLE_DELTA) {
	transforms[i] = getTransformation(0, 0, 0, 0, 0, angle*DEG_TO_RAD);
	Cloud sphere_posed;
	transformPointCloud(*sphere, sphere_posed, transforms[i]);
	*scene += sphere_posed;
  }
  Cloud::Ptr ground = genGround(2, 2, 100000);
  *scene += *ground;

  ros::init(argc, argv, "test_of_semantic_changes_vis");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<ChangedScene>("sem_changes", 10);
  tf::TransformBroadcaster t_broadcaster;
  tf::Transform identity; identity.setIdentity();

  ChangedScene sceneMsg;
  toROSMsg(*scene, sceneMsg.scene_cloud);

  SimpleChange add;
  toROSMsg(*sphere, add.cloud);
  ObjectDetectionBridge::transformationToROSMsg(transforms[0], add.pose);
  add.id = 0;
  add.label = "added_sphere";
  sceneMsg.added.push_back(add);

  SimpleChange remove;
  toROSMsg(*sphere, remove.cloud);
  ObjectDetectionBridge::transformationToROSMsg(transforms[1], remove.pose);
  remove.id = 1;
  remove.label = "removed_sphere";
  sceneMsg.removed.push_back(remove);

  MoveChange move;
  toROSMsg(*sphere, move.cloud);
  ObjectDetectionBridge::transformationToROSMsg(transforms[2], move.pose_from);
  ObjectDetectionBridge::transformationToROSMsg(transforms[3], move.pose_to);
  move.id = 2;
  move.label = "moved_sphere";
  sceneMsg.moved.push_back(move);

  SimpleChange preserve;
  toROSMsg(*sphere, preserve.cloud);
  ObjectDetectionBridge::transformationToROSMsg(transforms[4], preserve.pose);
  preserve.id = 3;
  preserve.label = "preserved_sphere";
  sceneMsg.preserved.push_back(preserve);

  ros::Rate loop_rate(1);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
	ros::Time now = ros::Time::now();
    sceneMsg.header.frame_id = "test_changes";
    sceneMsg.header.stamp = now;

    t_broadcaster.sendTransform(tf::StampedTransform(identity, now, "test_changes", "map"));
    publisher.publish(sceneMsg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return EXIT_SUCCESS;
}
