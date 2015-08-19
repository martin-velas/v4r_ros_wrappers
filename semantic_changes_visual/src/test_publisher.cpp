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
#include <pcl_conversions/pcl_conversions.h>

#include <v4r/changedet/Visualizer3D.h>

#include <sematic_changes_visual/ChangedScene.h>

using namespace pcl;
using namespace std;
using namespace sematic_changes_visual;

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
      sphere->points[i].z = 1 - 2 * (pow (rand_x1, 2) + pow (rand_x2, 2));
      sphere->points[i].r = rand() % 50 + 127;
      sphere->points[i].g = rand() % 50 + 127;
      sphere->points[i].b = rand() % 50 + 127;
    }

  Eigen::Translation3f translation(center);
  transformPointCloud(*sphere, *sphere, Eigen::Affine3f(translation));
  return sphere;
}

int main(int argc, char *argv[]) {

  const int POINTS_PER_SPHERE = 500;
  const float SPHERE_RADIUS = 0.1;
  const float DEG_TO_RAD = M_PI / 180.0;
  const int SPHERES = 5;

  //Visualizer3D vis;
  float angle = 0;
  vector<Cloud::Ptr> spheres(SPHERES);
  Cloud::Ptr scene(new Cloud());
  for(int i = 0; i < 5; i++, angle+=72.0) {
    Eigen::Vector3f center(0.5, 0, 0);
    spheres[i] = genColorSphere(center, SPHERE_RADIUS, POINTS_PER_SPHERE*6);
    transformPointCloud(*spheres[i], *spheres[i], getTransformation(0, 0, 0, angle*DEG_TO_RAD, 0, 0));
    //vis.addColorPointCloud(spheres[i]);
    *scene += *spheres[i];
  }
  //vis.show();

  ros::init(argc, argv, "test_of_semantic_changes_vis");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<ChangedScene>("sem_changes", 1);

  ChangedScene sceneMsg;
  toROSMsg(*scene, sceneMsg.scene_cloud);

  SimpleChange add;
  toROSMsg(*spheres[0], add.cloud);
  add.id = 0;
  add.label = "added_sphere";
  sceneMsg.added.push_back(add);

  SimpleChange remove;
  toROSMsg(*spheres[1], remove.cloud);
  remove.id = 1;
  remove.label = "removed_sphere";
  sceneMsg.removed.push_back(remove);

  MoveChange move;
  toROSMsg(*spheres[2], move.cloud_from);
  toROSMsg(*spheres[3], move.cloud_to);
  move.id = 2;
  move.label = "moved_sphere";
  sceneMsg.moved.push_back(move);

  publisher.publish(sceneMsg);
  ros::spin();

  return EXIT_SUCCESS;
}
