#include <v4r/recognition/multiview_object_recognizer_service.h>
#include "recognition_srv_definitions/recognize.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <v4r/changedet/ObjectDetection.h>

namespace v4r
{

class multiviewGraphROS : public MultiviewRecognizer
{
private:
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher image_pub_;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::Publisher vis_pc_pub_;
    ros::ServiceServer recognition_serv_;
    ros::ServiceClient changes_service_client;
    size_t view_counter_;
    std::vector< v4r::ObjectDetection<PointT> > previous_detections;

    bool respondSrvCall (recognition_srv_definitions::recognize::Request & req, recognition_srv_definitions::recognize::Response & response);

    virtual void findRemovedPoints(pcl::PointCloud<PointT>::ConstPtr observation,
    		const Eigen::Affine3f &pose);

public:
    multiviewGraphROS() : MultiviewRecognizer()
    {
        view_counter_ = 0;
    }

    virtual ~multiviewGraphROS() {
    }

    bool recognizeROS (recognition_srv_definitions::recognize::Request & req, recognition_srv_definitions::recognize::Response & response);
    bool initializeMV (int argc, char ** argv);
};

}
