/**

**/

#ifndef ARTAG_NAV_COMMANDER_NODE_H
#define ARTAG_NAV_COMMANDER_NODE_H

#include <apriltag_ros/common_functions.h>
#include <apriltag_ros/AnalyzeSingleImage.h>
#include <std_srvs/Trigger.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace artag_nav_commander
{
class ArtagNavCommander
{
    public:
     ArtagNavCommander(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    bool triggerCallBack(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
    void testFunc();
    
    bool analyzeImage();
    void waitForNavstack(MoveBaseClient& ac);
    void sendGoal(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal goal);

    private:
     apriltag_ros::TagDetector tag_detector_;
     apriltag_ros::AprilTagDetectionArray tag_detections_;
     ros::ServiceServer single_image_analysis_service_;
     ros::Publisher tag_detections_publisher_;
     sensor_msgs::CameraInfo camera_info_;

     //MoveBaseClient move_ac_;
};

} // artag_nav_commander

#endif // ARTAG_NAV_COMMANDER_NODE_H
