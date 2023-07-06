/**

**/

#include "artag_nav_commander/artag_nav_commander_node.h"

#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>

namespace artag_nav_commander
{

ArtagNavCommander::ArtagNavCommander(ros::NodeHandle& nh, ros::NodeHandle& pnh): tag_detector_(pnh){ 
  //testFunc();
  single_image_analysis_service_ =
    nh.advertiseService("single_image_tag_detection",
                        &ArtagNavCommander::triggerCallBack, this);
  tag_detections_publisher_ =
    nh.advertise<apriltag_ros::AprilTagDetectionArray>("tag_detections", 1);
  ROS_INFO_STREAM("Ready to do tag detection on single images");

  //MoveBaseClient ac_local("fdf", true);
  //move_ac_ = ac_local;
  //waitForNavstack(mv);

}

bool ArtagNavCommander::triggerCallBack(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) { 
  //testFunc();
  analyzeImage();
  response.success = true;
  response.message = "Triggerd!";
  return true;
}


void ArtagNavCommander::waitForNavstack(MoveBaseClient& mv_ac) {
    
  while(!mv_ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server ... ");
  }   
}

void ArtagNavCommander::sendGoal(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal goal) {
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);   

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The ifollow AMR has reached the target!");
  else
    ROS_INFO("Failed to reach the target! Exiting");
         
}

bool ArtagNavCommander::analyzeImage(){

  std::string full_path_where_to_get_image="/home/devcyclair/noetic_ws/src/apriltag_ros/apriltag_ros/ar_tags/ar_webcam.png";
  std::string full_path_where_to_save_image="/home/devcyclair/noetic_ws/src/apriltag_ros/apriltag_ros/ar_tags/ar_webcam_drawn.png";
  sensor_msgs::CameraInfo camera_info;
  camera_info.distortion_model = "plumb_bob";

  double fx = 643.651478;
  double fy = 644.265346;
  double cx = 304.4428;
  double cy = 226.340608;


  camera_info.K[0] = fx;
  camera_info.K[2] = cx;
  camera_info.K[4] = fy;
  camera_info.K[5] = cy;
  camera_info.K[8] = 1.0;
  camera_info.P[0] = fx;
  camera_info.P[2] = cx;
  camera_info.P[5] = fy;
  camera_info.P[6] = cy;
  camera_info.P[10] = 1.0;

  ROS_INFO("[ Summoned to analyze image ]");
  ROS_INFO("Image load path: %s",
           full_path_where_to_get_image.c_str());
  ROS_INFO("Image save path: %s",
           full_path_where_to_save_image.c_str());

  // Read the image
  cv::Mat image = cv::imread(full_path_where_to_get_image,
                             cv::IMREAD_COLOR);
  if (image.data == NULL) {
    // Cannot read image
    ROS_ERROR_STREAM("Could not read image " <<
                     full_path_where_to_get_image.c_str());
    return false;
  }

  // Detect tags in the image
  cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::Header(),
                                                            "bgr8", image));
  loaded_image->header.frame_id = "camera";

  tag_detections_ =
      tag_detector_.detectTags(loaded_image,sensor_msgs::CameraInfoConstPtr(
          new sensor_msgs::CameraInfo(camera_info)));

  // Publish detected tags (AprilTagDetectionArray, basically an array of
  // geometry_msgs/PoseWithCovarianceStamped)
  tag_detections_publisher_.publish(tag_detections_);

  // Send goal to the navstack
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>move_ac("move_base", true);
  // Wait for the action server to come up so that we can begin processing goals.
  while(!move_ac.waitForServer(ros::Duration(0.50))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  // Convert  pose in camera frame to the base_link
  goal.target_pose.pose.position.x = tag_detections_.detections[0].pose.pose.pose.position.z;
  goal.target_pose.pose.position.y = -tag_detections_.detections[0].pose.pose.pose.position.x;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  sendGoal(move_ac, goal);

  // Save tag detections image
  tag_detector_.drawDetections(loaded_image);
  cv::imwrite(full_path_where_to_save_image, loaded_image->image);

  ROS_INFO("Done!\n");

  return true;
}

} // namespace artag_nav_commander



int main(int argc, char **argv)
{
  ros::init(argc, argv, "artag_nav_commander");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  artag_nav_commander::ArtagNavCommander ar_commander(nh, pnh);
  ar_commander.analyzeImage();
  
  ros::spin();
}
