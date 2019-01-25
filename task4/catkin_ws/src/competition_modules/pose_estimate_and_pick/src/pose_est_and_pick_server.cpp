#include "ros/ros.h"
#include <ros/console.h>
#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl/common/centroid.h"
#include "pcl_ros/point_cloud.h"
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include "pose_estimate_and_pick/pose_est_and_pick.h"
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/CollisionObject.h>


using namespace std;
using namespace cv;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointXYZRGB;
const float nan_value = std::numeric_limits<float>::quiet_NaN();

Eigen::Vector3f normalize_vec(Eigen::Vector3f input_v){
  double mag = sqrt(input_v[0]*input_v[0] + input_v[1]*input_v[1] + input_v[2]*input_v[2]);
  Eigen::Vector3f output_v = input_v/mag;
  return (output_v);
}

Eigen::Matrix3f get_rot_matrix(double theta, double rotate_theta){
  Eigen::Vector3f acc(0,-sin(theta), cos(theta));
  Eigen::Matrix3f rot_mat;
  double vers = 1-cos(rotate_theta);
  rot_mat(0, 0) = acc[0] * acc[0] * vers + cos(rotate_theta);
  rot_mat(0, 1) = acc[0] * acc[1] * vers - acc[2] * sin(rotate_theta);
  rot_mat(0, 2) = acc[0] * acc[2] * vers + acc[1] * sin(rotate_theta);
  rot_mat(1, 0) = acc[0] * acc[1] * vers + acc[2] * sin(rotate_theta);
  rot_mat(1, 1) = acc[1] * acc[1] * vers + cos(rotate_theta);
  rot_mat(1, 2) = acc[1] * acc[2] * vers - acc[0] * sin(rotate_theta);
  rot_mat(2, 0) = acc[0] * acc[2] * vers - acc[1] * sin(rotate_theta);
  rot_mat(2, 1) = acc[1] * acc[2] * vers + acc[0] * sin(rotate_theta);
  rot_mat(2, 2) = acc[2] * acc[2] * vers + cos(rotate_theta);
  return rot_mat;
}

void adjust_obj_pose(Eigen::Vector3f &input_x, Eigen::Vector3f &input_y, double theta, int obj_class){
  Eigen::Vector3f x_unit( 1, 0, 0);
  Eigen::Vector3f y_unit( 0, 1, 0);
  Eigen::Vector3f output_x;
  Eigen::Vector3f output_y;
  double proj_x = x_unit.dot(input_x);
  double proj_y = y_unit.dot(input_x);
  double rotate_theta;
  if(obj_class==0){
    if (proj_x <=0){
      rotate_theta = M_PI;
    }
    else{
      return;
    }
  }

  else if (obj_class==2){
    cout << "x: "<< proj_x << "y: "<< proj_y <<endl;
    if (atan2(proj_y, proj_x) <= M_PI/4 && atan2(proj_y, proj_x) >= -M_PI/4){
      rotate_theta = 0;
      return;
    }
    else if(atan2(proj_y, proj_x) >= -3.0*M_PI/4 && atan2(proj_y, proj_x) <= -M_PI/4){
      rotate_theta = 3.0*M_PI/2;
    }
    else if(atan2(proj_y, proj_x) <= -3.0*M_PI/4 || atan2(proj_y, proj_x) >= 3.0*M_PI/4){
      rotate_theta = M_PI;
    }
    else if(atan2(proj_y, proj_x) <= 3.0*M_PI/4 && atan2(proj_y, proj_x) >= 1.0*M_PI/4){
      rotate_theta = 1.0*M_PI/2;
    }
  }

  cout << "theta: "<< rotate_theta/M_PI << endl;

  Eigen::Matrix3f rot_mat = get_rot_matrix(theta, rotate_theta);

  input_y = rot_mat * input_y;
  input_x = rot_mat * input_x;
  // Eigen::Matrix3f rot_z;
  // rot_z << 1,0,0,0,cos(theta),-sin(theta),0,sin(theta),cos(theta);
  // Eigen::Matrix3f rot_inv;
  // rot_inv << -1,0,0,0,-1,0,0,0,0;
  // double flag = input_x.dot(z) ;
  // if (flag < 0){
  //   Eigen::Vector3f output_x = rot_inv * (rot_z * theta);
  // }
  return;
}


class Pose_est_and_pick_server{

  public:
    Pose_est_and_pick_server();
    bool server_cb(pose_estimate_and_pick::pose_est_and_pick::Request &req,\
      pose_estimate_and_pick::pose_est_and_pick::Response &res);
  private:
    ros::NodeHandle n;
    ros::ServiceServer service ;
    vector<tf::Transform> object_pose_list;
    vector<int> object_class_list;
    vector<Mat> split_masks(Mat mask);
    bool pick_object(vector<tf::Transform> obj_tf);
    void pose_estimation(int label,vector<tf::Transform> &tf_list, \
      vector<int> &object_class_list, cv::Mat &mask,PointXYZRGB::Ptr cloud,int object_class);
    bool pick_object(tf::Transform obj_tf);
};

Pose_est_and_pick_server::Pose_est_and_pick_server(){
  //Initailize Pose_est_and_pick_server
  service = n.advertiseService("pose_est_and_pick_server", &Pose_est_and_pick_server::server_cb, this);

  ROS_INFO("Finish initializetion");
}

bool Pose_est_and_pick_server::server_cb(\
  pose_estimate_and_pick::pose_est_and_pick::Request &req,\
  pose_estimate_and_pick::pose_est_and_pick::Response &res){
  ROS_INFO("Pose estimation start");
  PointXYZRGB::Ptr cloud(new PointXYZRGB);
  pcl::fromROSMsg(req.pc, *cloud);
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(req.mask, sensor_msgs::image_encodings::BGR8);
  vector<Mat> masks = this->split_masks(cv_ptr->image);
  ROS_INFO("Calculate start");
  for (int i = 0; i < 3;i++){
    Mat mask(masks[i]);
    Mat binary_mask;
    // imwrite ("output1.jpg ", mask);
    threshold( mask, binary_mask, 100,255,THRESH_BINARY );
    // imwrite ("output2.jpg ", binary_mask);
    Mat labelImage(cv_ptr->image.size(), CV_32S);
    Mat stats, centroids;
    int nLabels = connectedComponentsWithStats(binary_mask, labelImage, stats, centroids, 8, CV_32S);
    printf ("nlabels: %d\n",nLabels);
    for (int label = 1; label < nLabels; label++ ){
      this->pose_estimation(label, object_pose_list, object_class_list, labelImage, cloud, i);
    }
  }
  ROS_INFO("Calculate finish");
  static tf::TransformBroadcaster br;
  ros::Rate loop_rate(10);
  /*while (ros::ok()){
     for (int i=0; i<object_pose_list.size(); i++){
       string reg;
       stringstream id(reg);
      id << i;
       string target = "object" + id.str();
       br.sendTransform(tf::StampedTransform(object_pose_list[i], ros::Time::now(), "camera_color_optical_frame", target));
     }
     loop_rate.sleep();
  }*/
  pick_object(object_pose_list);
  return true;

}

vector<Mat> Pose_est_and_pick_server::split_masks(Mat mask){
  vector<Mat> rgbChannels(3);
  split(mask, rgbChannels);
  return rgbChannels;
}

void Pose_est_and_pick_server::pose_estimation(\
  int label,\
  vector<tf::Transform> &tf_list, \
  vector<int> &object_class_list,\
  cv::Mat &mask,\
  PointXYZRGB::Ptr cloud,\
  int object_class){

  PointXYZRGB::Ptr pc_input(new PointXYZRGB);
  pcl::copyPointCloud(*cloud, *pc_input);
  for (int row=0; row<mask.rows; row++){
    for(int col=0;col<mask.cols;col++){
      if(mask.at<int>(row,col)!=label){
        pc_input->points[row*640+col].x = nan_value;
        pc_input->points[row*640+col].y = nan_value;
        pc_input->points[row*640+col].z = nan_value;
      }
    }
  }
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pc_input, *pc_input, indices);
  Eigen::Vector4f centroid;
  cout << "--------------------------------------------" << endl;
  //printf("Point Size: %d\n",pc_input->points.size());
  if (pc_input->points.size() < 100)
    return;
  pcl::compute3DCentroid (*pc_input, centroid);
  cout << "centroid:" << centroid[0] << " " << centroid[1] << " " << centroid[2] <<endl;
  Eigen::Matrix3f covariance;
  pcl::computeCovarianceMatrixNormalized(*pc_input, centroid, covariance);

  Eigen::Vector3f x;
  Eigen::Vector3f y;
  Eigen::Vector3f z( 0, -1.0*sqrt(2)/2, -1.0*sqrt(2)/2);
  if (object_class==0){
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    x << eigenVectorsPCA(2,0), eigenVectorsPCA(2,1), eigenVectorsPCA(2,2);
    y = z.cross(x);
    x = y.cross(z);
    x = normalize_vec(x);
    y = normalize_vec(y);
    adjust_obj_pose(x, y, 3.0/4*M_PI, object_class);
    // cout << "Eigen vectors" << endl;
    // cout << eigenVectorsPCA(0,0) << " " << eigenVectorsPCA(0,1) << " " << eigenVectorsPCA(0,2) << endl;
    // cout << eigenVectorsPCA(1,0) << " " << eigenVectorsPCA(1,1) << " " << eigenVectorsPCA(1,2) << endl;
    // cout << eigenVectorsPCA(2,0) << " " << eigenVectorsPCA(2,1) << " " << eigenVectorsPCA(2,2) << endl;
    // cout << "Eigen values" << endl;
    cout << x << endl;
    cout << y << endl;
    cout << z << endl;
  }
  else if(object_class==1){
    x << 1, 0, 0;
    y = z.cross(x);
    // adjust_obj_pose(x, y, 3.0/4*M_PI);
  }
  else{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    x << eigenVectorsPCA(2,0), eigenVectorsPCA(2,1), eigenVectorsPCA(2,2);
    y = z.cross(x);
    x = y.cross(z);
    x = normalize_vec(x);
    y = normalize_vec(y);
    adjust_obj_pose(x, y, 3.0/4*M_PI, object_class);
    // cout << "Eigen vectors" << endl;
    // cout << eigenVectorsPCA(0,0) << " " << eigenVectorsPCA(0,1) << " " << eigenVectorsPCA(0,2) << endl;
    // cout << eigenVectorsPCA(1,0) << " " << eigenVectorsPCA(1,1) << " " << eigenVectorsPCA(1,2) << endl;
    // cout << eigenVectorsPCA(2,0) << " " << eigenVectorsPCA(2,1) << " " << eigenVectorsPCA(2,2) << endl;
    // cout << "Eigen values" << endl;
    // cout << eigenValuesPCA << endl;
    cout << x << endl;
    cout << y << endl;
    cout << z << endl;
  }

  tf::Vector3 origin;
  tf::Matrix3x3 rotation;
  origin.setValue(centroid[0], centroid[1], centroid[2]);
  rotation.setValue(x[0], y[0], z[0],
                    x[1], y[1], z[1],
                    x[2], y[2], z[2]);
  tf::Transform tf_obj(rotation,origin);
  origin.setValue(0,0,0);
  rotation.setRPY(0,M_PI/2,M_PI/2 );
  tf::Transform tf_rot(rotation,origin);
  tf_obj = tf_obj * tf_rot;
  tf_list.push_back(tf_obj);
  object_class_list.push_back(object_class);
  return;
}


bool Pose_est_and_pick_server::pick_object(vector<tf::Transform> obj_tf){
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setGoalTolerance(0.05);
  move_group.setMaxVelocityScalingFactor(0.3); 
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  const robot_state::JointModelGroup* joint_model_group =  move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  //ros::AsyncSpinner spinner(1);
 // spinner.start();
  tf::TransformListener listener;
  tf::StampedTransform transform;
  ros::Time now = ros::Time::now();
  ROS_INFO("Start");
  listener.waitForTransform("/car_base", "/camera_color_optical_frame", now, ros::Duration(3.0));
  listener.lookupTransform("/car_base", "/camera_color_optical_frame", ros::Time(0), transform);
  ROS_INFO("end");
  bool success = false;
  geometry_msgs::Pose target_pose;
  int obj_id = 0;
  tf::Quaternion q_base_to_cam = transform.getRotation();
  tf::Matrix3x3 rot_base_to_cam(q_base_to_cam);
  tf::Vector3 tra_base_to_cam = transform.getOrigin();
  Eigen::Matrix4f tf_matrix_base_to_cam;
  tf_matrix_base_to_cam << rot_base_to_cam[0][0], rot_base_to_cam[0][1], rot_base_to_cam[0][2],tra_base_to_cam[0],
                           rot_base_to_cam[1][0], rot_base_to_cam[1][1], rot_base_to_cam[1][2],tra_base_to_cam[1],
                           rot_base_to_cam[2][0], rot_base_to_cam[2][1], rot_base_to_cam[2][2],tra_base_to_cam[2],
                           0,0,0,1;
  ROS_INFO("POSE SIZE:%d", obj_tf.size());
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = 0;  // radians
  joint_group_positions[1] = 0;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = 0;
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success){
  	move_group.execute(my_plan);
  	ros::Duration(2).sleep();
  }
  //move_group.execute(my_plan);
  //ros::Duration(5).sleep();

std::vector<double> group_variable_values1;
    move_group.getCurrentState()->copyJointGroupPositions(move_group.getCurrentState()->getRobotModel()->getJointModelGroup(move_group.getName()), group_variable_values1);
   
    group_variable_values1[0] = 0;
    group_variable_values1[1] = 0;
    group_variable_values1[2] = 0;
    group_variable_values1[3] = 0;

    robot_state::RobotState start_state(*move_group.getCurrentState());
    start_state.setJointGroupPositions(joint_model_group, group_variable_values1);
    move_group.setStartState(start_state);

   
  while(!success && obj_id<obj_tf.size()){
    tf::Transform tf_test = transform * obj_tf[obj_id];
    tf::Quaternion tfqt = tf_test.getRotation();
    tf::Vector3 tra_base_to_obj = tf_test.getOrigin();
    //tf3d_base_to_obj.getRotation(tfqt);
    target_pose.orientation.x = tfqt.x();
    target_pose.orientation.y = tfqt.y();
    target_pose.orientation.z = tfqt.z();
    target_pose.orientation.w = tfqt.w();
    target_pose.position.x = tra_base_to_obj.getX();
    target_pose.position.y = tra_base_to_obj.getY();
    target_pose.position.z = tra_base_to_obj.getZ();
    
    move_group.setPoseTarget(target_pose);
    ROS_INFO("Plan");
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Finish plan");    
ROS_INFO("Is success: %B", success );
    obj_id++;
  }
  static tf::TransformBroadcaster br;
  /*while (ros::ok()){
	
      ROS_INFO("Publish tf");
      tf::Transform tf_test = transform * obj_tf[0];
	br.sendTransform(tf::StampedTransform(tf_test, ros::Time::now(),"car_base", "object0"));
	ros::Rate loop_rate(10);
        loop_rate.sleep();
  }
  */
  if (success){
  
  move_group.execute(my_plan);
  ros::Duration(2).sleep();
  }

  joint_group_positions[0] = 0;  // radians
  joint_group_positions[1] = -2;
  joint_group_positions[2] = 2.1;
  joint_group_positions[3] = 1.18;
  move_group.setJointValueTarget(joint_group_positions);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success){
  	move_group.execute(my_plan);
 	 ros::Duration(2).sleep();
  }
}



int main(int argc, char **argv){
  ros::init(argc, argv, "pose_est_and_pick");
  Pose_est_and_pick_server pose_est_and_pick_server;
  ros::AsyncSpinner spinner(1);
  ROS_INFO("pose_est_and_pick_server is ready");
  spinner.start();
  ROS_INFO("pose_est_and_pick_server is ready");
  ros::spin();
}
