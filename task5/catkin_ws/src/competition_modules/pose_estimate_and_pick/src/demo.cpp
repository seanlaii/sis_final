#include "pose_estimation1.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "pose_estimation");
    pose_estimation pose_estimation;
    ros::spin();
    return 0;
}