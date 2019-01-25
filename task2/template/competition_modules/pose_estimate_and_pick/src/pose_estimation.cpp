#include "pose_estimation1.h"

pose_estimation::pose_estimation(){
    
    ////////////////////////////////////////////////
    /*for(int i=1;i<4;i++){
      object_clouds[i].reset(new PointCloud<PointXYZRGB>());
    }*/ 
    cs = 0;
    count = 0;
    total = 0; 
    scene_cloud.reset(new PointCloud<PointXYZRGB>()); 
    original_cloud1.reset(new PointCloud<PointXYZRGB>());
    original_cloud2.reset(new PointCloud<PointXYZRGB>());
    original_cloud3.reset(new PointCloud<PointXYZRGB>()); 
    downsampled_cloud.reset(new PointCloud<PointXYZRGB>()); 
    denoised_cloud.reset(new PointCloud<PointXYZRGB>());
    
    /////////////////Ros node initialization////////
    ros::Time::init();
    ros::NodeHandle nh;
    /////////////////Declare Ros publisher and subscriber////////
    original_object1_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/original_object_class1", 1);
    original_object2_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/original_object_class2", 1);
    original_object3_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/original_object_class3", 1);
    model_cloud1_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/model_cloud_class1", 1);
    model_cloud3_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/model_cloud_class3", 1);
    registered_cloud1_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/icp_registered_cloud1", 1);
    registered_cloud3_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/icp_registered_cloud3", 1);
    initial_guess_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/initial_guess", 1);
    object_publisher = nh.advertise<sensor_msgs::PointCloud2> ("/camera/object", 1);

  ////////////////////Server///////////////////
    service = nh.advertiseService("pose_estimation", &pose_estimation::serviceCb, this);
}

void pose_estimation::update_points(const sensor_msgs::PointCloud2 cloud){
    CAMERA_FRAME.frame_id = "camera_rgb_optical_frame";
	  pcl::fromROSMsg (cloud, *scene_cloud);
    //std::cout << "height " << scene_cloud->height << "width " << scene_cloud->width;
  	return;
}
bool pose_estimation::serviceCb(pose_estimate_and_pick::pose_estimation::Request &req, pose_estimate_and_pick::pose_estimation::Response &res){
  ros::ServiceClient client = nh.serviceClient<object_detection::task1out>("prediction");
  object_detection::task1out srv;
  static tf::TransformBroadcaster br;
  tf::TransformListener listener;
  //std::string res.object_list[20][2];
  if(client.call(srv)){ 
    update_points(srv.response.pc);
    
    cv_ptr = cv_bridge::toCvCopy(srv.response.mask, sensor_msgs::image_encodings::TYPE_8UC1); 
	  object_publisher.publish(scene_cloud);
    //point_cloud_preprocessing(scene_cloud);
    //ROS_INFO("Size of point cloud after preprocessing: %d\n",scene_cloud->points.size());
    int j = 0;
    object_cloud_filtering(cv_ptr);
    load_models();
    tf::StampedTransform transform;
    try{
          // listener.waitForTransform("/base_link", "/camera_rgb_optical_frame",
          //                     now, ros::Duration(3.0));
          listener.lookupTransform("/base_link", "/camera_rgb_optical_frame",  
                             ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
           ROS_ERROR("%s",ex.what());
           ros::Duration(1.0).sleep();
    }
    for(int i=0;i<3;i++){
      printf("Class %d:\n", i+1);
      printf("Original object cloud size: %d\n", object_clouds[i]->points.size());
      point_cloud_clustering(object_clouds[i]);
      for(j=0;j < count;j++){
        //ROS_INFO("P1 ");
        Eigen::Matrix4f tf1 = initial_guess(clusters[j], modelClouds[i]);
        //ROS_INFO("P2 ");
        pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_source_trans_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
        pcl::PointCloud<PointXYZRGB>::Ptr ini_guess_tf_cloud ( new PointCloud<PointXYZRGB> );
        pcl::transformPointCloud (*clusters[j], *ini_guess_tf_cloud, tf1);
        std::cout << "\nTF1: \n" << tf1;
        ini_guess_tf_cloud->header.frame_id = "camera_rgb_optical_frame";
        initial_guess_publisher.publish(ini_guess_tf_cloud);
        //ROS_INFO("P3 "); 
        Eigen::Matrix4f tf2 = point_2_plane_icp( ini_guess_tf_cloud, modelClouds[i], cloud_source_trans_normals);
        //ROS_INFO("P4 ");
        std::string obj_str = "object " + std::to_string(cs);
        res.obj_list.push_back(obj_str);
        std::string tag = std::to_string(i);
        res.tagID.push_back(tag);
        Eigen::Matrix4f final_tf = tf1 * tf2;
        std::cout << final_tf << endl;

        Eigen::Vector4f src_centroid;
        pcl::compute3DCentroid (*clusters[j], src_centroid);
        //final_tf(0,3) += src_centroid[0];
        //final_tf(1,3) += src_centroid[1];
        //final_tf(2,3) += src_centroid[2];

        //////Matrix4f to tf::transform//////
        tf::Vector3 tf_tran = tf::Vector3(src_centroid[0], src_centroid[1], src_centroid[2]);
        tf::Quaternion quat;
        tf::Matrix3x3 tf_rot = tf::Matrix3x3(final_tf(0,0), final_tf(0,1), final_tf(0,2),
                                             final_tf(1,0), final_tf(1,1), final_tf(1,2),
                                             final_tf(2,0), final_tf(2,1), final_tf(2,2));
        tf_rot.getRotation(quat);
        quat.normalize();

        // geometry_msgs::Pose pose;
        // quaternionTFToMsg(quat, pose.orientation); 
        // pose.position.x = src_centroid[0];
        // pose.position.y = src_centroid[1];
        // pose.position.z = src_centroid[2];
        // res.pose.push_back(pose);

        tf_rot.setRotation(quat);
        cout << "Tran" << endl;
        cout << final_tf(0,3) << final_tf(1,3) << final_tf(2,3);
        cout << "Rot" << endl;
        cout << "\n" << final_tf(0,0) << " " << final_tf(0,1) << " " << final_tf(0,2);
        cout << "\n" << final_tf(1,0) << " " << final_tf(1,1) << " " << final_tf(1,2);
        cout << "\n" << final_tf(2,0) << " " << final_tf(2,1) << " " << final_tf(2,2);
        tf::Transform tf = tf::Transform(tf_rot, tf_tran);

        tf::Transform output_tf = transform * tf;
        //cout << "\nTF_Final: " << output_tf;
        // tf::Quaternion quat1;
        // output_tf.getRotation()
        // tf::Matrix3x3 tf_rot1 = tf::Matrix3x3(output_tf(0,0), output_tf(0,1), output_tf(0,2),
        //                                      output_tf(1,0), output_tf(1,1), output_tf(1,2),
        //                                      output_tf(2,0), output_tf(2,1), output_tf(2,2));
        // tf_rot1.getRotation(quat1);
        // quat1.normalize();


        geometry_msgs::Pose pose;
        quaternionTFToMsg(output_tf.getRotation(), pose.orientation); 
        tf::Vector3 pose_trans =  output_tf.getOrigin();
        pose.position.x = pose_trans.getX();
        pose.position.y = pose_trans.getY();
        pose.position.z = pose_trans.getZ();
        res.pose.push_back(pose);

        /////////////////////////////////////
        br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), CAMERA_FRAME.frame_id, obj_str));
        br.sendTransform(tf::StampedTransform(output_tf, ros::Time::now(), "/base_link", "AndyisHandsome"));
        //Eigen::Matrix4f final_tf = tf1 * tf2;
        //point_cloud_pose_estimation(clusters[j], cs);
        ////Response////

        /*pose_estimate_and_pick::Object_Tag tagid;
        std::string obj_str = "object " + std::to_string(cs);
        std::string tag = std::to_string(i);
        tagid.tagID.push_back(obj_str);
        tagid.tagID.push_back(tag);

        res.object_list.push_back(tagid);*/
        //res.object_list[cs].tagID.push_back(tag); 
        //std::cout << "Object name: "<< res.object_list[cs].tagID[0] << " TagID: " << res.object_list[cs].tagID[1] << std::endl;
        //std::cout << tf2 ;
        cs ++;
        ROS_INFO("Success");
        if(i==0){
          cloud_source_trans_normals->header.frame_id = "camera_rgb_optical_frame";
          registered_cloud1_publisher.publish(cloud_source_trans_normals);
          //ROS_INFO("P5 ");
        }
        else if(i==2){
          cloud_source_trans_normals->header.frame_id = "camera_rgb_optical_frame";
          registered_cloud3_publisher.publish(cloud_source_trans_normals);
          
          //ROS_INFO("P6 ");
        }
      }
      count = 0;
      clusters.clear();
    }
    cs = 0;
    std::cout << total << std::endl;
    total = 0;
    //res.obj_list.clear();
    //initial_guess_publisher.publish(ini_guess_tf_cloud);
    //std::cout << "Object name: "<< res.object_list[cs].tagID[0] << " TagID: " << res.object_list[cs].tagID[1] << std::endl;
  }
  //registered_clouds.clear();
  object_clouds.clear();
  object_list.clear();
  return true;
}
void pose_estimation::point_cloud_preprocessing(PointCloud<PointXYZRGB>::Ptr noised_cloud){
  // std::vector<int> indices;
  // pcl::removeNaNFromPointCloud(*noised_cloud, *noised_cloud, indices);
  
  //////////////Pointcloud downsampling////////////////////
  pcl::VoxelGrid<PointXYZRGB> sor;
  sor.setInputCloud (noised_cloud);
  sor.setLeafSize (0.002f, 0.002f, 0.002f);
  sor.filter (*noised_cloud);  
  copyPointCloud(*noised_cloud, *downsampled_cloud);

  //////////////Pointcloud Denoise////////////////////
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
  if (noised_cloud->points.size()>100){
    sor2.setInputCloud (noised_cloud);
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (0.5);
    sor2.filter (*noised_cloud);
  }

  std::vector<int> indices2;
  pcl::removeNaNFromPointCloud(*noised_cloud, *noised_cloud, indices2);
  copyPointCloud(*noised_cloud, *denoised_cloud);
  return;
}
void pose_estimation::object_cloud_filtering(cv_bridge::CvImagePtr mask){
  int c = 0;
  int i = 0;
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  std::vector<int> indices;
  std::cout << "Encoding: "<< mask->encoding << endl;
  for( i=1; i<4; i++){
    printf ("Start filtering\n");
    copyPointCloud(*scene_cloud, *cloud);
    for (int row=0;row<480;row++){
      for(int column=0;column<640;column++){
        
        if  (mask->image.at<uchar>(row,column) != i){
          //printf ("%d\n", mask->image.at<uchar>(row,column));
          cloud->points[c].x= std::numeric_limits<float>::quiet_NaN();
          cloud->points[c].y= std::numeric_limits<float>::quiet_NaN();
          cloud->points[c].z= std::numeric_limits<float>::quiet_NaN();
          // cloud->at(column, row).x= std::numeric_limits<float>::quiet_NaN();
          // cloud->at(column, row).y= std::numeric_limits<float>::quiet_NaN();
          // cloud->at(column, row).z= std::numeric_limits<float>::quiet_NaN();
        }
        c++;
      }
    }
    printf ("Finish filtering\n");
    PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);
    pcl::removeNaNFromPointCloud(*cloud, *filtered_cloud, indices);
    object_clouds.push_back(filtered_cloud);
    if(i==1){
      //printf ("P1\n");
      // copyPointCloud(*cloud, *original_cloud1);
      original_object1_publisher.publish(filtered_cloud);
      //printf ("C1_finish\n");
    }
    else if(i==2){
      // copyPointCloud(*cloud, *original_cloud2);
      original_object2_publisher.publish(filtered_cloud);
      //printf ("C2_finish\n");
    }
    else if(i==3){
      // copyPointCloud(*cloud, *original_cloud3);
      original_object3_publisher.publish(filtered_cloud);
      //printf ("C3_finish\n");
    }
    c = 0;  
  } 
  return;
}

void pose_estimation::point_cloud_clustering(PointCloud<PointXYZRGB>::Ptr unclustered_cloud){
  
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  std::vector<int> indices;
  // pcl::removeNaNFromPointCloud(*unclustered_cloud, *unclustered_cloud, indices);
  tree->setInputCloud (unclustered_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.04); // 2cm
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (unclustered_cloud);
  ec.extract (cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      cloud_cluster->points.push_back (unclustered_cloud->points[*pit]); 
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
    //copyPointCloud(*cloud_cluster, *clusters[count]);
    clusters.push_back(cloud_cluster);
    count = count + 1;
  }
  total = total + count;
  std::cout << "Clusters size: " << clusters.size() << std::endl;
  return;
}

void pose_estimation::addNormal(PointCloud<PointXYZRGB>::Ptr cloud, PointCloud<PointXYZRGBNormal>::Ptr cloud_with_normals)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );
  pcl::search::KdTree<PointXYZRGB>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  searchTree->setInputCloud ( cloud );
  pcl::NormalEstimation<PointXYZRGB, Normal> normalEstimator;
  normalEstimator.setInputCloud ( cloud );
  normalEstimator.setSearchMethod ( searchTree );
  normalEstimator.setKSearch ( 15 );
  normalEstimator.compute ( *normals );
  pcl::concatenateFields( *cloud, *normals, *cloud_with_normals );
}


Eigen::Matrix4f pose_estimation::initial_guess(PointCloud<PointXYZRGB>::Ptr cloud_src, PointCloud<PointXYZRGB>::Ptr cloud_target)
{
  Eigen::Vector4f src_centroid, target_centroid;
  pcl::compute3DCentroid (*cloud_src, src_centroid);
  pcl::compute3DCentroid (*cloud_target, target_centroid);
  Eigen::Matrix4f tf_tran = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f tf_rot = Eigen::Matrix4f::Identity();

  Eigen::Matrix3f covariance;
  std::cout << "Source Centroid: " <<src_centroid << endl;
  std::cout << "Target Centroid: " << target_centroid << endl;
  pcl::computeCovarianceMatrixNormalized(*cloud_src, src_centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

  Eigen::Matrix3f covariance2;
  //ROS_INFO("Success_ in2");
  pcl::computeCovarianceMatrixNormalized(*cloud_target, target_centroid, covariance2);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver2(covariance2, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA2 = eigen_solver2.eigenvectors();

  // Eigen::Quaternion<float> rot_q = Eigen::Quaternion<float>::FromTwoVectors(eigenVectorsPCA.row(0),eigenVectorsPCA2.row(0));
  Eigen::Matrix3f R ;
  R = eigenVectorsPCA2 * eigenVectorsPCA.inverse();
  for (int i = 0;i<3;i++)
      for (int j = 0;j<3;j++)
          tf_rot(i,j) = R(i,j);


  tf_tran(0,3) = target_centroid[0] - src_centroid[0];
  tf_tran(1,3) = target_centroid[1] - src_centroid[1];
  tf_tran(2,3) = target_centroid[2] - src_centroid[2];
  std::cout << tf_tran(0,3) << tf_tran(1,3) << tf_tran(2,3)<< endl;
  Eigen::Matrix4f tf = tf_rot * tf_tran ;
  //ROS_INFO("Success_ in3");
  return tf;
}

Eigen::Matrix4f pose_estimation::point_2_plane_icp (PointCloud<PointXYZRGB>::Ptr sourceCloud, PointCloud<PointXYZRGB>::Ptr targetCloud, PointCloud<PointXYZRGBNormal>::Ptr cloud_source_trans_normals ){
  	pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_source_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
  	pcl::PointCloud<PointXYZRGBNormal>::Ptr cloud_target_normals ( new pcl::PointCloud<PointXYZRGBNormal> );
  	pcl::PointCloud<PointXYZRGB>::Ptr translated_sourceCloud(new pcl::PointCloud<PointXYZRGB>);
  	Eigen::Matrix4f transform_translation = Eigen::Matrix4f::Identity();
  	Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*sourceCloud, centroid);
    transform_translation(0,3) -= centroid[0];
    transform_translation(1,3) -= centroid[1];
    transform_translation(2,3) -= centroid[2];
    pcl::transformPointCloud (*sourceCloud, *translated_sourceCloud, transform_translation);
    
  	addNormal( translated_sourceCloud, cloud_source_normals );
  	addNormal( targetCloud, cloud_target_normals );
    /*addNormal( sourceCloud, cloud_source_normals );
    addNormal( targetCloud, cloud_target_normals );*/

  	// addNormal( cloud_source_trans, cloud_source_trans_normals );
  	pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp ( new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> () );
  	icp->setMaximumIterations ( 1000 );
  	//icp.setMaxCorrespondenceDistance(1);  
    icp->setTransformationEpsilon(1e-10);  
    icp->setEuclideanFitnessEpsilon(0.01);  
  	icp->setInputSource ( cloud_source_normals ); // not cloud_source, but cloud_source_trans!
  	icp->setInputTarget ( cloud_target_normals );
  
   	// registration
  	icp->align ( *cloud_source_trans_normals ); // use cloud with normals for ICP
    
  	if ( icp->hasConverged() ){
   		std::cout << "icp score: " << icp->getFitnessScore() << std::endl;
      // std::cout << icp->getFinalTransformation() << std::endl;
   	}
   	else
   		std::cout << "Not converged." << std::endl;
    //The Transformation from model to object cloud


    ///////////Generate the transform matrix from model to object scene
    Eigen::Matrix4f inverse_transformation = icp->getFinalTransformation();
    return inverse_transformation;
    //return object_transform_matrix; 
}
void pose_estimation::load_models(){
    //////////////////Define model path/////////////
    string object_model_path("/root/sis_mini_competition_2018/");
    //string bin_model_path("/home/nvidia/ctsphub-workshop-2018/04-perception/03-case_study/arc2016_TX2/catkin_ws/src/pose_estimation/src/model/bins/");
    //////////////////Create object list////////////
    //object_list.push_back("crayola_24_ct");
    //object_list.push_back("kleenex_tissue_box");
    object_list.push_back("doublemint");    
    object_list.push_back("kinder");
    object_list.push_back("kusan");
    ROS_INFO("Success_load: %d", object_list.size());
    /////////////////Create object Pointcloud list//
    for (int i = 0;i<3;i++){
      PointCloud<PointXYZRGBA>::Ptr sourceCloud(new PointCloud<PointXYZRGBA>);
      string model_path = object_model_path + object_list[i] + ".ply";
      printf("%s", model_path.c_str());
      pcl::PolygonMesh mesh;
      pcl::io::loadPLYFile(model_path, mesh);
      pcl::fromPCLPointCloud2( mesh.cloud, *sourceCloud );
      //io::loadPLYFile<PointXYZRGBA>(model_path, *sourceCloud);
      sourceCloud->header.frame_id = "camera_rgb_optical_frame";
      pcl::PointCloud<PointXYZRGB>::Ptr source_rgb(new PointCloud<PointXYZRGB>);
      pcl::copyPointCloud(*sourceCloud, *source_rgb);
      pcl::VoxelGrid<PointXYZRGB> sor;
      sor.setInputCloud (source_rgb);
      sor.setLeafSize (0.005f, 0.005f, 0.005f);
      sor.filter (*source_rgb);  
      printf("%s Model cloud size: %d\n",object_list[i].c_str(),sourceCloud->points.size());
      modelClouds.push_back(source_rgb);
    }
    model_cloud1_publisher.publish(modelClouds[0]);
    model_cloud3_publisher.publish(modelClouds[2]);
    printf("-----------Finish load model clouds----------\n");
}

