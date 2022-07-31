


/*
 * depth_to_scan.cpp
 *
 *  Created on: Jun 12, 2022
 *      Author: yakir huri
 *
 *
 * Cogniteam LTD CONFIDENTIAL
 *
 * Unpublished Copyright (c) 2016-2017 Cogniteam,        All Rights Reserved.
 *
 * NOTICE:  All information contained  herein  is,  and  remains the property
 * of Cogniteam.   The   intellectual   and   technical   concepts  contained
 * herein are proprietary to Cogniteam and may  be  covered  by  Israeli  and
 * Foreign Patents, patents in process,  and  are  protected  by trade secret
 * or copyright law. Dissemination of  this  information  or  reproduction of
 * this material is strictly forbidden unless  prior  written  permission  is
 * obtained  from  Cogniteam.  Access  to  the  source  code contained herein
 * is hereby   forbidden   to   anyone  except  current  Cogniteam employees,
 * managers   or   contractors   who   have   executed   Confidentiality  and
 * Non-disclosure    agreements    explicitly    covering     such     access
 *
 * The copyright notice  above  does  not  evidence  any  actual  or intended
 * publication  or  disclosure    of    this  source  code,   which  includes
 * information that is confidential  and/or  proprietary,  and  is  a   trade
 * secret, of   Cogniteam.    ANY REPRODUCTION,  MODIFICATION,  DISTRIBUTION,
 * PUBLIC   PERFORMANCE,  OR  PUBLIC  DISPLAY  OF  OR  THROUGH USE   OF  THIS
 * SOURCE  CODE   WITHOUT   THE  EXPRESS  WRITTEN  CONSENT  OF  Cogniteam  IS
 * STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND INTERNATIONAL
 * TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE  CODE  AND/OR RELATED
 * INFORMATION DOES  NOT CONVEY OR IMPLY ANY RIGHTS  TO  REPRODUCE,  DISCLOSE
 * OR  DISTRIBUTE ITS CONTENTS, OR TO  MANUFACTURE,  USE,  OR  SELL  ANYTHING
 * THAT      IT     MAY     DESCRIBE,     IN     WHOLE     OR     IN     PART
 *
 */
#include <iostream>
#include <limits>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>

#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>


#include <image_geometry/stereo_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <angles/angles.h>
#include <opencv2/opencv.hpp>

#include <map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>


using namespace std;
using namespace cv;
using namespace pcl;


typedef pair<cv::Point3d, float> pair_3d_point_with_distance;

class Depth2Scan{


public:

    Depth2Scan(){

        ros::NodeHandle nodePrivate("~"); 
         //rosparam
        nodePrivate.param("max_distance_meter", maxDistMeters_, 2.0);
        nodePrivate.param("min_distance_meter", minDistMeters_, 0.1);        
        nodePrivate.param("min_h", minHeight_, -0.5);  
        nodePrivate.param("max_h", maxHeight_, 1.0);

        nodePrivate.param("min_deg_angle", minDegAngle_, -90.0);
        nodePrivate.param("max_deg_angle", maxDegAngle_, 90.0);

        nodePrivate.param("base_frame", base_frame_, string("base_link")); 
   


        depth_image_sub = node_.subscribe("/camera/aligned_depth_to_color/image_raw", 1,
             &Depth2Scan::depthCallback, this);

        info_sub = node_.subscribe( "/camera/aligned_depth_to_color/camera_info", 1,
             &Depth2Scan::cameraInfoCallback, this);

        scan_pub_ = node_.advertise<sensor_msgs::LaserScan>("/scan_from_shallow_cloud", 1);

        // pubPclLaser_ = node_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >
		//  ("/shallow_cloud", 1, true); 
    }

    ~Depth2Scan(){}

private:

    

    geometry_msgs::PointStamped transformToByFrames(
        Point3d objectPoint3d, string base_Frame, string child_Frame)  {


        geometry_msgs::PointStamped pointStampedIn;
        geometry_msgs::PointStamped pointStampedOut;

        pointStampedIn.header.frame_id = child_Frame;
        pointStampedIn.header.stamp = ros::Time(0);
        pointStampedIn.point.x = objectPoint3d.x;
        pointStampedIn.point.y = objectPoint3d.y;
        pointStampedIn.point.z = objectPoint3d.z;

        if( !initTransform_){

            try{
                

                if  (tfListener_.waitForTransform(base_Frame, child_Frame, 
                    ros::Time(0), ros::Duration(0.005))){
                        
                    initTransform_ = true;

                    tfListener_.lookupTransform(base_Frame, child_Frame,  
                                        ros::Time(0), transform_);

                    tfListener_.transformPoint(base_Frame, pointStampedIn, pointStampedOut);

                    return pointStampedOut;
                    
                }else {

                   //cerr<<"Failed to find transform between "<<base_Frame<<" and "<<child_Frame<<endl;
                   return pointStampedOut;
                }

            }   
                catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
        }
        else {


            tfListener_.transformPoint(base_Frame, pointStampedIn, pointStampedOut);


            return pointStampedOut;
        }  

      
    }



    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &cam_info) {

        if (cameraInfoInited_ == false)
        {
            if (pinholeCameraModel_.fromCameraInfo(*cam_info))
            {
                cameraInfoInited_ = true;

                cx_ = pinholeCameraModel_.cx();
                cy_ = pinholeCameraModel_.cy();

                fx_ = pinholeCameraModel_.fx();
                fy_ = pinholeCameraModel_.fy();

            }
        }
    }

    void depthCallback(const sensor_msgs::ImageConstPtr &image) {

        if (cameraInfoInited_) {    


            if( !initFisrtMsg) {

                initFisrtMsg = true;
                start_scan_time_ = ros::Time::now();

            }

            cerr<<"1111111111111111111 "<<endl;       
          
            std::map<int, pair_3d_point_with_distance > deg_dist_map;

            string depth_camera_link_ = image->header.frame_id;


            cv::Mat depthColor;
            cv_bridge::CvImagePtr cvBridgePtr =
                cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);

            cv::Mat depth = cvBridgePtr->image;

          

            float minAngleRad = 9999;
            float maxAngleRad = -9999;


            ////////////////////////////
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            cloud.header.frame_id = base_frame_;

          
         
            ////////////////////

            // loop over dpeth img
            for(int j =0; j <depth.rows ; j++){

                float* pixel = depth.ptr<float>(j);  
                
                for(int i = 0; i< depth.cols ; i++){

                    float distValMM = pixel[i]; 
                    float distM = distValMM / 1000.0;


                    if (!std::isfinite(pixel[i]) || distValMM == 0.0) {
                        continue;
                    }
                    
                    // point to far or too close
                    if ( distM > maxDistMeters_
                         || distM < minDistMeters_ ){
                        continue;
                    }
                  
                    const cv::Point2d uv_rect(i,j);	
                    cv::Point3d threedp = pinholeCameraModel_.projectPixelTo3dRay(uv_rect);  
                    threedp.x *=  distM;
                    threedp.y *=  distM;
                    threedp.z *=  distM;
                    auto pose = transformToByFrames(threedp, base_frame_ , source_frame );

                    // if too high or too low
                    if(  pose.point.z < minHeight_ || pose.point.z > maxHeight_) {
                        continue;
                    }         
                                      

                    float angleDeg  = (angles::to_degrees(atan2(pose.point.y, pose.point.x)));
                    // cerr<<" angleDeg "<<angleDeg<<endl;

                    // if out of FOV
                    if( angleDeg < minDegAngle_ || angleDeg > maxDegAngle_){
                        
                        continue;
                    }

                    // get the min deg
                    if ( angles::from_degrees(angleDeg) < minAngleRad   ){
                        minAngleRad = angles::from_degrees(angleDeg);
                    }

                    // get the max deg
                    if ( angles::from_degrees(angleDeg) > maxAngleRad   ){
                        maxAngleRad = angles::from_degrees(angleDeg);
                    }

                    if( deg_dist_map.find(angleDeg) != deg_dist_map.end()){
                        //angle exsist
                        
                        // update te vaule (distnace)
                        if(  distM < deg_dist_map[angleDeg].second ) {

                            // cerr<<" map angle "<<angleDeg<<" old distance "<<deg_dist_map.at(angleDeg)<<endl;
                            // cerr<<"new distnace "<<distM<<endl;
                            deg_dist_map.at(angleDeg) = std::make_pair(cv::Point3d(pose.point.x, pose.point.y, pose.point.z), distM);
                            
                        }

                    } else {
                        //create new pair
                        deg_dist_map[angleDeg] = std::make_pair( cv::Point3d(pose.point.x, pose.point.y, pose.point.z), distM);

                    }                  

                }
            }           

            cerr<<"deg_dist_map "<<deg_dist_map.size()<<endl;

            // //  publish cloud
            publishPointCloud(deg_dist_map);
           

        }
            
    }

    void publishPointCloud(const std::map<int, pair_3d_point_with_distance >& deg_dist_map)  {

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.header.frame_id = base_frame_;

        //publish point cloud
            
        for (auto it = deg_dist_map.begin(); it != deg_dist_map.end(); ++it)
        {
            float closestDistM = it->second.second;
            float radAngle = angles::from_degrees(it->first);


            pcl::PointXYZRGB pRGB;                


            pRGB.x =   it->second.first.x;
            pRGB.y =   it->second.first.y;        
            pRGB.z =    0.0;   

           

        //    cerr<<" closestDistM "<<closestDistM<<" angle "<<it->first<<" x "<<pRGB.x<<" y "<<pRGB.y<<endl;

            uint8_t r = 0;
            uint8_t g = 255;
            uint8_t b = 0;

            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            pRGB.rgb = *reinterpret_cast<float*>(&rgb);

            cloud.points.push_back(pRGB); 

        }

      
       
        publishScan(cloud); 

        // pubPclLaser_.publish(cloud);
        
        

    }

    void publishScan(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_msg)  {


        auto end_scan_time = ros::Time::now();
        auto scan_duration = (end_scan_time - start_scan_time_).toSec();
        start_scan_time_ = end_scan_time;

            // build laserscan output
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.frame_id = base_frame_;
        scan_msg.header.stamp = ros::Time::now();
        
        scan_msg.angle_min = -M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = M_PI / 180.0;
        scan_msg.time_increment = 0.0; //scan_duration;
        scan_msg.scan_time = scan_duration;
        scan_msg.range_min = minDistMeters_;
        scan_msg.range_max = maxDistMeters_;

        // determine amount of rays to create
        uint32_t ranges_size = std::ceil(
            (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);

        // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
        scan_msg.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
       

        for( int i = 0; i < cloud_msg.size(); i++)
        {   
            pcl::PointXYZRGB pRGB = cloud_msg[i];  
            
            if (std::isnan(pRGB.x) || std::isnan(pRGB.y) || std::isnan(pRGB.z)) {
            
                continue;
            }
            

            double range = hypot(pRGB.x, pRGB.y);
            

            double angle = atan2(pRGB.y, pRGB.x);
            

            // overwrite range at laserscan ray if new range is smaller
            int index = (angle - scan_msg.angle_min) / scan_msg.angle_increment;
            if (range < scan_msg.ranges[index]) {
                scan_msg.ranges[index] = range;
            }
        }

        scan_pub_.publish(scan_msg);


    }
    
   

    bool extractDepthFromBboxObject(const cv::Point2d uv_rect, float d,
           geometry_msgs::PointStamped& pose) {
       

        cv::Point3d p = pinholeCameraModel_.projectPixelTo3dRay(uv_rect);  
        p.x *=  d;
        p.y *=  d;
        p.z *=  d;

        pose = transformToByFrames(p, base_frame_ , source_frame );
           

        return true;
    }


private:

    ros::Publisher scan_pub_;

    ros::Publisher pubPclLaser_;

    ros::Subscriber depth_image_sub;

    ros::Subscriber  info_sub;

    ros::NodeHandle node_;

    bool cameraInfoInited_ = false;

    tf::TransformListener tfListener_;


    image_geometry::PinholeCameraModel pinholeCameraModel_;


    string base_frame_= "base_link";

    string source_frame = "camera_depth_optical_frame";


    double maxDistMeters_ = 2.0;
    double minDistMeters_ = 0.0;

    double minHeight_ = -0.5;
    double maxHeight_ = 2.5;

    int depth_angle = 180;

    double minDegAngle_ = -90.0;
    double maxDegAngle_ = 90.0;

    float cx_ = 0;
    float cy_ = 0;
    float fx_ = 0;
    float fy_ = 0;

    bool initTransform_ = false;
    bool initFisrtMsg = false;
    tf::StampedTransform transform_;

    ros::Time start_scan_time_;



};


int main(int argc, char **argv){


    ros::init(argc, argv,"depth_to_scan");
    Depth2Scan depth2Scan;

    ros::spin();
    return 0;
}
