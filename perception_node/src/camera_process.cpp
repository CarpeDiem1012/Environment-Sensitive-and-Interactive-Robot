#include <cstring>
#include <iostream>
#include <string.h>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <list>
#include <numeric>
#include <random>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <unistd.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/aruco.hpp>


using namespace std;
using namespace cv;

using std::cout;
using std::endl;

// Class for image processing
class ImageProcess { 
    public:
        // Initialize node handler and image transport
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Subscriber image_sub_test_;

        ros::Subscriber image_sub_state_;

        image_transport::Publisher image_map_pub_;
        image_transport::Publisher image_obs_pub_;
        image_transport::Publisher image_track_pub_;
        image_transport::Publisher image_test_;

        ros::Publisher move_pub_;
        ros::Publisher husky_loc_pub_;


    public:
        // Create instance of ImageProcess
        ImageProcess():

        // Crreate subrsiber and publishers
        it_(nh_) {
            // ******SUBSCRIBERS*******
            // Subsrice to raw camera image
            image_sub_ = it_.subscribe("/camera_1/image_raw", 1, &ImageProcess::imageCb, this);

            // // Subsrice to state message
            image_sub_state_ = nh_.subscribe("/human_robot_interaction_node/FSM", 1, &ImageProcess::FSM, this);

            // // Publish to cmd_vel
            // move_pub_ = nh_.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);


            // *******PUBLISHERS*******
            // Publish processed images to each topic
            image_map_pub_ = it_.advertise("/image_process/occupancy_map", 1);
            image_obs_pub_ = it_.advertise("/image_process/moving_obstacles", 1);
            image_track_pub_ = it_.advertise("/image_process/husky_tracking", 1);
            image_test_ = it_.advertise("/image_process/test_maps", 1);

            // Publish husky coordinates and scaling factor
            husky_loc_pub_ = nh_.advertise<geometry_msgs::Point>("/image_process/husky_location", 1);

        }

        cv::Mat img_bgr, img_gray, img_track, img_track2, img_blur, diff, thresh, test_img, test_gray;
        geometry_msgs::Point husky_location;
        
        int i=0;
        int state=0;

        float husky_x;
        float husky_y;
        int th = 30;

        int start_x = 0;
        int end_x = 1280;
        int start_y = 0;
        int end_y = 720;

        float m_per_pix = 1;

        ~ImageProcess(){
        }   

        // Receive states from FSM
        void FSM(const std_msgs::String::ConstPtr& msg){
            //ROS_INFO_STREAM(msg->data);
            if(msg->data == "startup_state"){
                state = 1;
                ROS_INFO_STREAM("Startup_state: Perceiving World");    
            }
        }

        //***********IMAGE PROCESS****************

        // Callback function that's called after each new input image is received
        void imageCb(const sensor_msgs::ImageConstPtr& msg){
            // Use CV bridge format to import image

            cv_bridge::CvImagePtr cv_ptr;
            
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            }   catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            // Make a copy
            cv_ptr->image.copyTo(img_bgr);
            
            // Convert image to greyscale
            cvtColor(cv_ptr->image, img_gray, COLOR_BGR2GRAY);

            // image_map_pub_.publish(cv_ptr->toImageMsg());

            // Copy grey image to use for movement tracking
            img_gray.copyTo(img_track);


            // If state is received from FSM start processing
            if (state==1){              
                this->TrackMovement(img_track, cv_ptr);
                this->DetectMarker(img_bgr, msg);

                // this->imageTest(msg);
                this->ImageSegment(img_gray, msg);     
   
            }
          
            img_gray.copyTo(img_track2);
        }


        void imageTest(const sensor_msgs::ImageConstPtr& msg){
            char tmp[256];
            getcwd(tmp, 256);
            std::string cwd = string(tmp);
            
            std::string file_path = cwd + "/src/cor_mdp_husky/perception_node/src/test2.png";
            test_img = imread(file_path, IMREAD_COLOR);

            cvtColor(test_img, test_gray, COLOR_BGR2GRAY);

            
            sensor_msgs::Image::Ptr test_out = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, test_img).toImageMsg();

            image_test_.publish(test_out);
        }

        // Look for aruco marker and draw detection on map
        void DetectMarker(cv::Mat frame, const sensor_msgs::ImageConstPtr& msg){
            cv::Mat frame_cropped;
            cv::Mat img_cropped;
            frame_cropped = frame(Range(start_y, end_y), Range(start_x, end_x));
            img_cropped = img_bgr(Range(start_y, end_y), Range(start_x, end_x));

            Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
            Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f> > corners;
            cv::aruco::detectMarkers(frame_cropped, dictionary, corners, ids);
            cv::aruco::drawDetectedMarkers(img_cropped, corners, ids);

            if (i<10){
                // Calibrate camera
                m_per_pix = (corners[0][1].y - corners[0][0].y)/0.35;

            }
            
            // Determine Husky position
            husky_x = corners[0][0].x;
            husky_y = corners[0][0].y;

            husky_location.x = husky_x / m_per_pix;
            husky_location.y = husky_y / m_per_pix;
            husky_location.z = m_per_pix;
        

            sensor_msgs::Image::Ptr aruco_img = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, img_cropped).toImageMsg();

            image_track_pub_.publish(aruco_img);
            husky_loc_pub_.publish(husky_location);
        }


        // Segment raw image and create occupancy map
        void ImageSegment(cv::Mat img_gray, const sensor_msgs::ImageConstPtr& msg){
            // Blur for better edge detection
            cv::GaussianBlur(img_gray, img_blur, Size(3,3), 0);
            cv::Mat sobelx, sobelxy, occ_map;
            // cv::Sobel(img_blur, sobelx, CV_32F, 1, 0, 3);
            cv::Sobel(img_blur, sobelxy, CV_32F, 1, 1, 5);
            // cv::Sobel(img_blur, sobelxy, CV_32F, 1, 1, 5);

            
            cv::Mat edges;
            // cv::Canny(img_blur, sobelxy, 50, 100, 3, false);

            // Convert image back to Cv_8U
            cv::convertScaleAbs(sobelxy, occ_map);
      
            cv::Mat occ_map_cropped;
            
            // Create occupancy map and determine borders
            for (int i=0; i<occ_map.rows; i++){
                for (int j=0; j<occ_map.cols; j++){
                    float pixel = occ_map.at<uchar>(i, j);
                    float pixel_up1 = occ_map.at<uchar>(i, j+1);
                    float pixel_up3 = occ_map.at<uchar>(i, j+3);
                    float pixel_up5 = occ_map.at<uchar>(i, j+5);
                    float pixel_down1 = occ_map.at<uchar>(i, j-1);
                    float pixel_down3 = occ_map.at<uchar>(i, j-3);
                    float pixel_down5 = occ_map.at<uchar>(i, j-5);

                    if (pixel==0 && pixel_up1 > 0 && pixel_down1 && pixel_up3 > 0 && pixel_down3 > 0 && pixel_up5 > 0 && pixel_down5 > 0){
                        occ_map.at<uchar>(i, j) = 255;
                    }
                    if (pixel > 0){
                        occ_map.at<uchar>(i, j) = 255;
                        end_y = i-15;
                        if (i<occ_map.rows - 15){
                            end_x = j-15;            
                        }
                    }

                    if (start_x==0 && pixel!=0 && j<start_x){
                        start_x = j+15;
                    }

                    if (start_y==0 && pixel!=0){
                        start_y = i+15;
                    }

                }
            }
            
            // Crop map to remove outsides
            occ_map_cropped = occ_map(Range(start_y, end_y), Range(start_x, end_x));

            std::vector<int> obs_list_top(occ_map_cropped.cols);
            std::vector<int> obs_list_bot(occ_map_cropped.cols);

            std::iota(obs_list_top.begin(), obs_list_top.end(), 1);
            std::iota(obs_list_bot.begin(), obs_list_bot.end(), 1);

            // Fill borders of map (start from top)
            for (int i=0; i<occ_map_cropped.rows; i++){
                for (int j=0; j<occ_map_cropped.cols; j++){
                    float pixel = occ_map_cropped.at<uchar>(i, j);

                    if (pixel!=0 && std::find(obs_list_top.begin(), obs_list_top.end(), j) != obs_list_top.end()){
                        obs_list_top.erase(std::remove(obs_list_top.begin(), obs_list_top.end(), j), obs_list_top.end());
 
                    }
                    
                    if (pixel==0){
                        if (std::find(obs_list_top.begin(), obs_list_top.end(), j) != obs_list_top.end()){
                            occ_map_cropped.at<uchar>(i, j) = 255;
                        }
                    }
                    
                    // Color Husky grey
                    if (occ_map_cropped.at<uchar>(i, j)==255 && i<husky_y+th && i>husky_y-th && j<husky_x+th && j>husky_x-th && husky_x!=0 && husky_y!=0){

                        occ_map_cropped.at<uchar>(i, j) = 100;
                    }
                }
            }

            // Fill borders of map (start from top)
            for (int i=occ_map_cropped.rows; i>0; i--){
                for (int j=0; j<occ_map_cropped.cols; j++){
                    float pixel = occ_map_cropped.at<uchar>(i, j);

                    if (pixel!=0 && std::find(obs_list_bot.begin(), obs_list_bot.end(), j) != obs_list_bot.end()){
                        obs_list_bot.erase(std::remove(obs_list_bot.begin(), obs_list_bot.end(), j), obs_list_bot.end());
 
                    }
                    
                    if (pixel==0){
                        if (std::find(obs_list_bot.begin(), obs_list_bot.end(), j) != obs_list_bot.end()){
                            occ_map_cropped.at<uchar>(i, j) = 255;
                        }
                    }
                }
            }

        
            // Publish segmented image with 32FC1 encoding
            // sensor_msgs::Image::Ptr seg_img = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, sobelxy).toImageMsg();
            // sensor_msgs::Image::Ptr seg_img = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, occ_map).toImageMsg();
            // sensor_msgs::Image::Ptr seg_img = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, edges).toImageMsg();

            
            sensor_msgs::Image::Ptr seg_img = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, occ_map_cropped).toImageMsg();

            image_map_pub_.publish(seg_img);
        }


        void TrackMovement(cv::Mat img_in, cv_bridge::CvImagePtr cv_ptr){
            // Compute difference between first frame and current frame
            if(i!=0){
                absdiff(img_in, img_track2, diff);
                threshold(diff, thresh, 10, 255, THRESH_BINARY);
                morphologyEx(thresh, thresh, 2, cv::getStructuringElement(2, cv::Size(3, 3)));
                
                // Draw circle around moving object
                cv::Mat temp;
                cv::Rect objectBoundingRectangle = cv::Rect(0,0,0,0);
                thresh.copyTo(temp);
                vector<Vec4i> hierarchy;
                vector<vector<Point> > contours;
                findContours(temp, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

                // Draw circle on detected movements
                if(contours.size()>0){
                    vector<vector<Point> > largest_contour;
                    largest_contour.push_back(contours.at(contours.size()-1));
                    objectBoundingRectangle = cv::boundingRect(largest_contour.at(0));
                    int x = objectBoundingRectangle.x + objectBoundingRectangle.width/2;
                    int y = objectBoundingRectangle.y + objectBoundingRectangle.height/2;
                    cv::circle(cv_ptr->image,cv::Point(x,y), 35, cv::Scalar(0,255,0), 2);
                }
            }  
            // Publish image with tracking circle
            image_obs_pub_.publish(cv_ptr->toImageMsg());    
            i++;
        }
            
};

// Main loop
int main(int argc, char** argv){
    ros::init(argc, argv, "image_process");
    ROS_INFO_STREAM("Node initialized: process_image");

    ImageProcess ip;
    ros::spin();
    return 0;
}