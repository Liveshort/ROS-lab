#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/time.h>

class RobotControl{
private:
    ros::NodeHandle nh_;
    
public:
    // initializer
    RobotControl(ros::NodeHandle &nh){
        nh_ = nh;
    }
    
    // imagecallback that handles incoming images
    static void imageCallback(const sensor_msgs::ImageConstPtr& msg){
        // convert the image to a cv image
        cv_bridge::CvImagePtr srcImgPtr;
        try{
            srcImgPtr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e){
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            return;
        }
        
        int scaleFactor = 3;
        
            struct timeval tp;
            gettimeofday(&tp, NULL);
            long int begin = tp.tv_usec / 1000;
        
        // scale the image
        cv::resize(srcImgPtr->image, srcImgPtr->image, cv::Size(), 1.0/scaleFactor, 1.0/scaleFactor, CV_INTER_LINEAR);
        
        // blur the image slightly to level the noise and smoothen the edges
        cv::Mat blurred;
        cv::medianBlur(srcImgPtr->image, blurred, 3);
        
        // split the RGB channels
        cv::Mat bgr[3];
        cv::split(blurred, bgr);
        
        // turn every channel into a binary image
        // note that this only works correctly if the line is not too thin
        //     and the line does not take up more than 50% of the screen
        cv::Mat binaryBgr[3];
        for(int i=0; i < 3; i++){
            // get the minimum and maximum of the current picture
            double min, max;
            cv::minMaxIdx(bgr[i], &min, &max, NULL, NULL, cv::noArray());
            double mean = (min + max) / 2;
            // also get the std deviation to check if the current color channel is not empty
            cv::Scalar placeholderMean, std;
            cv::meanStdDev(bgr[i], placeholderMean, std, cv::noArray());
            
            // if the the current channel is not empty
            if (std.val[0] > 20.0){
                // threshold the image to binary on the mean
                cv::threshold(bgr[i], binaryBgr[i], (placeholderMean.val[0] + mean)/3, 255, 0);
                // if more than 50% of the image is white, invert
                if (cv::countNonZero(binaryBgr[i]) > (1920 * 1080 / scaleFactor / scaleFactor / 2)){
                    cv::bitwise_not(binaryBgr[i], binaryBgr[i]);
                }
            } else{
                binaryBgr[i] = cv::Mat::zeros(1080 / scaleFactor, 1920 / scaleFactor, 0);
            }
        }
        
        // combine the three color channels back into a single matrix
        cv::Mat res;
        cv::bitwise_or(binaryBgr[0], binaryBgr[1], res, cv::noArray());
        cv::bitwise_or(res, binaryBgr[2], res);
        
        // create a vector of detection points
        int noOfRefPoints = 3;
        int refPoints[] = {400, 510, 620};
        double meanBrights[noOfRefPoints];
        int noOfLineDetected = 0;
        // get the average y for all the reference points
        for (int j=0; j < noOfRefPoints; j++){
            // create a cutout of the image near the start and initilize an empty array of locations
            cv::Mat cutout;
            try{
                cutout = cv::Mat(res, cv::Rect(refPoints[j], 0, 10, 1080 / scaleFactor));
            } catch (cv::Exception& e){
                ROS_ERROR("%s: %s", e.err.c_str(), e.msg.c_str());
            }
            std::vector<cv::Point> locations;
            
            // save the locations of the bright pixels in the locations array
            try{
                cv::findNonZero(cutout, locations);
            } catch (cv::Exception& e){
                ROS_ERROR("No line detected for reference point at x = %d", refPoints[j]);
            }
            
            // if there are no bright pixels, set the current mena to zero
            // if there are, calculate the right mean
            if (locations.size() == 0){
                meanBrights[j] = 0;
            } else{
                // line detected
                noOfLineDetected++;
                // calculate mean of bright points
                double y = 0;
                for (int i=0; i < locations.size(); i++){
                    y += locations[i].y;
                }
                y /= locations.size();
                // draw a circle on the detected mean
                cv::circle(srcImgPtr->image, cv::Point(refPoints[j]+10, y), 5, cv::Scalar(0, 255, 0), 2, 8);
                // store the mean in the appropriate vector
                meanBrights[j] = y;
            }
        }
        
        // do the calculation for the final reference point
        double meanBright;
        if (noOfLineDetected > 0){
            // calculate the mean to use for driving
            meanBright = 0;
            for (int i=0; i < noOfRefPoints; i++){
                meanBright += meanBrights[i];
            }
            meanBright /= noOfLineDetected;
            ROS_INFO("The mean of the bright pixels is %f", meanBright);
        } else {
            // if there is no line, set meanBright to -1
            meanBright = -1;
            ROS_INFO("No line detected");
        }
        
        /*
        publish code goes here
        */
        
            gettimeofday(&tp, NULL);
            long int middle = tp.tv_usec / 1000;
        
        // display the image
        cv::transpose(srcImgPtr->image, srcImgPtr->image);
        cv::flip(srcImgPtr->image, srcImgPtr->image, 1);
        cv::imshow("view", srcImgPtr->image);
        cv::waitKey(30);
        
            gettimeofday(&tp, NULL);
            long int end = tp.tv_usec / 1000;
        
        long int dif1 = middle-begin;
        long int dif2 = end-middle;
        
        ROS_INFO("Time between begin and middle: %lo ms, time between middle and end: %lo ms", dif1, dif2);
    }
};

int main(int argc, char **argv){
    // initialize ros
    ros::init(argc, argv, "linefollower");
    ros::NodeHandle nh;
    
    // create the robotcontroller
    RobotControl rcontrol(nh);
    
    cv::namedWindow("view");
    cv::startWindowThread();
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image", 1, rcontrol.imageCallback);
    
    ros::spin();
    
    cv::destroyWindow("view");

    return 0;
}

