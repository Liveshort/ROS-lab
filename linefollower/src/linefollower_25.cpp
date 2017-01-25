#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/bind.hpp>
#include <cmath>

#include <sys/time.h>

const double maxDriveSpeed = 0.05;
const double cameraWidth = 0.15;
const double tankToCamera = 0.21;

// black out the far corners of an image to improve line detection
//     also draw the masked area on the original camera view
cv::Mat mask_corners(cv::Mat binaryImg, cv::Mat drawImg, int imgWidth, int imgHeight){
    cv::Mat mask = cv::Mat::zeros(imgHeight, imgWidth, 0);
    cv::Point triangle_points[2][3];
    triangle_points[0][0] = cv::Point(0, 0);
    triangle_points[0][1] = cv::Point(100, 0);
    triangle_points[0][2] = cv::Point(0, 50);
    triangle_points[1][0] = cv::Point(0, imgHeight);
    triangle_points[1][1] = cv::Point(100, imgHeight);
    triangle_points[1][2] = cv::Point(0, imgHeight-50);
    const cv::Point* ppt1[1] = {triangle_points[0]};
    const cv::Point* ppt2[1] = {triangle_points[1]};
    int npt[] = {3};
    cv::fillPoly(mask, ppt1, npt, 1, cv::Scalar(255, 255, 255), 8);
    cv::fillPoly(mask, ppt2, npt, 1, cv::Scalar(255, 255, 255), 8);
    cv::bitwise_not(mask, mask);
    // apply the mask
    cv::Mat resMasked;
    binaryImg.copyTo(resMasked, mask);
    // make the mask visible on the final viewport
    cv::Point offset = cv::Point(drawImg.cols - imgWidth, 0);
    cv::line(drawImg, triangle_points[0][1] + offset, triangle_points[0][2] + offset, cv::Scalar(255, 0, 0), 2, 8);
    cv::line(drawImg, triangle_points[0][2] + offset, triangle_points[1][2] + offset, cv::Scalar(255, 0, 0), 2, 8);
    cv::line(drawImg, triangle_points[1][2] + offset, triangle_points[1][1] + offset, cv::Scalar(255, 0, 0), 2, 8);
    // return the image
    return resMasked;
}

// checks for straight lines and corners
int check_pattern(double *brightCoordinates, int total){
    // make a difference vector and calculate mean
    double diff[total - 1];
    double mean;
    for (int i=0; i < total-1; i++){
        diff[i] = brightCoordinates[i+1] - brightCoordinates[i];
        mean += diff[i];
    }
    mean /= (total-1);
    // calculate std
    double std = 0;
    for (int i=0; i < total-1; i++){
        std += (diff[i] - mean) * (diff[i] - mean);
    }
    std = sqrt(std / (total-1));
    // if the std is low, the supplied points must be in a straight line
    if (std < 1){
        return 0;
    }
    // if not, proceed to check for corners
    double diff2[total-2];
    double diff2Min = 10000; 
    double diff2Max = -10000;
    for (int i=0; i < total-2; i++){
        diff2[i] = diff[i+1] - diff[i];
        if (diff2[i] > diff2Max) diff2Max = diff2[i];
        if (diff2[i] < diff2Min) diff2Min = diff2[i];
    }
    if (diff2Min > 6){
        // ROS_INFO("Sharp turn left ahead: %f", diff2Min);
        return 2;
    } else if (diff2Max < -6){
        // ROS_INFO("Sharp turn right ahead: %f", diff2Max);
        return 4;
    } else if (diff2Min > 3){
        // ROS_INFO("Turn left ahead: %f", diff2Min);
        return 1;
    } else if (diff2Max < 3){
        // ROS_INFO("Turn right ahead: %f", diff2Max);
        return 3;
    } else{
        // ROS_INFO("Non-simple pattern ahead");
        return -1;
    }
}

// draws a sign on the final image if a straight line or corner is detected
void draw_sign(int pattern, cv::Mat drawImg, int imgHeight){
    // if no special pattern was detected, return immediately
    if (pattern == -1) return;
    // draw sign
    cv::Point sign_points[1][4];
    sign_points[0][0] = cv::Point(100, imgHeight/2);
    sign_points[0][1] = cv::Point(130, imgHeight/2 - 30);
    sign_points[0][2] = cv::Point(160, imgHeight/2);
    sign_points[0][3] = cv::Point(130, imgHeight/2 + 30);
    const cv::Point* ppt[1] = {sign_points[0]};
    int npt[] = {4};
    cv::fillPoly(drawImg, ppt, npt, 1, cv::Scalar(255, 36, 20), 8);
    // draw beginning of arrow
    cv::Point start = cv::Point(150, imgHeight/2);
    cv::Point middle = cv::Point(130, imgHeight/2);
    cv::line(drawImg, start, middle, cv::Scalar(255, 255, 255), 4, 8, 0);
    // determine the endpoint of the line
    cv::Point end;
    switch(pattern){
        case 0:
            end = cv::Point(110, imgHeight/2);
            break;
        case 1:
            end = cv::Point(118, imgHeight/2 + 12);
            break;
        case 2:
            end = cv::Point(130, imgHeight/2 + 25);
            break;
        case 3:
            end = cv::Point(118, imgHeight/2 - 12);
            break;
        case 4:
            end = cv::Point(130, imgHeight/2 - 25);
            break;
        default:
            end = middle;
    }
    // draw arrow
    cv::line(drawImg, middle, end, cv::Scalar(255, 255, 255), 4, 8, 0);
    return;
}

// imagecallback that handles incoming images
void imageCallback(const sensor_msgs::ImageConstPtr& msg, ros::Publisher pub){
    // convert the image to a cv image
    cv_bridge::CvImagePtr srcImgPtr;
    try{
        srcImgPtr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        return;
    }
    
    // check if images are large enough
    if (srcImgPtr->image.cols < 1700 || srcImgPtr->image.rows < 1080){
        ROS_ERROR("Supplied pictures are of insufficient size");
        return;
    }
    
    int scaleFactor = 3;
    
        struct timeval tp;
        gettimeofday(&tp, NULL);
        long int begin = tp.tv_usec / 1000;
    
    // scale the image
    cv::resize(srcImgPtr->image, srcImgPtr->image, cv::Size(), 1.0/scaleFactor, 1.0/scaleFactor, CV_INTER_LINEAR);
    
    // cut the upper half of the image to reduce errors due to looking ahead to far
    cv::Mat processImageRaw;
    int imgWidth, imgHeight;
    try{
        processImageRaw = cv::Mat(srcImgPtr->image, cv::Rect(srcImgPtr->image.cols - 320, 0, 320, srcImgPtr->image.rows));
        imgHeight = processImageRaw.rows;
        imgWidth = processImageRaw.cols;
    } catch (cv::Exception& e){
        ROS_ERROR("Cutting of the upper half failed: %s: %s", e.err.c_str(), e.msg.c_str());
        return;
    }
    
    // blur the image slightly to level the noise and smoothen the edges
    cv::Mat blurred;
    cv::medianBlur(processImageRaw, blurred, 3);
    
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
            if (cv::countNonZero(binaryBgr[i]) > (imgHeight * imgWidth / 2)){
                cv::bitwise_not(binaryBgr[i], binaryBgr[i]);
            }
        } else{
            binaryBgr[i] = cv::Mat::zeros(imgHeight, imgWidth, 0);
        }
    }
    
    // combine the three color channels back into a single matrix
    cv::Mat res;
    cv::bitwise_or(binaryBgr[0], binaryBgr[1], res, cv::noArray());
    cv::bitwise_or(res, binaryBgr[2], res);
    
    // black out corners of image
    cv::Mat resMasked = mask_corners(res, srcImgPtr->image, imgWidth, imgHeight);
    
    // create a vector of detection points
    int noOfRefPoints = 5;
    int refPoints[] = {imgWidth - 220, imgWidth - 170, imgWidth - 120, imgWidth - 70, imgWidth - 20};
    double meanBrights[noOfRefPoints];
    double meanBrightsBU[noOfRefPoints];
    int noOfLineDetected = 0;
    int noOfSomethingDetected = 0;
    // get the average y for all the reference points
    for (int j=0; j < noOfRefPoints; j++){
        // create a cutout of the image near the start and initilize an empty array of locations
        cv::Mat cutout;
        try{
            cutout = cv::Mat(resMasked, cv::Rect(refPoints[j], 0, 10, imgHeight));
        } catch (cv::Exception& e){
            ROS_ERROR("Cutout failed: %s: %s", e.err.c_str(), e.msg.c_str());
            return;
        }
        std::vector<cv::Point> locations;
        
        // save the locations of the bright pixels in the locations array
        try{
            cv::findNonZero(cutout, locations);
        } catch (cv::Exception& e){
            ROS_INFO("No line detected for reference point at x = %d", refPoints[j]);
        }
        
        // if there are no bright pixels, set the current mena to zero
        // if there are, calculate the right mean
        if (locations.size() == 0){
            meanBrights[j] = 0;
            meanBrightsBU[j] = 0;
        } else{
            // something is detected
            // calculate mean of bright points
            double y = 0;
            for (int i=0; i < locations.size(); i++){
                y += locations[i].y;
            }
            y /= locations.size();
            // calculate std of bright points
            double yStd = 0;
            for (int i=0; i < locations.size(); i++){
                yStd += (locations[i].y - y) * (locations[i].y - y);
            }
            yStd = sqrt(yStd/locations.size());
            // if the std of the line is too high, something messed up must be going on
            //     so then ignore the point
            if (yStd > 35){
                // something detected, but std too high to be a line
                noOfSomethingDetected++;
                // draw a red circle on the false detected mean
                cv::circle(srcImgPtr->image, cv::Point(refPoints[j]+srcImgPtr->image.cols-imgWidth+5, y), 5, cv::Scalar(0, 0, 255), 2, 8);
                meanBrights[j] = 0;
            } else{
                // line detected
                noOfLineDetected++;
                // draw a green circle on the detected mean
                cv::circle(srcImgPtr->image, cv::Point(refPoints[j]+srcImgPtr->image.cols-imgWidth+5, y), 5, cv::Scalar(0, 255, 0), 2, 8);
                // store the mean in the appropriate vector
                meanBrights[j] = y;
            }
            meanBrightsBU[j] = y;
        }
    }
    
    // determine final reference point
    double finalRefPoint;
    double finalSpeedFactor;
    double extraSteering = 1.0;
    // initialize some variables that keep track of driving backwards and standing still
    static int standingStill = 0;
    static int driveBackwards = 0;
    // check for straight line or corner if all data-points are present
    if (noOfLineDetected == noOfRefPoints){
        int pattern = check_pattern(meanBrights, noOfRefPoints);
        draw_sign(pattern, srcImgPtr->image, imgHeight);
        finalRefPoint = meanBrights[0];
        switch(pattern){
            case 0:
                finalSpeedFactor = 1;
                break;
            case 1:
                finalSpeedFactor = 0.7;
                break;
            case 2:
                finalSpeedFactor = 0.4;
                break;
            case 3:
                finalSpeedFactor = 0.7;
                break;
            case 4:
                finalSpeedFactor = 0.4;
                break;
            default:
                finalSpeedFactor = 0.4;
        }
        standingStill = 0;
        driveBackwards = 0;
    } else if (noOfLineDetected > 0){
        // calculate the mean to use for driving if at least one green point is present
        finalRefPoint = 0;
        for (int i=0; i < noOfRefPoints; i++){
            finalRefPoint += meanBrights[i];
        }
        finalRefPoint /= noOfLineDetected;
        finalSpeedFactor = 1 / sqrt(noOfRefPoints - noOfLineDetected + 1);
        extraSteering = 1.2;
        standingStill = 0;
        driveBackwards = 0;
    } else if (noOfSomethingDetected > 0){
        // calculate the mean based on the unreliable points, in the hope that this will go in the general right direction
        finalRefPoint = 0;
        for (int i=0; i < noOfRefPoints; i++){
            finalRefPoint += meanBrightsBU[i];
        }
        finalRefPoint /= noOfRefPoints;
        finalSpeedFactor = 1 / sqrt(noOfRefPoints);
        extraSteering = 1.2;
        standingStill = 0;
        driveBackwards = 0;
    } else{
        // if the tank has been standing still for a few frames, try driving backwards to reaquire the track
        if (standingStill > 5 && driveBackwards < 20){
            finalRefPoint = imgWidth/2;
            finalSpeedFactor = -0.5;
            driveBackwards++;
        // if no track is found for too long, just stand still
        } else{
            finalRefPoint = imgWidth/2;;
            finalSpeedFactor = 0;
        }
        standingStill++;
    }
    
    // calculate the ratio between linear and angular speed
    finalRefPoint = (finalRefPoint - imgWidth/2) / (imgWidth/2) * cameraWidth/2;
    double linAngRatio = (finalRefPoint * finalRefPoint + tankToCamera * tankToCamera) / (2 * finalRefPoint);
    ROS_INFO("Linangratio: %f", linAngRatio);
    
    // set up the command for publishing
    geometry_msgs::Twist cmd;
    cmd.linear.x = maxDriveSpeed * finalSpeedFactor;
    cmd.angular.z = maxDriveSpeed * finalSpeedFactor / linAngRatio * 1.7 * extraSteering;
    ROS_INFO("Linear speed: %f, angular speed: %f", cmd.linear.x, cmd.angular.z);
    
    // publish the command
    pub.publish(cmd);
    
        gettimeofday(&tp, NULL);
        long int middle = tp.tv_usec / 1000;
    
    // display the image
    cv::transpose(srcImgPtr->image, srcImgPtr->image);
    cv::flip(srcImgPtr->image, srcImgPtr->image, 1);
    cv::imshow("view", srcImgPtr->image);
    cv::waitKey(20);
    
        gettimeofday(&tp, NULL);
        long int end = tp.tv_usec / 1000;
    
    long int dif1 = middle-begin;
    long int dif2 = end-middle;
    
    ROS_INFO("Time between begin and middle: %lo ms, time between middle and end: %lo ms", dif1, dif2);
}

int main(int argc, char **argv){
    // initialize ros
    ros::init(argc, argv, "linefollower");
    ros::NodeHandle nh;
    
    // initialize publisher
    ros::NodeHandle pnh;
    ros::Publisher pub = pnh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
    // initialize image crawler
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/image", 1, boost::bind(imageCallback, _1, pub));
    
    cv::namedWindow("view");
    cv::startWindowThread();
    
    ros::spin();
    
    cv::destroyWindow("view");

    return 0;
}

