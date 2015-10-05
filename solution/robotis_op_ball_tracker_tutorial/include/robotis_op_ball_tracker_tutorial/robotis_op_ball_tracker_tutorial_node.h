#ifndef ROBOTIS_OP_BALL_TRACKING_TUTORIAL_NODE_H
#define ROBOTIS_OP_BALL_TRACKING_TUTORIAL_NODE_H

#include <ros/ros.h>


#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <robotis_op_ball_tracker_tutorial/robotis_op_ball_tracker_tutorialConfig.h>

namespace robotis_op {

class RobotisOPBallTrackingNode {
public:
    RobotisOPBallTrackingNode(ros::NodeHandle nh);
    ~RobotisOPBallTrackingNode();
    void dynamicReconfigureCb(robotis_op_ball_tracker_tutorial::robotis_op_ball_tracker_tutorialConfig &config, uint32_t level);

    void Process();
protected:


private:


    //returns true if a cirvle has been detected, offset contains the distance from the center to the detected circle if an circle has been detected
    bool detectCircles(const sensor_msgs::Image& msg, cv::Point& offset);
    /*
     * TODO
     * define Callback(s)
     * */
    void imageCb(const sensor_msgs::Image& msg);
    void jointStatesCb(const sensor_msgs::JointState& msg);   

    ros::NodeHandle nh_;

    /*
     * TODO
     * define Publisher and Subscriber
     * */
    ros::Subscriber joint_states_sub_;
    ros::Subscriber image_sub_;

    ros::Publisher tilt_pub_;
    ros::Publisher pan_pub_;
    ros::Publisher vel_pub_;
    ros::Publisher transformed_img_pub_;
    ros::Publisher blob_img_pub_;


    double dp_, minDist_, param1_, param2_, min_radius_, max_radius_;
    double pan_, tilt_;
    int count_no_detection_, count_search_loop_;



};

}
#endif //ROBOTIS_OP_BALL_TRACKING_TUTORIAL_NODE_H
