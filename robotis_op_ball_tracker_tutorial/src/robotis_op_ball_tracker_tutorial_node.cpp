
#include <robotis_op_ball_tracker_tutorial/robotis_op_ball_tracker_tutorial_node.h>

using namespace robotis_op;

#include <iostream>

#include <stdlib.h>     /* srand, rand */
#include <std_msgs/Float64.h>
#include <math.h>
#include <sensor_msgs/image_encodings.h>
namespace robotis_op {


RobotisOPBallTrackingNode::RobotisOPBallTrackingNode(ros::NodeHandle nh)
    : nh_(nh)
{
    /*
     * TODO
     * initialize Publisher and Subscriber
     * */
    joint_states_sub_ = nh_.subscribe("/robotis_op/joint_states", 100, &RobotisOPBallTrackingNode::jointStatesCb, this);

}

RobotisOPBallTrackingNode::~RobotisOPBallTrackingNode()
{
}


void RobotisOPBallTrackingNode::jointStatesCb(const sensor_msgs::JointState& msg)
{
    pan_ = msg.position.at(10);
    tilt_ = msg.position.at(21);
}

void RobotisOPBallTrackingNode::dynamicReconfigureCb(robotis_op_ball_tracker_tutorial::robotis_op_ball_tracker_tutorialConfig &config, uint32_t level)
{

    dp_ = config.dp;
    minDist_ = config.minDist;
    param1_ = config.param1;
    param2_ = config.param2;
    min_radius_ = config.minRadius;
    max_radius_ = config.maxRadius;
}

bool RobotisOPBallTrackingNode::detectCircles(const sensor_msgs::Image& msg, cv::Point& offset)
{
    cv_bridge::CvImagePtr image_trans, image_blob;
    image_trans = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
    image_blob = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);

    cv::Mat& mat_blob = image_blob->image;
    cv::Mat& mat_trans = image_trans->image;
    cv::GaussianBlur( mat_trans, mat_trans, cv::Size(9, 9), 2, 2 );

    std::vector<cv::Mat> canales;
    cv::split(mat_trans, canales);
    canales[0] =  2*canales[0]-canales[1]-canales[2];
    canales[1] = canales[2]-canales[2];
    canales[2] = canales[2]-canales[2];
    cv::merge(canales, mat_trans);
    cv::cvtColor(mat_trans,mat_trans, CV_RGB2GRAY);

    std::vector<cv::Vec3f> circles;

    //ROS_INFO("%f %f %f %f %f %f",dp_, minDist_, param1_, param2_,min_radius_, max_radius_);

    cv::HoughCircles(mat_trans, circles, CV_HOUGH_GRADIENT, dp_, minDist_, param1_, param2_,min_radius_, max_radius_);//1,  mat_trans.rows/8, 200, 10, 0, 0 );


    ROS_INFO("detected %i circles",(int)circles.size());
    cv::Point ball_position;
    bool ball_detected = false;
    for( size_t i = 0; i < circles.size(); i++ )
    {
        ball_detected = true;
        cv::Point center((circles[i][0]), (circles[i][1]));
        ball_position = center;
        cv::Point image_center(mat_blob.size().width/2.0,mat_blob.size().height/2.0);
        offset = ball_position - image_center ;
        int radius = (circles[i][2]);
        // circle center
        cv::circle( mat_blob, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        cv::circle( mat_blob, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
        ball_position = center;
    }

    /*
     * TODO
     * Publish image with ball marker (image_blob)
     * */

    return ball_detected;
}

/*
 * TODO
 * Implement Callback function(s)
 * */


}



int main(int argc, char **argv)
{

    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ros::NodeHandle nh;
    double control_rate;
    nh.param("robotis_op_walking/control_rate", control_rate, 125.0);
    control_rate = 125.0;

    RobotisOPBallTrackingNode ball_tracking_tutorial_node(nh);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time last_time = ros::Time::now();
    ros::Rate rate(control_rate);

    dynamic_reconfigure::Server<robotis_op_ball_tracker_tutorial::robotis_op_ball_tracker_tutorialConfig> srv;
    dynamic_reconfigure::Server<robotis_op_ball_tracker_tutorial::robotis_op_ball_tracker_tutorialConfig>::CallbackType cb;
    cb = boost::bind(&RobotisOPBallTrackingNode::dynamicReconfigureCb, &ball_tracking_tutorial_node, _1, _2);
    srv.setCallback(cb);

    ROS_INFO("Ball tracking enabled");

    while (ros::ok())
    {
        rate.sleep();
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        last_time = current_time;
    }

    return 0;
}

