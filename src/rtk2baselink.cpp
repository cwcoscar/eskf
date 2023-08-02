#include "ros/ros.h"
#include "coordinate_mat_transformation.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <uwb_ins_eskf_msgs/uwbFIX.h>
#include <ublox_msgs/NavATT.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <novatel_gps_msgs/Inspva.h>

#define NCKUEE_LATITUDE 22.99665875 /* degree */
#define NCKUEE_LONGITUDE 120.222584889 /* degree */
#define NCKUEE_HEIGHT 98.211 /* meter */

class Baselink{
    private:
        ros::Publisher pub_b_;
        ros::Publisher pub_b2_;
        tf::TransformBroadcaster br_;
        tf::TransformBroadcaster br2_;
        Eigen::Vector3d pos_;
        Eigen::Vector3d pos2_;
        Eigen::Vector3d vel_;
        Eigen::Matrix3d R_b_l_;
        Eigen::Vector3d att_l_;
        bool fix_flag_ = false;
        bool vel_flag_ = false;
        bool att_flag_ = false;
        std::string rtk_topic_;
        Eigen::Vector3d rtk_b_; /* antenna location in b-frame */
        ros::Time now_;
    public:
        Baselink(ros::Publisher &pub_b, ros::Publisher &pub_b2, std::string &rtk_topic, Eigen::Vector3d &rtk_b);
        void GNSSattcallback(const ublox_msgs::NavATT& msg);
        void GNSSvelcallback(const geometry_msgs::TwistWithCovarianceStamped& msg);
        void GNSSfixcallback(const sensor_msgs::NavSatFix& msg);
        void Novatelinspvacallback(const novatel_gps_msgs::Inspva& msg);
        void Novatelfixcallback(const sensor_msgs::NavSatFix& msg);
        void send_tf(uwb_ins_eskf_msgs::uwbFIX now_fix, Eigen::Vector3d now_enu, std::string tf_name);
        void publish();
};

Baselink::Baselink(ros::Publisher &pub_b, ros::Publisher &pub_b2, std::string &rtk_topic, Eigen::Vector3d &rtk_b)
:pub_b_(pub_b), pub_b2_(pub_b2), rtk_topic_(rtk_topic), rtk_b_(rtk_b){}

void Baselink::send_tf(uwb_ins_eskf_msgs::uwbFIX now_fix, Eigen::Vector3d now_enu, std::string tf_name){
    tf::Transform transform;
    tf::Quaternion current_q;

    current_q.setRPY(coordinate_mat_transformation::deg2Rad(now_fix.att_e), 
                     coordinate_mat_transformation::deg2Rad(now_fix.att_n), 
                     coordinate_mat_transformation::deg2Rad(now_fix.att_u));
    transform.setOrigin(tf::Vector3(now_enu(0), now_enu(1), now_enu(2)));
    transform.setRotation(current_q);
    br_.sendTransform(tf::StampedTransform(transform, now_fix.header.stamp, "/map", tf_name));
    std::cout << "Sending tf" + tf_name << std::endl;
}

void Baselink::publish(){
    if(fix_flag_ & vel_flag_ & att_flag_){
        uwb_ins_eskf_msgs::uwbFIX msg;

        Eigen::Vector3d ref_lla(NCKUEE_LATITUDE, NCKUEE_LONGITUDE, NCKUEE_HEIGHT);
        Eigen::VectorXd now_enu = coordinate_mat_transformation::lla2enu(pos_ ,ref_lla);
        Eigen::Vector3d baselink_enu = now_enu - R_b_l_*rtk_b_;
        std::cout << std::fixed << std::setprecision(10);
        Eigen::VectorXd baselink_lla = coordinate_mat_transformation::enu2Geodetic(baselink_enu ,ref_lla);

        msg.header.stamp = now_;
        msg.header.frame_id = "local";
        msg.latitude = baselink_lla[0];
        msg.longitude = baselink_lla[1];
        msg.altitude = baselink_lla[2];
        msg.velocity_e = vel_[0];
        msg.velocity_n = vel_[1];
        msg.velocity_u = vel_[2];
        msg.att_e = att_l_[0];
        msg.att_n = att_l_[1];
        msg.att_u = att_l_[2];

        /* For /novatel/inspva */
        if(rtk_topic_ == "/novatel"){
            uwb_ins_eskf_msgs::uwbFIX msg2 = msg;
            Eigen::VectorXd now_enu2 = coordinate_mat_transformation::lla2enu(pos2_ ,ref_lla);
            Eigen::Vector3d baselink_enu2 = now_enu2 - R_b_l_*rtk_b_;
            Eigen::VectorXd baselink_lla2 = coordinate_mat_transformation::enu2Geodetic(baselink_enu2 ,ref_lla);
            msg2.latitude = baselink_lla2[0];
            msg2.longitude = baselink_lla2[1];
            msg2.altitude = baselink_lla2[2];
            send_tf(msg2, baselink_enu2, rtk_topic_+"_inspva_baselink");
            pub_b2_.publish(msg2);
        }

        pub_b_.publish(msg);
        send_tf(msg, baselink_enu, rtk_topic_+"_baselink");
    }
}

void Baselink::GNSSfixcallback(const sensor_msgs::NavSatFix& msg){
    Eigen::Vector3d position(msg.latitude, msg.longitude, msg.altitude);
    now_ = msg.header.stamp;
    pos_ = position;
    fix_flag_ = true;
    // std::cout << "\033[32m" << "position " << std::endl << position << "\033[0m" << std::endl;
}

void Baselink::GNSSvelcallback(const geometry_msgs::TwistWithCovarianceStamped& msg){ // /ublox_f9k/fix_velocity:ENU
    Eigen::Vector3d velocity(msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
    vel_ = velocity;
    vel_flag_ = true;
}
            
void Baselink::GNSSattcallback(const ublox_msgs::NavATT& msg){ 
    Eigen::Vector3d att(msg.roll/100000, msg.pitch/100000, msg.heading/100000);
    /* NED -> ENU */
    att(2) = 360 - att(2);
    double tmp = att(1);
    att(1) = att(0);
    att(0) = tmp;
    /* 0 ~ 360 -> -180 ~ 180 */
    for (int i = 0; i < 3; i++){
        if (att(i) > 180 && att(i) < 360){
            att(i) = att(i) - 360;
        }
    }
    Eigen::Matrix3d R_b_l = coordinate_mat_transformation::Rotation_matrix((att/ 180.0) * M_PI);
    Eigen::Vector3d att_l = att;
    // std::cout << "\033[32m" << "att_l " << std::endl << att_l << "\033[0m" << std::endl;
    R_b_l_ = R_b_l;
    att_l_ = att_l;
    att_flag_ = true;
    publish();
}

void Baselink::Novatelinspvacallback(const novatel_gps_msgs::Inspva& msg){
    Eigen::Vector3d position(msg.latitude, msg.longitude, msg.height);
    now_ = msg.header.stamp;
    pos2_ = position;
    Eigen::Vector3d velocity(msg.east_velocity, msg.north_velocity, msg.up_velocity);
    vel_ = velocity;
    vel_flag_ = true;
    Eigen::Vector3d att(msg.roll, msg.pitch, msg.azimuth);
    /* NED -> ENU */
    att(2) = 360 - att(2);
    double tmp = att(1);
    att(1) = att(0);
    att(0) = tmp;
    /* 0 ~ 360 -> -180 ~ 180 */
    for (int i = 0; i < 3; i++){
        if (att(i) > 180 && att(i) < 360){
            att(i) = att(i) - 360;
        }
    }
    Eigen::Matrix3d R_b_l = coordinate_mat_transformation::Rotation_matrix((att/ 180.0) * M_PI);
    Eigen::Vector3d att_l = att;
    R_b_l_ = R_b_l;
    att_l_ = att_l;
    att_flag_ = true;
    publish();
}
void Baselink::Novatelfixcallback(const sensor_msgs::NavSatFix& msg){
    Eigen::Vector3d position(msg.latitude, msg.longitude, msg.altitude);
    now_ = msg.header.stamp;
    pos_ = position;
    fix_flag_ = true;
    // std::cout << "\033[32m" << "position " << std::endl << position << "\033[0m" << std::endl;
}

int main(int argc, char **argv) {
    std::string rtk_topic;
    Eigen::Vector3d rtk_b;

    ros::init(argc, argv, "rtk2baselink");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.param("rtk_topic", rtk_topic, std::string("/ublox_f9k"));
    nh.param("antenna_x", rtk_b(0), 0.00);
    nh.param("antenna_y", rtk_b(1), 0.05);
    nh.param("antenna_z", rtk_b(2), 0.43);

    ros::Publisher pub_b = n.advertise<uwb_ins_eskf_msgs::uwbFIX>(rtk_topic + "/fix/baselink", 1);
    ros::Publisher pub_b2 = n.advertise<uwb_ins_eskf_msgs::uwbFIX>(rtk_topic + "/inspva/baselink", 1);

    Baselink rtk2baselink(pub_b, pub_b2, rtk_topic, rtk_b);

    ros::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;
    if(rtk_topic == "/ublox_f9k"){
        sub1 = n.subscribe(rtk_topic+"/fix", 1, &Baselink::GNSSfixcallback, &rtk2baselink);
        sub2 = n.subscribe(rtk_topic+"/fix_velocity", 1, &Baselink::GNSSvelcallback, &rtk2baselink);
        sub3 = n.subscribe(rtk_topic+"/navatt", 1, &Baselink::GNSSattcallback, &rtk2baselink);
    }
    if(rtk_topic == "/novatel"){
        sub1 = n.subscribe(rtk_topic+"/inspva", 1, &Baselink::Novatelinspvacallback, &rtk2baselink);
        sub2 = n.subscribe(rtk_topic+"/fix", 1, &Baselink::Novatelfixcallback, &rtk2baselink);
    }

    ros::spin();
}