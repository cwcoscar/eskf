#include <eskf.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "eskf");
    ESKF::config eskf_config;

    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    nh.param("fusion_type", eskf_config.fusion_type, 2);
    nh.param("antenna_x", eskf_config.gnss_b(0), 0.00);
    nh.param("antenna_y", eskf_config.gnss_b(1), 0.25);
    nh.param("antenna_z", eskf_config.gnss_b(2), 0.43);
    nh.param("A0_x", eskf_config.uwb_b(0), 0.00);
    nh.param("A0_y", eskf_config.uwb_b(1), 0.05);
    nh.param("A0_z", eskf_config.uwb_b(2), 0.43);

    /* Publishers of fusion solution */
    ros::Publisher pub_fusion_fix = n.advertise<uwb_ins_eskf_msgs::fusionFIX>("/fusion/baselink/fix", 1);

    ESKF::Fusion fusion(pub_fusion_fix, eskf_config);
    fusion.Initilize_error_state();
    fusion.Initilize_eskf_variable();

    /* Subscribers of GNSS data */
    ros::Subscriber sub_uwb;
    ros::Subscriber sub_ublox_fix;
    ros::Subscriber sub_ublox_vel;
    ros::Subscriber sub_ublox_att;
    ros::Subscriber sub_novatel;
    ros::Subscriber sub_novatel_cov;
    if(eskf_config.fusion_type == 0){
        sub_uwb = n.subscribe("/uwb_position/A0", 1, &ESKF::Fusion::uwbFIXcallback, &fusion);
    }
    else if(eskf_config.fusion_type == 1){
        sub_ublox_fix = n.subscribe("/ublox_f9k/fix", 1, &ESKF::Fusion::ubloxFIXcallback, &fusion);
        sub_ublox_vel = n.subscribe("/ublox_f9k/fix_velocity", 1, &ESKF::Fusion::ubloxVELcallback, &fusion);
        sub_ublox_att = n.subscribe("/ublox_f9k/navatt", 1, &ESKF::Fusion::ubloxATTcallback, &fusion);
    }
    else if(eskf_config.fusion_type == 2){
        sub_novatel = n.subscribe("/novatel/inspva", 1, &ESKF::Fusion::novatelINSPVAcallback, &fusion);
        sub_novatel_cov = n.subscribe("/novatel/inscov", 1, &ESKF::Fusion::novatelINSCOVcallback, &fusion);
    }
    
    ros::Subscriber sub_ins = n.subscribe("/ins/fix", 1, &ESKF::Fusion::insFIXcallback, &fusion);

    ros::spin();

    return 0;
}