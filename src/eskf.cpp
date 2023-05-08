#include <eskf.h>

using namespace ESKF;

Eigen::Vector3d a_b(-0.348513345960806, -0.26021251227717, 0.132337782341719);
Eigen::Vector3d a_s(-0.00426378745053201, 0.000725755116990245, 0.00263712381843959);
Eigen::Vector3d g_b(0.00710659149934062, 0.00211909908717263, -0.0000592951099686292);
Eigen::Vector3d g_s(-2.36707629594559, -0.490347919324706, -0.686283178454847);
static double random_walk_noise_vel_x = 0.0795288358707579; // (m/s^2)/(s^0.5)
static double random_walk_noise_vel_y = 0.084039859849866; // (m/s^2)/(s^0.5)
static double random_walk_noise_vel_z = 0.108319949650409; // (m/s^2)/(s^0.5)
static double random_walk_noise_att_x = 0.0108912090708668; // (rad)/(s^0.5)
static double random_walk_noise_att_y = 0.00574123650658026; // (rad)/(s^0.5)
static double random_walk_noise_att_z = 0.0064446242367083; // (rad)/(s^0.5)

inline double earth_radius_along_meridian(double lat_rad){
    double m = (EARTH_SEMIMAJOR*(1-ECCENTRICITY*ECCENTRICITY))/pow(1-pow(ECCENTRICITY*sin(lat_rad),2),1.5);
    return m;
}
inline double earth_radius_along_prime_vertical(double lat_rad){
    double n = EARTH_SEMIMAJOR/sqrt(1-pow(ECCENTRICITY*sin(lat_rad),2));
    return n;
}

Fusion::Fusion(ros::Publisher pub_fusion, ESKF::config eskf_config):pub_fusion_(pub_fusion), eskf_config_(eskf_config){};

// Initialize
void Fusion::Initilize_error_state(){
    Eigen::Vector3d zero(0,0,0);
    err_state_.position = zero;
    err_state_.velocity = zero;
    err_state_.attitude = zero;
    err_state_.gyroscope = zero;
    err_state_.accelerometer = zero;
}
void Fusion::Initilize_eskf_variable(){
    eskf_var_.F = Eigen::MatrixXd::Zero(15,15);
    eskf_var_.x = Eigen::MatrixXd::Zero(15,1);
    eskf_var_.G = Eigen::MatrixXd::Identity(15,15);
    eskf_var_.Q = Eigen::MatrixXd::Zero(15,15);
    eskf_var_.Z = Eigen::MatrixXd::Zero(15,1);
    eskf_var_.H = Eigen::MatrixXd::Identity(15,15);
    eskf_var_.R = Eigen::MatrixXd::Zero(15,15);
    eskf_var_.x_priori = Eigen::MatrixXd::Zero(15,1);
    eskf_var_.Theta = Eigen::MatrixXd::Zero(15,15);
    eskf_var_.P_priori = Eigen::MatrixXd::Zero(15,15);
    eskf_var_.K = Eigen::MatrixXd::Zero(15,15);
    eskf_var_.P = Eigen::MatrixXd::Zero(15,15);
}

// update data
void Fusion::ubloxFIXcallback(const sensor_msgs::NavSatFix& msg){
    Eigen::Vector3d R_pos(msg.position_covariance[0],msg.position_covariance[4],msg.position_covariance[8]);
    ublox_fix_ = msg;
    R_pos_ = R_pos;
    eskf_config_.ublox_fix_flag = true;
    if(eskf_config_.ins_flag == true){
        if(eskf_config_.ublox_vel_flag && eskf_config_.ublox_att_flag){
            eskf_config_.ublox_fix_flag = eskf_config_.ublox_vel_flag = eskf_config_.ublox_att_flag = false;
            time_interval_ = msg.header.stamp.toSec() - ins_fix_.header.stamp.toSec();
            KF_algorithm();
            publish_fusion();
        }
    }
    else std::cout << "\033[33m" << "Waiting for First INS data !" << "\033[0m" << std::endl;
}
void Fusion::ubloxVELcallback(const geometry_msgs::TwistWithCovarianceStamped& msg){
    Eigen::Vector3d R_vel(msg.twist.covariance[0],msg.twist.covariance[7],msg.twist.covariance[14]);
    ublox_vel_ = msg;
    R_vel_ = R_vel;
    eskf_config_.ublox_vel_flag = true;
    if(eskf_config_.ins_flag == true){
        if(eskf_config_.ublox_fix_flag && eskf_config_.ublox_att_flag){
            eskf_config_.ublox_fix_flag = eskf_config_.ublox_vel_flag = eskf_config_.ublox_att_flag = false;
            time_interval_ = msg.header.stamp.toSec() - ins_fix_.header.stamp.toSec();
            KF_algorithm();
            publish_fusion();
        }
    }
    else std::cout << "\033[33m" << "Waiting for First INS data !" << "\033[0m" << std::endl;
}       
void Fusion::ubloxATTcallback(const ublox_msgs::NavATT& msg){
    Eigen::Vector3d R_att(msg.accPitch,msg.accRoll,msg.accHeading);
    R_att = R_att/100000*DEG_TO_RAD;
    ublox_att_ = msg;
    c = R_att;
    eskf_config_.ublox_att_flag = true;
    if(eskf_config_.ins_flag == true){
        if(eskf_config_.ublox_fix_flag && eskf_config_.ublox_vel_flag){
            eskf_config_.ublox_fix_flag = eskf_config_.ublox_vel_flag = eskf_config_.ublox_att_flag = false;
            time_interval_ = ublox_fix_.header.stamp.toSec() - ins_fix_.header.stamp.toSec();
            KF_algorithm();
            publish_fusion();
        }
    } 
    else std::cout << "\033[33m" << "Waiting for First INS data !" << "\033[0m" << std::endl;
}
void Fusion::novatelINSPVAcallback(const novatel_gps_msgs::Inspva& msg){
    if(eskf_config_.ins_flag == true){
        if(eskf_config_.novatel_count >= 100 && eskf_config_.novatel_cov_flag){
            eskf_config_.novatel_count = 0;
            eskf_config_.novatel_cov_flag = false;
            time_interval_ = msg.header.stamp.toSec() - ins_fix_.header.stamp.toSec();
            novatel_fix_ = msg;
            KF_algorithm();
            publish_fusion();
        }
        else eskf_config_.novatel_count++;
    } 
    else std::cout << "\033[33m" << "Waiting for First INS data !" << "\033[0m" << std::endl;
}
void Fusion::novatelINSCOVcallback(const novatel_gps_msgs::Inscov& msg){
    eskf_config_.novatel_cov_flag = true;
    Eigen::Vector3d R_pos(msg.position_covariance[0],msg.position_covariance[4],msg.position_covariance[8]);
    Eigen::Vector3d R_vel(msg.velocity_covariance[0],msg.velocity_covariance[4],msg.velocity_covariance[8]);
    Eigen::Vector3d R_att(msg.attitude_covariance[0],msg.attitude_covariance[4],msg.attitude_covariance[8]);
    R_pos_ = R_pos;
    R_vel_ = R_vel;
    R_att_ = R_att;
}
void Fusion::uwbFIXcallback(const uwb_ins_eskf_msgs::uwbFIX& msg){
    uwb_fix_ = msg;
    // TBD
    // R_pos_ = 
    // R_vel_ = 
    // R_att_ =
    if(eskf_config_.ins_flag == true){
        time_interval_ = msg.header.stamp.toSec() - ins_fix_.header.stamp.toSec();
        KF_algorithm();
        publish_fusion();
    }
    else std::cout << "\033[33m" << "Waiting for First INS data !" << "\033[0m" << std::endl;
}
void Fusion::insFIXcallback(const uwb_ins_eskf_msgs::InsFIX& msg){
    eskf_config_.ins_flag = true;
    ins_fix_ = msg;
}
void Fusion::update_error_state(){
    Eigen::Vector3d tmp;
    if(eskf_config_.fusion_type == 0){
        tmp(0) = uwb_fix_.latitude - ins_fix_.latitude;
        tmp(1) = uwb_fix_.longitude - ins_fix_.longitude;
        tmp(2) = uwb_fix_.altitude - ins_fix_.altitude;
        err_state_.position = tmp;

        tmp(0) = uwb_fix_.velocity_e - ins_fix_.velocity_e;
        tmp(1) = uwb_fix_.velocity_n - ins_fix_.velocity_n;
        tmp(2) = uwb_fix_.velocity_u - ins_fix_.velocity_u;
        err_state_.velocity = tmp;

        // tmp(0) = uwb_fix_.att_e - ins_fix_.att_e;
        // tmp(1) = uwb_fix_.att_n - ins_fix_.att_n;  
        // tmp(2) = uwb_fix_.att_u - ins_fix_.att_u;
        // err_state_.attitude = tmp;
    }
    else if(eskf_config_.fusion_type == 1){
        tmp(0) = ublox_fix_.latitude - ins_fix_.latitude;
        tmp(1) = ublox_fix_.longitude - ins_fix_.longitude;
        tmp(2) = ublox_fix_.altitude - ins_fix_.altitude;
        err_state_.position = tmp;

        tmp(0) = ublox_vel_.twist.twist.linear.x - ins_fix_.velocity_e;
        tmp(1) = ublox_vel_.twist.twist.linear.y - ins_fix_.velocity_n;
        tmp(2) = ublox_vel_.twist.twist.linear.z - ins_fix_.velocity_u;
        err_state_.velocity = tmp;

        // tmp(0) = ublox_att_.pitch - ins_fix_.att_e;
        // tmp(1) = ublox_att_.roll - ins_fix_.att_n;  
        // tmp(2) = -ublox_att_.heading - ins_fix_.att_u;
        // err_state_.attitude = tmp;
    }
    else if(eskf_config_.fusion_type == 2){
        tmp(0) = novatel_fix_.latitude - ins_fix_.latitude;
        tmp(1) = novatel_fix_.longitude - ins_fix_.longitude;
        tmp(2) = novatel_fix_.altitude - ins_fix_.altitude;
        err_state_.position = tmp;

        tmp(0) = novatel_fix_.east_velocity - ins_fix_.velocity_e;
        tmp(1) = novatel_fix_.north_velocity - ins_fix_.velocity_n;
        tmp(2) = novatel_fix_.up_velocity - ins_fix_.velocity_u;
        err_state_.velocity = tmp;

        // tmp(0) = novatel_fix_.pitch - ins_fix_.att_e;
        // tmp(1) = novatel_fix_.roll - ins_fix_.att_n;  
        // tmp(2) = -novatel_fix_.azimuth - ins_fix_.att_u;
        // err_state_.attitude = tmp;
    }

    err_state_.gyroscope = g_b;

    err_state_.accelerometer = a_b;
}

// algorithm
void Fusion::update_F(){
    //position
    eskf_var_.F(0,4) = 1/(earth_radius_along_meridian(ins_fix_.latitude*DEG_TO_RAD)+ins_fix_.altitude);
    eskf_var_.F(1,3) = 1/((earth_radius_along_prime_vertical(ins_fix_.latitude*DEG_TO_RAD)+ins_fix_.altitude)*cos(ins_fix_.latitude*DEG_TO_RAD));
    eskf_var_.F(2,5) = 1;
    //velocity
    eskf_var_.F(3,7) = ins_fix_.f_u;
    eskf_var_.F(3,8) = -ins_fix_.f_n;
    eskf_var_.F(4,6) = -ins_fix_.f_u;
    eskf_var_.F(4,8) = ins_fix_.f_e;
    eskf_var_.F(5,6) = ins_fix_.f_n;
    eskf_var_.F(5,7) = -ins_fix_.f_e;
    //attitude
    eskf_var_.F(6,4) = 1/(earth_radius_along_prime_vertical(ins_fix_.latitude*DEG_TO_RAD)+ins_fix_.altitude);
    eskf_var_.F(7,3) = -1/(earth_radius_along_prime_vertical(ins_fix_.latitude*DEG_TO_RAD)+ins_fix_.altitude);
    eskf_var_.F(8,3) = -(tan(ins_fix_.latitude*DEG_TO_RAD))/(earth_radius_along_prime_vertical(ins_fix_.latitude*DEG_TO_RAD)+ins_fix_.altitude);
}
void Fusion::update_Q(){
    double time_interval = time_interval_;

    // position
    eskf_var_.Q(0,0) = pow(random_walk_noise_vel_x,2)*time_interval+pow(random_walk_noise_att_x,2)*time_interval;
    eskf_var_.Q(1,1) = pow(random_walk_noise_vel_y,2)*time_interval+pow(random_walk_noise_att_y,2)*time_interval;
    eskf_var_.Q(2,2) = pow(random_walk_noise_vel_z,2)*time_interval+pow(random_walk_noise_att_z,2)*time_interval;
    // velocity
    eskf_var_.Q(3,3) = pow(random_walk_noise_vel_x,2)*time_interval;
    eskf_var_.Q(4,4) = pow(random_walk_noise_vel_y,2)*time_interval;
    eskf_var_.Q(5,5) = pow(random_walk_noise_vel_z,2)*time_interval;
    // attitude
    eskf_var_.Q(6,6) = pow(random_walk_noise_att_x,2)*time_interval;
    eskf_var_.Q(7,7) = pow(random_walk_noise_att_y,2)*time_interval;
    eskf_var_.Q(8,8) = pow(random_walk_noise_att_z,2)*time_interval;
}

void Fusion::update_Z(){
    //position
    eskf_var_.Z(0) = err_state_.position(0)*DEG_TO_RAD;
    eskf_var_.Z(1) = err_state_.position(1)*DEG_TO_RAD;
    eskf_var_.Z(2) = err_state_.position(2);
    //velocity
    eskf_var_.Z(3) = err_state_.velocity(0);
    eskf_var_.Z(4) = err_state_.velocity(1);
    eskf_var_.Z(5) = err_state_.velocity(2);
    //attitude
    // eskf_var_.Z(6) = err_state_.attitude(0)*DEG_TO_RAD;
    // eskf_var_.Z(7) = err_state_.attitude(1)*DEG_TO_RAD;
    // eskf_var_.Z(8) = err_state_.attitude(2)*DEG_TO_RAD;
}

void Fusion::update_R(){
    // position
    eskf_var_.R(0,0) = R_pos_(0);
    eskf_var_.R(1,1) = R_pos_(1);
    eskf_var_.R(2,2) = R_pos_(2);
    // velocity
    eskf_var_.R(3,3) = R_vel_(0);
    eskf_var_.R(4,4) = R_vel_(1);
    eskf_var_.R(5,5) = R_vel_(2);
    // attitude
    eskf_var_.R(6,6) = R_att_(0);
    eskf_var_.R(7,7) = R_att_(1);
    eskf_var_.R(8,8) = R_att_(2);
}
void Fusion::update_Theta(){
    double time_interval = time_interval_;
    eskf_var_.Theta = Eigen::MatrixXd::Identity(15,15) + time_interval * eskf_var_.F;
}
void Fusion::update_x_priori(){
    eskf_var_.x_priori = eskf_var_.Theta * eskf_var_.x;
}
void Fusion::update_P_priori(){
    eskf_var_.P_priori = eskf_var_.Theta * eskf_var_.P * eskf_var_.Theta.transpose() + eskf_var_.G * eskf_var_.Q * eskf_var_.G.transpose();
}
void Fusion::update_K(){
    eskf_var_.K = eskf_var_.P_priori * eskf_var_.H.transpose() * (eskf_var_.H * eskf_var_.P_priori * eskf_var_.H.transpose() + eskf_var_.R).inverse();
}
void Fusion::update_x(){
    eskf_var_.x = eskf_var_.x_priori + eskf_var_.K * (eskf_var_.Z - eskf_var_.H * eskf_var_.x_priori);
}
void Fusion::update_P(){
    eskf_var_.P = (Eigen::MatrixXd::Identity(15,15) - eskf_var_.K * eskf_var_.H) * eskf_var_.P_priori;
}
void Fusion::Prediction(){
    update_x_priori();
    update_P_priori();
}
void Fusion::Update(){
    update_K();
    update_x();
    update_P();
}
void Fusion::KF_algorithm(){
    update_error_state();
    update_F();
    update_Q();
    update_Z();
    update_R();
    update_Theta();
    Prediction();
    Update();
}

// Result
void Fusion::publish_fusion(){
    uwb_ins_eskf_msgs::fusionFIX msg;
    ros::Time now = ros::Time::now();

    msg.header.stamp = now;
    msg.header.frame_id = "local";
    msg.latitude = ins_fix_.latitude + eskf_var_.x(0)*RAD_TO_DEG;
    msg.longitude = ins_fix_.longitude + eskf_var_.x(1)*RAD_TO_DEG;
    msg.altitude = ins_fix_.altitude + eskf_var_.x(2);
    msg.velocity_e = ins_fix_.velocity_e + eskf_var_.x(3);
    msg.velocity_n = ins_fix_.velocity_n + eskf_var_.x(4);
    msg.velocity_u = ins_fix_.velocity_u + eskf_var_.x(5);
    msg.att_e = ins_fix_.att_e + eskf_var_.x(6)*RAD_TO_DEG;
    msg.att_n = ins_fix_.att_n + eskf_var_.x(7)*RAD_TO_DEG;
    msg.att_u = ins_fix_.att_u + eskf_var_.x(8)*RAD_TO_DEG;

    pub_fusion_.publish(msg);
}