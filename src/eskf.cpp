#include <eskf.h>
#include "coordinate_mat_transformation.h"

using namespace ESKF;

Eigen::Vector3d a_b(-0.348513345960806, -0.26021251227717, 0.132337782341719);
Eigen::Vector3d g_b(0.00710659149934062, 0.00211909908717263, -0.0000592951099686292);
// static double random_walk_noise_vel_x = 0.0795288358707579; // (m/s^2)/(s^0.5)
// static double random_walk_noise_vel_y = 0.084039859849866; // (m/s^2)/(s^0.5)
// static double random_walk_noise_vel_z = 0.108319949650409; // (m/s^2)/(s^0.5)
// static double random_walk_noise_att_x = 0.0108912090708668; // (rad)/(s^0.5)
// static double random_walk_noise_att_y = 0.00574123650658026; // (rad)/(s^0.5)
// static double random_walk_noise_att_z = 0.0064446242367083; // (rad)/(s^0.5)
static double random_walk_noise_vel_x = 1; // (m/s^2)/(s^0.5)
static double random_walk_noise_vel_y = 1; // (m/s^2)/(s^0.5)
static double random_walk_noise_vel_z = 1; // (m/s^2)/(s^0.5)
static double random_walk_noise_att_x = 1; // (rad)/(s^0.5)
static double random_walk_noise_att_y = 1; // (rad)/(s^0.5)
static double random_walk_noise_att_z = 1; // (rad)/(s^0.5)

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
    eskf_var_.Q = Eigen::MatrixXd::Identity(15,15);
    eskf_var_.Z = Eigen::MatrixXd::Zero(15,1);
    eskf_var_.H = Eigen::MatrixXd::Identity(15,15);
    eskf_var_.R = Eigen::MatrixXd::Zero(15,15);
    eskf_var_.x_priori = Eigen::MatrixXd::Zero(15,1);
    eskf_var_.Theta = Eigen::MatrixXd::Zero(15,15);
    eskf_var_.P_priori = Eigen::MatrixXd::Zero(15,15);
    eskf_var_.inn = Eigen::MatrixXd::Zero(15,1);
    eskf_var_.S_hat = Eigen::MatrixXd::Zero(15,15);
    eskf_var_.r_k = 0;
    eskf_var_.R_hat = Eigen::MatrixXd::Zero(15,15);
    eskf_var_.s = 0;
    eskf_var_.d = 0;
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
            send_tf();
            // Send ublox tf
            Eigen::Vector3d now_lla(ublox_fix_.latitude, ublox_fix_.longitude, ublox_fix_.altitude);
            Eigen::Vector3d now_att(ublox_att_.pitch, ublox_att_.roll, -ublox_att_.heading);
            send_tf(now_lla, now_att, "ublox");
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
            send_tf();
            // Send ublox tf
            Eigen::Vector3d now_lla(ublox_fix_.latitude, ublox_fix_.longitude, ublox_fix_.altitude);
            Eigen::Vector3d now_att(ublox_att_.pitch, ublox_att_.roll, -ublox_att_.heading);
            send_tf(now_lla, now_att, "ublox");
        }
    }
    else std::cout << "\033[33m" << "Waiting for First INS data !" << "\033[0m" << std::endl;
}       
void Fusion::ubloxATTcallback(const ublox_msgs::NavATT& msg){
    Eigen::Vector3d R_att(msg.accPitch,msg.accRoll,msg.accHeading);
    R_att = R_att/100000*DEG_TO_RAD;
    ublox_att_ = msg;
    R_att_ = R_att;
    eskf_config_.ublox_att_flag = true;
    if(eskf_config_.ins_flag == true){
        if(eskf_config_.ublox_fix_flag && eskf_config_.ublox_vel_flag){
            eskf_config_.ublox_fix_flag = eskf_config_.ublox_vel_flag = eskf_config_.ublox_att_flag = false;
            time_interval_ = ublox_fix_.header.stamp.toSec() - ins_fix_.header.stamp.toSec();
            KF_algorithm();
            publish_fusion();
            send_tf();
            // Send ublox tf
            Eigen::Vector3d now_lla(ublox_fix_.latitude, ublox_fix_.longitude, ublox_fix_.altitude);
            Eigen::Vector3d now_att(ublox_att_.pitch, ublox_att_.roll, -ublox_att_.heading);
            send_tf(now_lla, now_att, "ublox");
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
            // std::cout << std::fixed << std::setprecision(6);
            // std::cout << "\033[33m" << "novatel.toSec(): " << std::endl << msg.header.stamp.toSec() << "\033[0m" << std::endl;
            // std::cout << "\033[33m" << "ins_fix_.toSec(): " << std::endl << ins_fix_.header.stamp.toSec() << "\033[0m" << std::endl;
            novatel_fix_ = msg;
            KF_algorithm();
            publish_fusion();
            send_tf();
            // Send novatel tf
            Eigen::Vector3d now_lla(novatel_fix_.latitude, novatel_fix_.longitude, novatel_fix_.height);
            Eigen::Vector3d now_att(novatel_fix_.pitch, novatel_fix_.roll, -novatel_fix_.azimuth);
            send_tf(now_lla, now_att, "novatel");
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
    eskf_config_.uwb_flag = true;
    // initialize MNC
    static bool ini_cov = false;
    if(!ini_cov){
        double cov = 0.001;
        Eigen::Vector3d R_pos(cov,cov,10*cov);
        Eigen::Vector3d R_vel(cov,cov,10*cov);
        Eigen::Vector3d R_att(cov,cov,cov);
        R_pos_ = R_pos;
        R_vel_ = R_vel;
        R_att_ = R_att;
        ini_cov = true;
    }
    if(eskf_config_.ins_flag == true){
        time_interval_ = msg.header.stamp.toSec() - ins_fix_.header.stamp.toSec();
        KF_algorithm();
        publish_fusion();
        send_tf();
        eskf_config_.ins_flag = false;
        // std::cout << "\033[33m" << "publish eskf: " << ros::Time::now().toSec() << "\033[0m" << std::endl;
    }
    else std::cout << "\033[33m" << "Waiting for First INS data !" << "\033[0m" << std::endl;
}
void Fusion::insFIXcallback(const uwb_ins_eskf_msgs::InsFIX& msg){
    eskf_config_.ins_flag = true;
    ins_fix_ = msg;
    if(eskf_config_.uwb_flag == false){
        publish_ins();
        Eigen::Vector3d now_lla(ins_fix_.latitude, ins_fix_.longitude, ins_fix_.altitude);
        Eigen::Vector3d now_att(ins_fix_.att_e, ins_fix_.att_n, ins_fix_.att_u);
        send_tf(now_lla, now_att, "eskf");
        // std::cout << std::fixed << std::setprecision(2);
        // std::cout << "\033[33m" << "publish ins: " << ins_fix_.header.stamp.toSec() << "\033[0m" << std::endl;
    }
    eskf_config_.uwb_flag = false;
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

        tmp(0) = uwb_fix_.att_e - ins_fix_.att_e;
        tmp(1) = uwb_fix_.att_n - ins_fix_.att_n;  
        tmp(2) = uwb_fix_.att_u - ins_fix_.att_u;
        err_state_.attitude = tmp;
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

        Eigen::Vector3d att(ublox_att_.roll, ublox_att_.pitch, ublox_att_.heading);
        // NED -> ENU
        att(2) = 360 - att(2);
        double temp = att(1);
        att(1) = att(0);
        att(0) = temp;
        // 0 ~ 2PI -> -PI ~ PI
        for (int i = 0; i < 3; i++){
            if (att(i) > 180 && att(i) < 360){
                att(i) = att(i) - 360;
            }
        }
        tmp(0) = att(0) - ins_fix_.att_e;
        tmp(1) = att(1) - ins_fix_.att_n;  
        tmp(2) = att(2) - ins_fix_.att_u;
        err_state_.attitude = tmp;
    }
    else if(eskf_config_.fusion_type == 2){
        tmp(0) = novatel_fix_.latitude - ins_fix_.latitude;
        tmp(1) = novatel_fix_.longitude - ins_fix_.longitude;
        tmp(2) = novatel_fix_.height - ins_fix_.altitude;
        err_state_.position = tmp;

        tmp(0) = novatel_fix_.east_velocity - ins_fix_.velocity_e;
        tmp(1) = novatel_fix_.north_velocity - ins_fix_.velocity_n;
        tmp(2) = novatel_fix_.up_velocity - ins_fix_.velocity_u;
        err_state_.velocity = tmp;

        Eigen::Vector3d att(novatel_fix_.roll, novatel_fix_.pitch, novatel_fix_.azimuth);
        // NED -> ENU
        att(2) = 360 - att(2);
        double temp = att(1);
        att(1) = att(0);
        att(0) = temp;
        // 0 ~ 2PI -> -PI ~ PI
        for (int i = 0; i < 3; i++){
            if (att(i) > 180 && att(i) < 360){
                att(i) = att(i) - 360;
            }
        }
        tmp(0) = att(0) - ins_fix_.att_e;
        tmp(1) = att(1) - ins_fix_.att_n;  
        tmp(2) = att(2) - ins_fix_.att_u;
        err_state_.attitude = tmp;

        // std::cout << "\033[33m" << "position" << std::endl << err_state_.position << "\033[0m" << std::endl;
        // std::cout << "\033[33m" << "velocity" << std::endl << err_state_.velocity << "\033[0m" << std::endl;
        // std::cout << "\033[33m" << "ini_ins_attitude" << std::endl << err_state_.attitude << "\033[0m" << std::endl;
        // std::cout << "\033[33m" << "ini_gnss_attitude" << std::endl << err_state_.attitude << "\033[0m" << std::endl;
        // std::cout << "\033[33m" << "ini_err_attitude" << std::endl << err_state_.attitude << "\033[0m" << std::endl;
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
    // std::cout << "\033[33m" << "eskf_var_.F" << std::endl << eskf_var_.F << "\033[0m" << std::endl;
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
    // std::cout << "\033[33m" << "time_interval: " << std::endl << time_interval << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "eskf_var_.Q" << std::endl << eskf_var_.Q << "\033[0m" << std::endl;
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
    // 0 ~ 360 -> -180 ~ 180
    for (int i = 0; i < 3; i++){
        if (err_state_.attitude(i) > 180 && err_state_.attitude(i) < 360){
            err_state_.attitude(i) = err_state_.attitude(i) - 360;
        }
    } 
    eskf_var_.Z(6) = err_state_.attitude(0)*DEG_TO_RAD;
    eskf_var_.Z(7) = err_state_.attitude(1)*DEG_TO_RAD;
    eskf_var_.Z(8) = err_state_.attitude(2)*DEG_TO_RAD;
    // std::cout << "\033[33m" << "eskf_var_.Z" << std::endl << eskf_var_.Z << "\033[0m" << std::endl;
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
    // std::cout << "\033[33m" << "eskf_var_.R" << std::endl << eskf_var_.R << "\033[0m" << std::endl;
}
void Fusion::update_Theta(){
    double time_interval = time_interval_;
    eskf_var_.Theta = Eigen::MatrixXd::Identity(15,15) + time_interval * eskf_var_.F;
    // std::cout << "\033[33m" << "eskf_var_.Theta" << std::endl << eskf_var_.Theta << "\033[0m" << std::endl;
}
void Fusion::update_x_priori(){
    eskf_var_.x_priori = eskf_var_.Theta * eskf_var_.x;
    // std::cout << "\033[33m" << "eskf_var_.Theta" << std::endl << eskf_var_.Theta << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "eskf_var_.x" << std::endl << eskf_var_.x << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "eskf_var_.x_priori" << std::endl << eskf_var_.x_priori << "\033[0m" << std::endl;
}
void Fusion::update_P_priori(){
    eskf_var_.P_priori = eskf_var_.Theta * eskf_var_.P * eskf_var_.Theta.transpose() + eskf_var_.G * eskf_var_.Q * eskf_var_.G.transpose();
    // std::cout << "\033[33m" << "eskf_var_.P_priori" << std::endl << eskf_var_.P_priori << "\033[0m" << std::endl;
}
void Fusion::update_inn(){
    eskf_var_.inn = eskf_var_.Z - eskf_var_.H * eskf_var_.x_priori;
    eskf_var_.inn(0) = eskf_var_.inn(0) * 6345500;
    eskf_var_.inn(1) = eskf_var_.inn(1) * 5874500;
    // eskf_var_.inn(8) = eskf_var_.inn(8) * 0.5;
    // std::cout << "\033[33m" << "eskf_var_.Z" << std::endl << eskf_var_.Z << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "eskf_var_.x_priori" << std::endl << eskf_var_.x_priori << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "eskf_var_.inn" << std::endl << eskf_var_.inn << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "eskf_var_.Z: " << std::endl << eskf_var_.Z(8) << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "H * x_priori: " << std::endl << (eskf_var_.H * eskf_var_.x_priori)(8) << "\033[0m" << std::endl;
}
void Fusion::update_S_hat_window(){
    Eigen::MatrixXd tmp = eskf_var_.inn * eskf_var_.inn.transpose();
    if(eskf_var_.S_hat_window.size() < eskf_config_.MNC_window){
        eskf_var_.S_hat_window.push_back(tmp);
    }
    else if (eskf_var_.S_hat_window.size() == eskf_config_.MNC_window){
        eskf_var_.S_hat_window.push_back(tmp);
        eskf_var_.S_hat_window.erase(eskf_var_.S_hat_window.begin());
    }
}
void Fusion::update_S_hat(){
    Eigen::MatrixXd tmp = Eigen::MatrixXd::Zero(eskf_var_.S_hat.rows(),eskf_var_.S_hat.cols());
    double weight = 0;
    double a = eskf_config_.MNC_faing_rate;
    if(eskf_var_.S_hat_window.size() < eskf_config_.MNC_window){
        for(int i = 0; i < eskf_var_.S_hat_window.size(); i++){
            weight = pow(a,eskf_var_.S_hat_window.size()-1-i) * (1-a) / (1 - pow(a,eskf_var_.S_hat_window.size()));
            tmp = tmp + weight * eskf_var_.S_hat_window[i];
        }
    }
    else if (eskf_var_.S_hat_window.size() == eskf_config_.MNC_window){
        for(int i = 0; i < eskf_var_.S_hat_window.size(); i++){
            weight = pow(a,eskf_config_.MNC_window-1-i) * (1-a) / (1 - pow(a,eskf_config_.MNC_window));
            tmp = tmp + weight * eskf_var_.S_hat_window[i];
        }
    }
    eskf_var_.S_hat = tmp;
    // std::cout << "\033[33m" << "eskf_var_.inn" << std::endl << eskf_var_.inn << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "eskf_var_.S_hat" << std::endl << eskf_var_.S_hat << "\033[0m" << std::endl;
}
void Fusion::update_R_hat(){
    // eskf_var_.R_hat = (eskf_var_.S_hat - eskf_var_.H * eskf_var_.P_priori * eskf_var_.H.transpose());
    // std::cout << "\033[33m" << "eskf_var_.R_hat S_hat" << std::endl << eskf_var_.R_hat << "\033[0m" << std::endl; 
    eskf_var_.R_hat = (eskf_var_.inn * eskf_var_.inn.transpose() - eskf_var_.H * eskf_var_.P_priori * eskf_var_.H.transpose());
    // std::cout << "\033[33m" << "eskf_var_.R_hat inn*inn" << std::endl << eskf_var_.R_hat << "\033[0m" << std::endl;
    eskf_var_.R_hat(2,2) = eskf_var_.R(2,2);
    eskf_var_.R_hat(5,5) = eskf_var_.R(5,5);
    eskf_var_.R_hat(6,6) = eskf_var_.R(6,6);
    eskf_var_.R_hat(7,7) = eskf_var_.R(7,7);
    for(int i = 0; i < eskf_var_.R_hat.rows(); i++){
        if(eskf_var_.R_hat(i,i) > 4*eskf_var_.P_priori(i,i)){
            eskf_var_.R_hat(i,i) = 4*eskf_var_.P_priori(i,i);
        }
        else if(eskf_var_.R_hat(i,i) < 0){
            eskf_var_.R_hat(i,i) = 0;
        }
    }
    // std::cout << "\033[33m" << "eskf_var_.inn*inn" << std::endl << eskf_var_.inn * eskf_var_.inn.transpose() << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "eskf_var_.R_hat" << std::endl << eskf_var_.R_hat << "\033[0m" << std::endl;
}
void Fusion::update_r_k(){
    Eigen::MatrixXd S = eskf_var_.H * eskf_var_.P_priori * eskf_var_.H.transpose() + eskf_var_.R;
    // std::cout << "\033[33m" << "eskf_var_.S" << std::endl << S << "\033[0m" << std::endl;
    double S_hat_sum = 0;
    double S_sum = 0;
    for(int i = 0; i < 9; i++){
        if(i == 2 || i == 5 || i == 6 || i == 7)
            continue;
        S_hat_sum = S_hat_sum + eskf_var_.S_hat(i,i);
        S_sum = S_sum + S(i,i);
    }
    eskf_var_.r_k = abs(S_hat_sum/S_sum - 1);
    // std::cout << "\033[33m" << "eskf_var_.S_hat_sum" << std::endl << S_hat_sum << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "eskf_var_.S_sum" << std::endl << S_sum << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "eskf_var_.r_k" << std::endl << eskf_var_.r_k << "\033[0m" << std::endl;
}
void Fusion::update_s(){
    static int k_s_counting = 1;
    if(k_s_counting <= eskf_config_.MICW_threshold_count){
        eskf_var_.s = 1;
        k_s_counting++;
    }
    else if(k_s_counting > eskf_config_.MICW_threshold_count){
        eskf_var_.s = (0.8 + eskf_var_.r_k) > 2 ? 2 : (0.8 + eskf_var_.r_k);
    }
    // std::cout << "\033[33m" << "eskf_var_.s" << std::endl << eskf_var_.s << "\033[0m" << std::endl;
}
void Fusion::update_d(){
    static int k_d_counting = 1;
    double lamda = eskf_config_.MICW_lamda;
    double b = eskf_config_.MICW_forgetting_factor;
    eskf_var_.d = (lamda - b)/(lamda - pow(b, k_d_counting+1));
    if(k_d_counting < 10000)
        k_d_counting++; 
}
void Fusion::update_K(){
    eskf_var_.K = eskf_var_.P_priori * eskf_var_.H.transpose() * (eskf_var_.H * eskf_var_.P_priori * eskf_var_.H.transpose() + eskf_var_.R).inverse();
    // std::cout << "\033[33m" << "P(-)HT" << std::endl << eskf_var_.P_priori * eskf_var_.H.transpose() << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "(HPH+R)-1" << std::endl << (eskf_var_.H * eskf_var_.P_priori * eskf_var_.H.transpose() + eskf_var_.R).inverse() << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "(P(-)+R).-1" << std::endl << (eskf_var_.H * eskf_var_.P_priori * eskf_var_.H.transpose() + eskf_var_.R).inverse() << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "eskf_var_.K" << std::endl << eskf_var_.K << "\033[0m" << std::endl;
}
void Fusion::update_x(){
    eskf_var_.x = eskf_var_.x_priori + eskf_var_.K * (eskf_var_.Z - eskf_var_.H * eskf_var_.x_priori);
    // std::cout << "\033[33m" << "eskf_var_.x" << std::endl << eskf_var_.x << "\033[0m" << std::endl;
}
void Fusion::update_P(){
    eskf_var_.P = (Eigen::MatrixXd::Identity(15,15) - eskf_var_.K * eskf_var_.H) * eskf_var_.P_priori;
    // std::cout << "\033[33m" << "eskf_var_.P" << std::endl << eskf_var_.P << "\033[0m" << std::endl;
}
void Fusion::Prediction(){
    update_x_priori();
    update_P_priori();
}
void Fusion::MNC_estimate(){
    Eigen::MatrixXd R_k;
    update_inn();
    update_S_hat_window();
    update_S_hat();
    update_R_hat();
    update_r_k();
    update_s();
    update_d();
    std::cout << "\033[33m" << "eskf_var_.s" << std::endl << eskf_var_.s << "\033[0m" << std::endl;
    std::cout << "\033[33m" << "eskf_var_.d" << std::endl << eskf_var_.d << "\033[0m" << std::endl;
    R_k = (1 - eskf_var_.s*eskf_var_.d) * eskf_var_.R + eskf_var_.s*eskf_var_.d * eskf_var_.R_hat;

    // If last change of heading over the threshold, next R of heading is forced to be small
    static double heading = eskf_var_.inn(8);
    if(abs(heading) > 0.7){
        R_k(8,8) = 0.1 * eskf_var_.P_priori(8,8);
    }
    heading = eskf_var_.inn(8);
    //
    // R must higher than minimum threshold
    for(int i = 0; i < 8; i++){
        if(R_k(i,i) < 0.01*eskf_var_.P_priori(i,i)){
            R_k(i,i) == 0.01*eskf_var_.P_priori(i,i);
        }
    }
    std::cout << "\033[33m" << "R_k" << std::endl << R_k << "\033[0m" << std::endl;
    Eigen::Vector3d R_pos(R_k(0, 0), R_k(1, 1), R_k(2, 2));
    Eigen::Vector3d R_vel(R_k(3, 3), R_k(4, 4), R_k(5, 5));
    Eigen::Vector3d R_att(R_k(6, 6), R_k(7, 7), R_k(8, 8));
    R_pos_ = R_pos;
    R_vel_ = R_vel;
    R_att_ = R_att;
    update_R();
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
    MNC_estimate();
    Update();
}

void Fusion::send_tf(){
    tf::Transform transform;
    tf::Quaternion current_q;
    Eigen::Vector3d nckuee(NCKUEE_LATITUDE, NCKUEE_LONGITUDE, NCKUEE_HEIGHT);
    Eigen::Vector3d now_lla(ins_fix_.latitude + eskf_var_.x(0)*RAD_TO_DEG, 
                            ins_fix_.longitude + eskf_var_.x(1)*RAD_TO_DEG, 
                            ins_fix_.altitude + eskf_var_.x(2));
    Eigen::Vector3d now_att(ins_fix_.att_e*DEG_TO_RAD + eskf_var_.x(6), 
                            ins_fix_.att_n*DEG_TO_RAD + eskf_var_.x(7), 
                            ins_fix_.att_u*DEG_TO_RAD + eskf_var_.x(8));
    ros::Time now = ros::Time::now();
    
    Eigen::Vector3d now_enu = coordinate_mat_transformation::lla2enu(now_lla, nckuee);
    // std::cout << "\033[33m" << "ENU:" << std::endl << now_enu << "\033[0m" << std::endl;
    // std::cout << "\033[33m" << "yaw: " << -state_vector_.att_l(2) << "\033[0m" << std::endl;

    current_q.setRPY(now_att(0), now_att(1), now_att(2)); // refer to "map" frame (ENU)

    transform.setOrigin(tf::Vector3(now_enu(0), now_enu(1), now_enu(2)));
    transform.setRotation(current_q);
    br_.sendTransform(tf::StampedTransform(transform, now, "/map", "eskf"));
}
void Fusion::send_tf(Eigen::Vector3d now_lla, Eigen::Vector3d now_att, std::string frame){ //lla(deg deg m) att(deg, deg, deg)
    tf::Transform transform;
    tf::Quaternion current_q;
    Eigen::Vector3d nckuee(NCKUEE_LATITUDE, NCKUEE_LONGITUDE, NCKUEE_HEIGHT);
    ros::Time now = ros::Time::now();
    
    Eigen::Vector3d now_enu = coordinate_mat_transformation::lla2enu(now_lla, nckuee);
    // std::cout << "\033[33m" << "ENU:" << std::endl << now_enu << "\033[0m" << std::endl;
    now_att = now_att*DEG_TO_RAD;
    current_q.setRPY(now_att(0), now_att(1), now_att(2)); // refer to "map" frame (ENU)

    transform.setOrigin(tf::Vector3(now_enu(0), now_enu(1), now_enu(2)));
    transform.setRotation(current_q);
    br_.sendTransform(tf::StampedTransform(transform, now, "/map", frame));
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
    // 0 ~ 360 -> -180 ~ 180
    Eigen::Vector3d att(ins_fix_.att_e + eskf_var_.x(6)*RAD_TO_DEG, 
                        ins_fix_.att_n + eskf_var_.x(7)*RAD_TO_DEG, 
                        ins_fix_.att_u + eskf_var_.x(8)*RAD_TO_DEG);
    for (int i = 0; i < 3; i++){
        if (att(i) > 180 && att(i) < 360){
            att(i) = att(i) - 360;
        }
    }
    msg.att_e = att(0);
    msg.att_n = att(1);
    msg.att_u = att(2);

    for(int i = 0; i < 9; i++){
        msg.R_covariance.push_back(eskf_var_.R(i,i));
        msg.P_covariance.push_back(eskf_var_.P(i,i));
    }
    pub_fusion_.publish(msg);
    // std::cout << "\033[33m" << "attitude" << std::endl << msg.att_u << "\033[0m" << std::endl;
}

void Fusion::publish_ins(){
    uwb_ins_eskf_msgs::fusionFIX msg;
    ros::Time now = ros::Time::now();

    msg.header.stamp = now;
    msg.header.frame_id = "local";
    msg.latitude = ins_fix_.latitude;
    msg.longitude = ins_fix_.longitude;
    msg.altitude = ins_fix_.altitude;
    msg.velocity_e = ins_fix_.velocity_e;
    msg.velocity_n = ins_fix_.velocity_n;
    msg.velocity_u = ins_fix_.velocity_u;
    msg.att_e = ins_fix_.att_e;
    msg.att_n = ins_fix_.att_n;
    msg.att_u = ins_fix_.att_u;

    pub_fusion_.publish(msg);
    // std::cout << "\033[33m" << "attitude" << std::endl << msg.att_u << "\033[0m" << std::endl;
}