#ifndef ESKF_H_
#define ESKF_H_

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <novatel_gps_msgs/Inspva.h>
#include <novatel_gps_msgs/Inscov.h>
#include <ublox_msgs/NavATT.h>
#include <uwb_ins_eskf_msgs/InsFIX.h>
#include <uwb_ins_eskf_msgs/uwbFIX.h>
#include <uwb_ins_eskf_msgs/fusionFIX.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Dense>

#define EARTH_SEMIMAJOR ((double)6378137) // meter
#define EARTH_ROTATION_RATE ((double)0.00007292115) // rad/s
#define FLATTENING ((double)1/298.25722563)
#define ECCENTRICITY sqrt(FLATTENING*(2-FLATTENING))
#define DEG_TO_RAD M_PI/180
#define RAD_TO_DEG 180/M_PI
#define NCKUEE_LATITUDE 22.99665875 //  degree
#define NCKUEE_LONGITUDE 120.222584889 //  degree
#define NCKUEE_HEIGHT 98.211 // meter
#define DEG_TO_RAD M_PI/180
#define RAD_TO_DEG 180/M_PI

namespace ESKF{

    typedef struct eskf_config{
        int fusion_type;
        int novatel_count = 0;
        bool ublox_fix_flag = false;
        bool ublox_vel_flag = false;
        bool ublox_att_flag = false;
        bool novatel_cov_flag = false;
        bool uwb_flag = false;
        bool ins_flag = false;
        int MNC_window = 20;
        double MNC_faing_rate = 0.99;
        int MICW_threshold_count = 20;
        double MICW_lamda = 1.1;
        double MICW_forgetting_factor = 0.99;
    }config;

    typedef struct Error_state{
        Eigen::VectorXd position; // deg, deg, m
        Eigen::VectorXd velocity; // m/s
        Eigen::VectorXd attitude; // deg
        Eigen::VectorXd gyroscope; // rad/s2
        Eigen::VectorXd accelerometer; // m/s2
    }error_state;

    typedef struct ESKF_variable{
        /*"denote"
        .T: transpose , .-1: inverse , .tr:sum of diagonal elements , (-): priori , (+): posteriori
        Unit: rad / meter
        "Formula"
        .
        x = Fx+Gw , E(ww.T) = Q
        Z = Hx+v , E(vv.T) = R
        */
        Eigen::MatrixXd F; //state transition matrix
        Eigen::VectorXd x; // x_k(+) : error state vector
        Eigen::MatrixXd G; //process noise coefficient matrix
        Eigen::MatrixXd Q; //process noise covariance matrix
        Eigen::VectorXd Z; //measurement vector
        Eigen::MatrixXd H; //measurement coefficient matrix
        Eigen::MatrixXd R; //measurement noise covariance matrix

        /* "Prediction"
        x_k(-) = Theta * x_k-1(+)
        P_k(-) = Theta * P_k-1(+) * Theta.T + G_k-1 * Q_k-1 * G_k-1.T
        */
        Eigen::MatrixXd x_priori; // x_k(-) : priori of error state vector
        Eigen::MatrixXd Theta;
        Eigen::MatrixXd P_priori; // P_k(-) : priori of error covariance

        /* "Measurement Noise Covariance Estimate"
        inn = Z_k - H_k * x_k(-)
        S_hat = sum(weight * inn * inn.T)
        S = H*P_k(-)*H.T + R(-)
        r_k = |S_hat.tr / S.tr - 1|
        R_hat  = inn * inn.T - H*P_k(-)*H.T
        R(+) = (1 - s*d) * R(-) + s*d * R_hat
        */
        Eigen::VectorXd inn; // inn: innovation
        Eigen::MatrixXd S_hat;
        std::vector<Eigen::MatrixXd> S_hat_window;
        double r_k;
        Eigen::MatrixXd R_hat;
        double s;
        double d;

        /* "Update"
        K_k = P_k(-) * H_k.T * (H_k * P_k(-) * H_k.T).-1
        x_k(+) = x_k(-) + K_k * inn
        P_k(+) = (I - K_k * H_k) * P_k(-)
        */
        Eigen::MatrixXd K; // K_k : Kalman gain
        Eigen::MatrixXd P; // P_k(+) : posteriori of error covariance


    }eskf_variable;

    class Fusion{
        private:
            error_state err_state_;
            eskf_variable eskf_var_;
            config eskf_config_;
            ros::Publisher pub_fusion_;
            tf::TransformBroadcaster br_;
            sensor_msgs::NavSatFix ublox_fix_;
            geometry_msgs::TwistWithCovarianceStamped ublox_vel_;
            ublox_msgs::NavATT ublox_att_;
            double time_interval_ = 1;
            uwb_ins_eskf_msgs::uwbFIX uwb_fix_;
            uwb_ins_eskf_msgs::InsFIX ins_fix_;
            novatel_gps_msgs::Inspva novatel_fix_;
            Eigen::Vector3d R_pos_;
            Eigen::Vector3d R_vel_;
            Eigen::Vector3d R_att_;
        public:
            // constructor
            Fusion(ros::Publisher pub_fusion, ESKF::config eskf_config);

            // Initialize
            void Initilize_error_state();
            void Initilize_eskf_variable();

            // update data
            void ubloxFIXcallback(const sensor_msgs::NavSatFix& msg);
            void ubloxVELcallback(const geometry_msgs::TwistWithCovarianceStamped& msg);
            void ubloxATTcallback(const ublox_msgs::NavATT& msg);
            void novatelINSPVAcallback(const novatel_gps_msgs::Inspva& msg);
            void novatelINSCOVcallback(const novatel_gps_msgs::Inscov& msg);
            void uwbFIXcallback(const uwb_ins_eskf_msgs::uwbFIX& msg);
            void insFIXcallback(const uwb_ins_eskf_msgs::InsFIX& msg);
            void update_error_state();

            // algorithm
            void update_F();
            void update_x();
            void update_Q();
            void update_Z();
            void update_R();
            void update_Theta();
            void update_x_priori();
            void update_P_priori();
            void update_inn();
            void update_S_hat_window();
            void update_S_hat();
            void update_R_hat();
            void update_r_k();
            void update_s();
            void update_d();
            void update_K();
            void update_P();
            void Prediction();
            void MNC_estimate();
            void Update();
            void KF_algorithm();

            // Result
            void send_tf();
            void send_tf(Eigen::Vector3d now_lla, Eigen::Vector3d now_att, std::string frame);
            void publish_fusion();
            void publish_ins();
    };

}
#endif