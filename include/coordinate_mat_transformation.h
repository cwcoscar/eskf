#ifndef COORDINATE_MAT_TRANSFORMATION_H_
#define COORDINATE_MAT_TRANSFORMATION_H_
#include <Eigen/Dense>
#include <Eigen/Geometry> //Quaternion
#include <iostream>

// Geodetic system parameters
#define kSemimajorAxis ((double)6378137)
#define kSemiminorAxis ((double)6356752.3142)
#define kFirstEccentricitySquared ((double)6.69437999014*0.001)
#define kSecondEccentricitySquared ((double)6.73949674228*0.001)
#define kFlattening ((double)1/298.257223563)

namespace coordinate_mat_transformation
{
    // unit transform
    inline double rad2Deg(const double radians){return (radians / M_PI) * 180.0;}

    inline double deg2Rad(const double degrees){return (degrees / 180.0) * M_PI;}
    
    // vector -> skew symmetric matrix
    Eigen::MatrixXd Skew_symmetric_transform(const Eigen::VectorXd& vector, int dim){
        Eigen::MatrixXd result(dim,dim);
        if (dim == 3){
            result << 0,          -vector(2), vector(1),
                    vector(2),  0,          -vector(0),
                    -vector(1), vector(0),  0;
        }
        else if (dim == 4){
            result << 0,          vector(2),  -vector(1), vector(0),
                    -vector(2), 0,          vector(0),  vector(1),
                    vector(1),  -vector(0), 0,          vector(2),
                    -vector(0), -vector(1), -vector(2), 0;
        }
        else{
            std::cout << "The dimension of transformation is not supported." << std::endl;
        }
        return result;
    }

    // roll pitch yaw (rad) -> quaternion(x,y,z,w)
    Eigen::VectorXd att_Q_transform(Eigen::VectorXd& att){
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(att(0), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(att(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(att(2), Eigen::Vector3d::UnitZ());

        q.normalize(); // check and make it orthogonal
        return q.coeffs();
    }

    // quaternion(x,y,z,w) -> roll pitch yaw (rad)
    Eigen::VectorXd Q_att_transform(Eigen::VectorXd& quat){
        Eigen::Vector3d att(0,0,0);
        Eigen::Quaterniond q(quat(3), quat(0), quat(1), quat(2));
        q.normalize(); // check and make it orthogonal
        Eigen::Matrix3d Rotation_matrix = q.toRotationMatrix();

        att(0) = atan2(-Rotation_matrix(1,2), Rotation_matrix(2,2));//atan2(y,x)
        att(1) = asin(Rotation_matrix(0,2));
        att(2) = atan2(-Rotation_matrix(0,1), Rotation_matrix(0,0));
        return att;
    }

    // roll pitch yaw -> rotation matrix (XYZ)
    Eigen::Matrix3d Rotation_matrix(Eigen::VectorXd att){
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(att(0), Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(att(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(att(2), Eigen::Vector3d::UnitZ());
        return q.toRotationMatrix();
    }

    // lat, long, alt -> ecef
    Eigen::Vector3d lla2xyz(double L1, double L2, double H1){ //deg deg m
        double a, b, e;
        double sinphi, cosphi, coslam, sinlam, tan2phi;
        double tmp, tmpden, tmp2;
        Eigen::Vector3d xyzC;

        L1=(L1*M_PI)/180;
        L2=(L2*M_PI)/180;

        a = 6378137.0000;
        b = 6356752.3142;
        e = sqrt(1-(b/a)*(b/a));  

        sinphi = sin(L1);
        cosphi = cos(L1);
        coslam = cos(L2);
        sinlam = sin(L2);
        tan2phi = (tan(L1))*(tan(L1));
        tmp = 1 - e*e;
        tmpden = sqrt( 1 + tmp*tan2phi );

        xyzC(0) = (a*coslam)/tmpden + H1*coslam*cosphi;

        xyzC(1) = (a*sinlam)/tmpden + H1*sinlam*cosphi;

        tmp2 = sqrt(1 - e*e*sinphi*sinphi);
        xyzC(2) = (a*tmp*sinphi)/tmp2 + H1*sinphi;

        return (xyzC);
    }

    // lat, long, alt(deg deg m) -> enu(m)
    Eigen::Vector3d lla2enu(Eigen::Vector3d rover, Eigen::Vector3d base){
        Eigen::Vector3d xe2 = lla2xyz(base(0),base(1),base(2));
        Eigen::Vector3d xe1 = lla2xyz(rover(0),rover(1),rover(2));
        
        Eigen::Vector3d enuC;
        double a, b, c;
        double phi, lam, sinphi, cosphi, sinlam, coslam;

        a=xe1(0)-xe2(0);
        b=xe1(1)-xe2(1);
        c=xe1(2)-xe2(2);
        
        phi=(base(0)*M_PI)/180;
        lam=(base(1)*M_PI)/180;
        sinphi=sin(phi);
        cosphi=cos(phi);
        sinlam=sin(lam);
        coslam=cos(lam);

        enuC(0)=(-sinlam)*a+(coslam)*b+(0)*c;
        enuC(1)=(-sinphi*coslam)*a+(-sinphi*sinlam)*b+(cosphi)*c;
        enuC(2)=(cosphi*coslam)*a+(cosphi*sinlam)*b+(sinphi)*c;

        return (enuC);
    }

    // lat, long (rad, rad)
    inline Eigen::Matrix3d nRe(const double lat_radians, const double lon_radians){
        const double sLat = sin(lat_radians);
        const double sLon = sin(lon_radians);
        const double cLat = cos(lat_radians);
        const double cLon = cos(lon_radians);

        Eigen::Matrix3d ret;
        ret(0, 0) = -sLat * cLon;
        ret(0, 1) = -sLat * sLon;
        ret(0, 2) = cLat;
        ret(1, 0) = -sLon;
        ret(1, 1) = cLon;
        ret(1, 2) = 0.0;
        ret(2, 0) = cLat * cLon;
        ret(2, 1) = cLat * sLon;
        ret(2, 2) = sLat;

        return ret;
    }

    void geodetic2Ecef(const double latitude, const double longitude, const double altitude, double* x, double* y, double* z){
        // Convert geodetic coordinates to ECEF.
        // http://code.google.com/p/pysatel/source/browse/trunk/coord.py?r=22
        double lat_rad = deg2Rad(latitude);
        double lon_rad = deg2Rad(longitude);
        double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
        *x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
        *y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
        *z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
    }

    Eigen::Vector3d ecef2Geodetic(const double x, const double y, const double z)
    {
        // Convert ECEF coordinates to geodetic coordinates.
        // J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
        // to geodetic coordinates," IEEE Transactions on Aerospace and
        // Electronic Systems, vol. 30, pp. 957-961, 1994.
        Eigen::Vector3d ret;
        double r = sqrt(x * x + y * y);
        double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
        double F = 54 * kSemiminorAxis * kSemiminorAxis * z * z;
        double G = r * r + (1 - kFirstEccentricitySquared) * z * z - kFirstEccentricitySquared * Esq;
        double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / pow(G, 3);
        double S = cbrt(1 + C + sqrt(C * C + 2 * C));
        double P = F / (3 * pow((S + 1 / S + 1), 2) * G * G);
        double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
        double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q)
            + sqrt(
                0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q)
                    - P * (1 - kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
        double U = sqrt(pow((r - kFirstEccentricitySquared * r_0), 2) + z * z);
        double V = sqrt(
            pow((r - kFirstEccentricitySquared * r_0), 2) + (1 - kFirstEccentricitySquared) * z * z);
        double Z_0 = kSemiminorAxis * kSemiminorAxis * z / (kSemimajorAxis * V);
        ret(2) = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
        ret(0) = rad2Deg(atan((z + kSecondEccentricitySquared * Z_0) / r));
        ret(1) = rad2Deg(atan2(y, x));
        return ret;
    }

    Eigen::Vector3d ned2Ecef(const double north, const double east, const double down, const Eigen::Vector3d ref_lla){
        // NED (north/east/down) to ECEF coordinates
        // ref_lla in (deg, deg, m)
        Eigen::Vector3d ned, ret;
        ned(0) = north;
        ned(1) = east;
        ned(2) = -down;

        Eigen::Matrix3d ned_to_ecef_matrix_ = nRe(deg2Rad(ref_lla(0)), deg2Rad(ref_lla(1))).transpose();

        double initial_ecef_x_, initial_ecef_y_, initial_ecef_z_;
        geodetic2Ecef(ref_lla(0), ref_lla(1), ref_lla(2), &initial_ecef_x_, &initial_ecef_y_, &initial_ecef_z_);

        ret = ned_to_ecef_matrix_ * ned;
        ret(0) = ret(0) + initial_ecef_x_;
        ret(1) = ret(1) + initial_ecef_y_;
        ret(2) = ret(2) + initial_ecef_z_;
        return ret;
    }

    Eigen::Vector3d enu2Geodetic(const Eigen::Vector3d enu, const Eigen::Vector3d ref_lla){
        // Local ENU position to geodetic coordinates
        // ref_lla in (deg, deg, m)
        Eigen::Vector3d ecef = ned2Ecef(enu(1), enu(0), -enu(2), ref_lla);
        Eigen::Vector3d ret = ecef2Geodetic(ecef(0), ecef(1), ecef(2));
        return ret;
    }

}
#endif