#include "utilities.h"
#include "sample.h"
#include <math.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
using namespace Eigen;



Vector3d sample_position(const Vector3d &ub, const Vector3d &lb){
    
    Vector3d t(randd(),randd(),randd());
    Vector3d q;
    q = t.cwiseProduct(ub - lb) + lb;
    return q;
}

Quaterniond generate_unit_quaternion(){
    double q_norm = 2.0;
    double qq[4];
    while (q_norm > 1){
        q_norm = 0;
        for (int i = 0; i<4; i++){
            qq[i] = randd()*2 - 1;
            q_norm += qq[i]*qq[i];
        }
        q_norm = sqrt(q_norm);
    }
    Quaterniond q(qq[0]/q_norm, qq[1]/q_norm, qq[2]/q_norm, qq[3]/q_norm);
    return q;
}

Vector3d steer_position(const Vector3d &p0, const Eigen::Vector3d &p1, const double &length){
    Vector3d dp;
    dp = p1 - p0;
    double dp_norm = dp.norm();
    Vector3d p;
    if (dp_norm > length){
        double t = length/dp_norm;
        p = p0 + t*dp;
    }
    else{
        p = p1;
    }
    return p;
    
}

Quaterniond steer_quaternion(const Quaterniond &q0, const Quaterniond &q1, const double &angle){

    double angle_bt_quat = angBTquat(q0, q1);
    Quaterniond q;
    if (angle_bt_quat > angle){
        double t = angle/angle_bt_quat;
        q = q0.slerp(t, q1);
    }
    else{
        q = q1;
    }
    return q;
}