#include"contact_kinematics.h"
#include <math.h>

Matrix6d contact_jacobian(const Vector3d &position, const Vector3d &normal){
    Vector3d z(0,0,1);
    Vector3d rotvec = z.cross(normal);
    Matrix3d R;

    if (rotvec.norm() < 1e-6){    
        R.setIdentity();
    } else if ((rotvec.norm() < 1e-6) && (signbit(z[2]) != signbit(normal[2]))){
        R << -1,0,0,0,1,0,0,0,-1;
    }
    else{
        rotvec = rotvec*(1/rotvec.norm());
        double rotangle = acos(z.dot(normal)/normal.norm());
        AngleAxisd aaxis(rotangle, rotvec);
        R = aaxis.toRotationMatrix();
    }
    // Matrix4d T = Matrix4d::Identity();
    // T.block<3, 3>(0, 0) = R;
    // T.block<3, 1>(0, 3) = position;
    // Matrix6d Adgoc = SE32Adj(T);
    Matrix4d T_inv = Matrix4d::Identity();
    T_inv.block<3, 3>(0, 0) = R.transpose();
    T_inv.block<3, 1>(0, 3) = -R.transpose()*position;
    Matrix6d Adgco = SE32Adj(T_inv);
    return Adgco;
}