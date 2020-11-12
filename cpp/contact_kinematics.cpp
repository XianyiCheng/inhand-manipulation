#include"contact_kinematics.h"

Matrix6d contact_jacobian(const Vector3d &position, const Vector3d &normal){
    Vector3d z(0,0,1);
    Vector3d rotvec = z.cross(normal);
    Matrix3d R;

    if (rotvec.norm() < 1e-6){    
        R.setIdentity();
    }
    else{
        rotvec = rotvec*(1/rotvec.norm());
        double rotangle = acos(z.dot(normal)/normal.norm());
        AngleAxisd aaxis(rotangle, rotvec);
        R = aaxis.toRotationMatrix();
    }
    Matrix4d T = Matrix4d::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = position;
    Matrix6d Adgco = SE32Adj(T);
    return Adgco;
}