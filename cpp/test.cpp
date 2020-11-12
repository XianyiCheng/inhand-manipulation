#include "utilities.h"
#include "sample.h"
#include "contact_kinematics.h"

const static double PI = 3.1415926;

void test_uniform_quaternion(){
    printf("test uniform quaternion: ");
    Quaterniond q = generate_unit_quaternion();
    std::cout << q.w() << ", " << q.x() << ", "<< q.y() << ", " << q.x() << std::endl;
}

void test_sample_position(){
    printf("test sample position: ");
    Vector3d ub(1,1,2);
    Vector3d lb(-1,0,-2);
    Vector3d p;
    p = sample_position(ub, lb);
    std::cout << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
}

void test_steer_position(){
    printf("test steer position: \n");
    Vector3d ub(1,1,2);
    Vector3d lb(-1,0,-2);
    Vector3d p0;
    Vector3d p1;
    p0 = sample_position(ub, lb);
    p1 = sample_position(ub, lb);
    Vector3d p = steer_position(p0, p1, 0.5);
    std::cout << "p0: "<< p0[0] << ", " << p0[1] << ", " << p0[2] << std::endl;
    std::cout << "p1: "<< p1[0] << ", " << p1[1] << ", " << p1[2] << std::endl;
    std::cout << "p: "<< p[0] << ", " << p[1] << ", " << p[2] << std::endl;
}

void test_steer_orientation(){
    printf("test steer orientation: \n");
    Quaterniond q0(1,0,0,0);
    std::cout << q0.normalized().toRotationMatrix() << std::endl;
    Quaterniond q1 = generate_unit_quaternion();
    Quaterniond q = steer_quaternion(q0,q1, PI/4);
    std::cout << "q0: " << q0.w() << ", " << q0.x() << ", "<< q0.y() << ", " << q0.x() << std::endl;
    std::cout << "q1: " << q1.w() << ", " << q1.x() << ", "<< q1.y() << ", " << q1.x() << std::endl;
    std::cout << "q: " << q.w() << ", " << q.x() << ", "<< q.y() << ", " << q.x() << std::endl;
    std::cout << "max angle: " << PI/4 << ", angle between q0, q: " << angBTquat(q0, q) << std::endl;
}

void test_contact_jacobian(){
    printf("test contact jacobian: \n");
    Vector3d p(1,2,3);
    Vector3d n(0,0,1);
    Matrix6d adgco = contact_jacobian(p,n);
    std::cout << adgco << std::endl;
}

int main(){
    set_rand_seed();
    test_uniform_quaternion();
    test_sample_position();
    test_steer_position();
    test_steer_orientation();
    test_contact_jacobian();
}