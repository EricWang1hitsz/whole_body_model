#include <WholeBodyKinematic.h>
#include <RigidBodyDynamics.h>

#include <iostream>
#include <iomanip>

using namespace std;
using namespace wbc;

int main(int argc, char **argv)
{
    WholeBodyKinematic wbkn;

    string urdf_file = "/home/eric/catkin_ws_eric/src/whole_body_model/test/hyq.urdf";

    wbkn.modelFromURDFModel(urdf_file, true);

    rbd::Vector6d base_position;
    base_position << 0, 0, 0,
                     0, 0, 0;

    Eigen::VectorXd joint_position(12);
    joint_position << 0, 0.75, -1.5,
                      0, -0.75, 1.5,
                      0, 0.75, -1.5,
                      0, -0.75, 1.5;

    rbd::Vector6d base_velocity;
    base_velocity << 0, 0, 0,
                     0, 0, 0;

    Eigen::VectorXd joint_velocity(12);
    joint_velocity << 0, 0, 0,
                      0, 0, 0,
                      0, 0, 0,
                      0, 0, 0;

    rbd::Vector6d base_acceleration;
    base_acceleration << 0, 0, 0,
                         0, 0, 0;

    Eigen::VectorXd joint_acceleration(12);
    joint_acceleration << 0, 0, 0,
                      0, 0, 0,
                      0, 0, 0,
                      0, 0, 0;

    rbd::BodySelector limbs_(4);
    limbs_.push_back("lf_foot");
    limbs_.push_back("lh_foot");
    limbs_.push_back("rf_foot");
    limbs_.push_back("rh_foot");

    rbd::BodyVectorXd foot_velocity;
    wbkn.computeVelocity(foot_velocity,
                         base_position, joint_position,
                         base_velocity, joint_velocity,
                         limbs_);
    std::cout << "LF foot velocity: " << foot_velocity.find("lf_foot")->second.transpose() << std::endl;
    std::cout << "LH foot velocity: " << foot_velocity.find("lh_foot")->second.transpose() << std::endl;
    std::cout << "RF foot velocity: " << foot_velocity.find("rf_foot")->second.transpose() << std::endl;
    std::cout << "RH foot velocity: " << foot_velocity.find("rh_foot")->second.transpose() << std::endl;

    rbd::BodyVectorXd foot_acceleration;
    wbkn.computeAcceleration(foot_acceleration,
                             base_position, joint_position,
                             base_velocity, joint_velocity,
                             base_acceleration, joint_acceleration,
                             limbs_);
    std::cout << "LF foot acceleration: " << foot_acceleration.find("lf_foot")->second.transpose() << std::endl;
    std::cout << "LH foot acceleration: " << foot_acceleration.find("lh_foot")->second.transpose() << std::endl;
    std::cout << "RF foot acceleration: " << foot_acceleration.find("rf_foot")->second.transpose() << std::endl;
    std::cout << "RH foot acceleration: " << foot_acceleration.find("rh_foot")->second.transpose() << std::endl;

    Eigen::MatrixXd jacobian;
    wbkn.computeJacobian(jacobian,
                         base_position, joint_position,
                         limbs_);
    std::cout << fixed << setprecision(3) << std::endl;
    std::cout << jacobian << std::endl;

    rbd::BodyVectorXd jacd_qd;
    wbkn.computeJdotQdot(jacd_qd,
                         base_position, joint_position,
                         base_velocity, joint_velocity,
                         limbs_);
    std::cout << "LF foot JdQd: " << jacd_qd.find("lf_foot")->second.transpose() << std::endl;
    std::cout << "LH foot JdQd: " << jacd_qd.find("lh_foot")->second.transpose() << std::endl;
    std::cout << "RF foot JdQd: " << jacd_qd.find("rf_foot")->second.transpose() << std::endl;
    std::cout << "RH foot JdQd: " << jacd_qd.find("rh_foot")->second.transpose() << std::endl;


}
