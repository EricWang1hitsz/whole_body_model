#include <WholeBodyDynamic.h>
#include <RigidBodyDynamics.h>

#include <iostream>
#include <iomanip>

using namespace std;
using namespace dwl;

int main(int argc, char **argv)
{
    WholeBodyDynamic wbdy;

    string urdf_file = "/home/eric/catkin_ws_eric/src/whole_body_model/test/hyq.urdf";

    wbdy.modelFromURDFModel(urdf_file, true);

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

    rbd::BodyVector6d contact_force_; // ground reaction force.
    contact_force_["lf_foot"] << 0, 0, 0, 0, 0, 190.778;
    contact_force_["rf_foot"] << 0, 0, 0, 0, 0, 190.778;
    contact_force_["lh_foot"] << 0, 0, 0, 0, 0, 190.778;
    contact_force_["rh_foot"] << 0, 0, 0, 0, 0, 190.778;

    Eigen::MatrixXd joint_space_inertia_;
    joint_space_inertia_ = wbdy.computeJointSpaceInertiaMatrix(base_position, joint_position);

    std::cout << fixed << setprecision(3) << std::endl;
    std::cout << std::endl <<"Joint Space Inertia " << std::endl;
    std::cout << joint_space_inertia_ << std::endl;

    Eigen::VectorXd nonlinear_effects_force_;
    nonlinear_effects_force_ = wbdy.computeNonlinearEffectsForce(base_position, joint_position,
                                                                   base_velocity, joint_velocity);
    std::cout << std::endl << "Nonlinear effects force: " << std::endl;
    std::cout << nonlinear_effects_force_ << std::endl;

    Eigen::VectorXd joint_force_(12);
    wbdy.computeFloatingBaseInverseDynamics(base_acceleration, joint_force_,
                                            base_position, joint_position,
                                            base_velocity, joint_velocity,
                                            joint_acceleration, contact_force_);
    std::cout << std::endl << "Joint force by ID: " << std::endl;
    std::cout << joint_force_ << std::endl;

}
