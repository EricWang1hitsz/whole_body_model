#include <WholeBodyDynamic.h>

namespace dwl {

WholeBodyDynamic::WholeBodyDynamic()
{
    systemDOF = 18;
    jointDOF = 12;
    std::cout << "Initializing WholeBodyDynamic Class " << std::endl;
}

WholeBodyDynamic::~WholeBodyDynamic()
{
    std::cout << "Deconstructed WholeBodyKinematic Class " << std::endl;
}

void WholeBodyDynamic::modelFromURDFModel(const std::string &urdf_model, bool info)
{
    char* urdf_model_ = (char*)urdf_model.c_str();
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_model_, &rbd, true, false);

    rbd::getListOfBodies(body_id_, rbd);

    if(info)
        rbd::printModelInfo(rbd);

    joint_inertia_mat_.resize(systemDOF, systemDOF);
    joint_inertia_mat_.setZero();
}

const Eigen::VectorXd& WholeBodyDynamic::toGeneralizedJointState(const rbd::Vector6d& base_state,
                                               const Eigen::VectorXd& joint_state)
{
    rbd::Vector6d _base_state = base_state;

    // Must resize before add data
    full_state_.resize(systemDOF);
    full_state_ << rbd::linearPart(_base_state),
                   rbd::angularPart(_base_state),
                   joint_state;

    return full_state_;
}

void WholeBodyDynamic::fromGeneralizedJointState(rbd::Vector6d &base_state,
                                                 Eigen::VectorXd &joint_state,
                                                 const Eigen::VectorXd &generalized_state)
{
    joint_state.resize(jointDOF);

    base_state << generalized_state.segment<3>(rbd::LX),
                  generalized_state.segment<3>(rbd::AX);
    joint_state = generalized_state.segment(6, jointDOF);
}

void WholeBodyDynamic::convertAppliedExternalForces(std::vector<RigidBodyDynamics::Math::SpatialVector>& fext,
                                                     const rbd::BodyVector6d& ext_force,
                                                     const Eigen::VectorXd& q)
{
    // Computing the applied external spatial forces for every body
    fext.resize(rbd.mBodies.size());
    // Searching over the movable bodies
    for(unsigned int body_id = 0; body_id < rbd.mBodies.size(); body_id++)
    {
        std::string body_name = rbd.GetBodyName(body_id);

        if(ext_force.count(body_name) > 0)
        {
            // Converting the applied force to spatial force vector in base coordinates
            rbd::Vector6d force = ext_force.at(body_name);
            Eigen::Vector3d force_point =
                    RigidBodyDynamics::CalcBodyToBaseCoordinates(rbd, q, body_id, Eigen::Vector3d::Zero(), true);
            rbd::Vector6d spatial_force = rbd::convertPointForceToSpatialForce(force, force_point);

            fext.at(body_id) = spatial_force;
        }
        else
        {
            fext.at(body_id) = rbd::Vector6d::Zero();
        }
    }

    // Searching over the fixed bodies
    for (unsigned int it = 0; it < rbd.mFixedBodies.size(); it++) {
        unsigned int body_id = it + rbd.fixed_body_discriminator;
        std::string body_name = rbd.GetBodyName(body_id);

        if (ext_force.count(body_name) > 0) {
            // Converting the applied force to spatial force vector in
            // base coordinates
            rbd::Vector6d force = ext_force.at(body_name);
            Eigen::Vector3d force_point =
                    RigidBodyDynamics::CalcBodyToBaseCoordinates(rbd,
                                              q, body_id,
                                              Eigen::Vector3d::Zero(), true);
            rbd::Vector6d spatial_force =
                    rbd::convertPointForceToSpatialForce(force, force_point);

            unsigned parent_id = rbd.mFixedBodies[it].mMovableParent;
            fext.at(parent_id) += spatial_force;
        }
    }
}

const Eigen::MatrixXd& WholeBodyDynamic::computeJointSpaceInertiaMatrix(const rbd::Vector6d& base_pos,
                                                                         const Eigen::VectorXd& joint_pos)
{
    // Converting base and joint states to generalized joint states
    Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);

    // Computing the joint space inertia matrix using the Composite
    // Rigid Body Algorithm
    // \f$ M(q) \f$
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(rbd,
                                                   q, joint_inertia_mat_, true);

    return joint_inertia_mat_;
}

const Eigen::VectorXd& WholeBodyDynamic::computeNonlinearEffectsForce(const rbd::Vector6d& base_pos,
                                                                       const Eigen::VectorXd& joint_pos,
                                                                       const rbd::Vector6d& base_vel,
                                                                       const Eigen::VectorXd& joint_vel)
{
    Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);
    Eigen::VectorXd q_dot = toGeneralizedJointState(base_vel, joint_vel);
    nonliner_effects_ = Eigen::VectorXd::Zero(systemDOF);

//    std::vector<SpatialVector_t> fext;
    RigidBodyDynamics::NonlinearEffects(rbd, q, q_dot, nonliner_effects_, NULL);

    return nonliner_effects_;

}

void WholeBodyDynamic::computeFloatingBaseInverseDynamics(rbd::Vector6d &base_acc,
                                                          Eigen::VectorXd &joint_forces,
                                                          const rbd::Vector6d &base_pos,
                                                          const Eigen::VectorXd &joint_pos,
                                                          const rbd::Vector6d &base_vel,
                                                          const Eigen::VectorXd &joint_vel,
                                                          const Eigen::VectorXd &joint_acc,
                                                          const rbd::BodyVector6d &ext_force)
{
    // Setting the size of the joint forces vector
    joint_forces.resize(jointDOF);

    // Converting base and joint states to generalized joint states
    Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);
    Eigen::VectorXd q_dot = toGeneralizedJointState(base_vel, joint_vel);
    Eigen::VectorXd q_ddot = toGeneralizedJointState(base_acc, joint_acc);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(systemDOF);

    // Computing the applied external spatial forces for every body
    std::vector<SpatialVector_t> fext;
    convertAppliedExternalForces(fext, ext_force, q);

    RigidBodyDynamics::Math::SpatialVector base_ddot =
            RigidBodyDynamics::Math::SpatialVector(base_acc);
    rbd::FloatingBaseInverseDynamics(rbd, q, q_dot, q_ddot, base_ddot, tau, &fext);

    // Converting the base acceleration
    base_acc = base_ddot;

    // Converting the generalized joint forces to base wrench and joint forces
    rbd::Vector6d base_wrench;
    fromGeneralizedJointState(base_wrench, joint_forces, tau);

}

}
