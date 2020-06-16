#include <WholeBodyKinematic.h>

namespace dwl {

WholeBodyKinematic::WholeBodyKinematic()
{
    systemDOF = 18;
    jointDOF = 12;
    std::cout << "Initializing WholeBodyKinematic Class " << std::endl;
}

WholeBodyKinematic::~WholeBodyKinematic()
{
    std::cout << "Deconstructed WholeBodyKinematic Class " << std::endl;
}


void WholeBodyKinematic::modelFromURDFModel(std::string &urdf_model, bool info)
{
    char* urdf_model_ = (char*)urdf_model.c_str();
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_model_, &rbd, true, false);

    rbd::getListOfBodies(body_id_, rbd);

    if(info)
        rbd::printModelInfo(rbd);
}

int WholeBodyKinematic::getNumberOfActiveEndEffectors(const rbd::BodySelector &body_set)
{
    int num_body_set = 0;
    for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
            body_iter != body_set.end();
            body_iter++)
    {
        std::string body_name = *body_iter;
        if (body_id_.count(body_name) > 0) {
            ++num_body_set;
        } else
            printf(YELLOW "WARNING: The %s link is not an end-effector\n"
                    COLOR_RESET, body_name.c_str());
    }

    return num_body_set;
}

const Eigen::VectorXd& WholeBodyKinematic::toGeneralizedJointState(const rbd::Vector6d& base_state,
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

void WholeBodyKinematic::computeJacobian(Eigen::MatrixXd &jacobian,
                                         const rbd::Vector6d &base_pos,
                                         const Eigen::VectorXd &joint_pos,
                                         const rbd::BodySelector &body_set)
{
    // Only think about linear part
    int num_vars = 3;
    int num_body_set = getNumberOfActiveEndEffectors(body_set);
    jacobian.resize(num_vars * num_body_set, systemDOF);
    // check jacobian matrix rows and cols.
    std::cout << "Full Jacobian Matrix Size:" << jacobian.rows() << "x" << jacobian.cols() << std::endl;
    jacobian.setZero();

    int body_counter = 0;
    for(rbd::BodySelector::const_iterator body_iter = body_set.begin();
        body_iter != body_set.end();
        body_iter++)
    {
        int init_row = body_counter * num_vars;

        std::string body_name = *body_iter;
        if(body_id_.count(body_name) > 0)
        {
            int body_id = body_id_.find(body_name)->second;

            Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);
            Eigen::MatrixXd jac(Eigen::MatrixXd::Zero(6, systemDOF));

            // body_point_position is coordinates of the point in body coordinates, so set zero
            if (body_counter == 0) {
                rbd::computePointJacobian(rbd,
                                          q, body_id,
                                          Eigen::Vector3d::Zero(),
                                          jac, true);
            } else {
                rbd::computePointJacobian(rbd,
                                          q, body_id,
                                          Eigen::Vector3d::Zero(),
                                          jac, false);
            }
            // Extract linear part from full jacobian
            jacobian.block(init_row, 0, num_vars, systemDOF) =
                    jac.block(3, 0, 3, systemDOF);

            ++body_counter;
        }
    }


}

void WholeBodyKinematic::computeVelocity(rbd::BodyVectorXd &op_vel,
                                         const rbd::Vector6d &base_pos,
                                         const Eigen::VectorXd &joint_pos,
                                         const rbd::Vector6d &base_vel,
                                         const Eigen::VectorXd &joint_vel,
                                         const rbd::BodySelector &body_set)
{
    int num_vars = 6;

    Eigen::VectorXd body_vel(num_vars);

    // Adding the velocity only for the active end-effectors
    for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
            body_iter != body_set.end();
            body_iter++)
    {
        std::string body_name = *body_iter;
        if(body_id_.count(body_name) > 0)
        {
            int body_id = body_id_.find(body_name)->second;

            Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);
            Eigen::VectorXd q_dot = toGeneralizedJointState(base_vel, joint_vel);
            rbd::Vector6d point_vel =
                    rbd::computePointVelocity(rbd, q, q_dot, body_id,
                                              Eigen::Vector3d::Zero(), true);
            body_vel.segment<6>(0) = point_vel;

            op_vel[body_name] = body_vel;

        }
    }

}

void WholeBodyKinematic::computeAcceleration(rbd::BodyVectorXd &op_acc,
                                             const rbd::Vector6d &base_pos,
                                             const Eigen::VectorXd &joint_pos,
                                             const rbd::Vector6d &base_vel,
                                             const Eigen::VectorXd &joint_vel,
                                             const rbd::Vector6d &base_acc,
                                             const Eigen::VectorXd &joint_acc,
                                             const rbd::BodySelector &body_set)
{   // Only think about linear part
    int num_vars = 3;

    Eigen::VectorXd body_acc(num_vars);

    for(rbd::BodySelector::const_iterator body_iter = body_set.begin();
        body_iter != body_set.end();
        body_iter++)
    {
        std::string body_name = *body_iter;
        if(body_id_.count(body_name) > 0)
        {
            unsigned int body_id = body_id_.find(body_name)->second;

            Eigen::VectorXd q = toGeneralizedJointState(base_pos, joint_pos);
            Eigen::VectorXd q_dot = toGeneralizedJointState(base_vel, joint_vel);
            Eigen::VectorXd q_ddot = toGeneralizedJointState(base_acc, joint_acc);

            // Computing the point acceleration
            rbd::Vector6d point_acc =
                    rbd::computePointAcceleration(rbd,
                                                  q, q_dot, q_ddot,
                                                  body_id,
                                                  Eigen::Vector3d::Zero(), true);
            body_acc.segment<3>(0) = rbd::linearPart(point_acc);

            op_acc[body_name] = body_acc;
        }
    }
}

const rbd::BodyVectorXd& WholeBodyKinematic::computeAcceleration(const rbd::Vector6d& base_pos,
                                                                  const Eigen::VectorXd& joint_pos,
                                                                  const rbd::Vector6d& base_vel,
                                                                  const Eigen::VectorXd& joint_vel,
                                                                  const rbd::Vector6d& base_acc,
                                                                  const Eigen::VectorXd& joint_acc,
                                                                  const rbd::BodySelector& body_set)
{
    computeAcceleration(body_acc_,
                    base_pos, joint_pos,
                    base_vel, joint_vel,
                    base_acc, joint_acc,
                    body_set);
    return body_acc_;
}

void WholeBodyKinematic::computeJdotQdot(rbd::BodyVectorXd &jacd_qd,
                                         const rbd::Vector6d &base_pos,
                                         const Eigen::VectorXd &joint_pos,
                                         const rbd::Vector6d &base_vel,
                                         const Eigen::VectorXd &joint_vel,
                                         const rbd::BodySelector &body_set)
{
    rbd::BodyVectorXd op_vel, op_acc;
    computeAcceleration(op_acc,
                        base_pos, joint_pos,
                        base_vel, joint_vel,
                        rbd::Vector6d::Zero(),
                        Eigen::VectorXd::Zero(jointDOF),
                        body_set);

    int num_vars = 3;

    computeVelocity(op_vel,
                    base_pos, joint_pos,
                    base_vel, joint_vel,
                    body_set);

    Eigen::VectorXd body_jacd_qd(num_vars);

    for (rbd::BodySelector::const_iterator body_iter = body_set.begin();
            body_iter != body_set.end();
            body_iter++)
    {
        std::string body_name = *body_iter;
        if(body_id_.count(body_name) > 0)
        {
            rbd::Vector6d point_vel = op_vel[body_name];
            Eigen::Vector3d ang_vel, lin_vel;
            ang_vel = rbd::angularPart(point_vel);
            lin_vel = rbd::linearPart(point_vel);

            // Computing the JdQd for current point
            body_jacd_qd.segment<3>(0) =
                    op_acc[body_name] + ang_vel.cross(lin_vel);

            jacd_qd[body_name] = body_jacd_qd;

        }
    }
}

const rbd::BodyVectorXd& WholeBodyKinematic::computeJdotQdot(const rbd::Vector6d& base_pos,
                                                              const Eigen::VectorXd& joint_pos,
                                                              const rbd::Vector6d& base_vel,
                                                              const Eigen::VectorXd& joint_vel,
                                                              const rbd::BodySelector& body_set)
{
    computeJdotQdot(jdot_qdot_,
                    base_pos, joint_pos,
                    base_vel, joint_vel,
                    body_set);
    return jdot_qdot_;
}

}
