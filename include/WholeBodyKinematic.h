#include <RigidBodyDynamics.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
namespace dwl

{

class WholeBodyKinematic
{
public:
    WholeBodyKinematic();

    ~WholeBodyKinematic();

    void modelFromURDFModel(std::string& urdf_model, bool info);

    int getNumberOfActiveEndEffectors(const rbd::BodySelector& body_set);

    const Eigen::VectorXd& toGeneralizedJointState(const rbd::Vector6d& base_state,
                                                   const Eigen::VectorXd& joint_state);

    void computeJacobian(Eigen::MatrixXd& jacobian,
                         const rbd::Vector6d& base_pos,
                         const Eigen::VectorXd& joint_pos,
                         const rbd::BodySelector& body_set);

    void computeVelocity(rbd::BodyVectorXd& op_vel,
                         const rbd::Vector6d& base_pos,
                         const Eigen::VectorXd& joint_pos,
                         const rbd::Vector6d& base_vel,
                         const Eigen::VectorXd& joint_vel,
                         const rbd::BodySelector& body_set);

    void computeAcceleration(rbd::BodyVectorXd& op_acc,
                             const rbd::Vector6d& base_pos,
                             const Eigen::VectorXd& joint_pos,
                             const rbd::Vector6d& base_vel,
                             const Eigen::VectorXd& joint_vel,
                             const rbd::Vector6d& base_acc,
                             const Eigen::VectorXd& joint_acc,
                             const rbd::BodySelector& body_set);

    const rbd::BodyVectorXd& computeAcceleration(const rbd::Vector6d& base_pos,
                                                 const Eigen::VectorXd& joint_pos,
                                                 const rbd::Vector6d& base_vel,
                                                 const Eigen::VectorXd& joint_vel,
                                                 const rbd::Vector6d& base_acc,
                                                 const Eigen::VectorXd& joint_acc,
                                                 const rbd::BodySelector& body_set);

    void computeJdotQdot(rbd::BodyVectorXd& jac_qd,
                         const rbd::Vector6d& base_pos,
                         const Eigen::VectorXd& joint_pos,
                         const rbd::Vector6d& base_vel,
                         const Eigen::VectorXd& joint_vel,
                         const rbd::BodySelector& body_set);

    const rbd::BodyVectorXd& computeJdotQdot(const rbd::Vector6d& base_pos,
                                             const Eigen::VectorXd& joint_pos,
                                             const rbd::Vector6d& base_vel,
                                             const Eigen::VectorXd& joint_vel,
                                             const rbd::BodySelector& body_set);

private:

    RigidBodyDynamics::Model rbd;

    rbd::BodyID body_id_;

    int systemDOF; // 18
    int jointDOF; // 12

    Eigen::VectorXd full_state_;

    rbd::BodyVectorXd body_acc_;
    rbd::BodyVectorXd jdot_qdot_;



};

}

