#include <RigidBodyDynamics.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

namespace dwl

{

class WholeBodyDynamic
{
public:

    WholeBodyDynamic();

    ~WholeBodyDynamic();

    void modelFromURDFModel(const std::string& urdf_model, bool info);

    void fromGeneralizedJointState(rbd::Vector6d& base_state,
                                   Eigen::VectorXd& joint_state,
                                   const Eigen::VectorXd& generalized_state);

    void convertAppliedExternalForces(std::vector<RigidBodyDynamics::Math::SpatialVector>& f_ext,
                                           const rbd::BodyVector6d& ext_force,
                                           const Eigen::VectorXd& generalized_joint_pos);

    const Eigen::VectorXd& toGeneralizedJointState(const rbd::Vector6d& base_state,
                                                   const Eigen::VectorXd& joint_state);

    const Eigen::MatrixXd& computeJointSpaceInertiaMatrix(const rbd::Vector6d& base_pos,
                                                          const Eigen::VectorXd& joint_pos);

    const Eigen::VectorXd& computeNonlinearEffectsForce(const rbd::Vector6d& base_pos,
                                                        const Eigen::VectorXd& joint_pos,
                                                        const rbd::Vector6d& base_vel,
                                                        const Eigen::VectorXd& joint_vel);
    void computeFloatingBaseInverseDynamics(rbd::Vector6d& base_acc,
                                            Eigen::VectorXd& joint_forces,
                                            const rbd::Vector6d& base_pos,
                                            const Eigen::VectorXd& joint_pos,
                                            const rbd::Vector6d& base_vel,
                                            const Eigen::VectorXd& joint_vel,
                                            const Eigen::VectorXd& joint_acc,
                                            const rbd::BodyVector6d& ext_force);


private:

    int systemDOF;

    int jointDOF;

    RigidBodyDynamics::Model rbd;

    rbd::BodyID body_id_;

    Eigen::VectorXd full_state_;

    /** @brief The joint-space inertial matrix of the system */
    Eigen::MatrixXd joint_inertia_mat_;

    /** @brief The Coriolis, centrifugal and gravity force */
    Eigen::VectorXd nonliner_effects_;


};

}
