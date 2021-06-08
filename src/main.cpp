#include "task.hpp"

int main()
{
    vector_t q_start(3);
    q_start.setConstant(-0.1);

    trafo2d_t goal = trafo2d_t::Identity();
    goal.translation()(0) = 1.0;

    Eigen::VectorXd goalJointValues(3);

    std::cout <<"goal:\n" <<transformationMatrixToPose(goal) << std::endl;
    vector_t result = inverse_kinematics(q_start,goal);
    std::cout <<"joint values for the given pose are:\n" <<result << std::endl;
    std::cout << "goal pose is:\n" << transformationMatrixToPose(goal) <<std::endl;
    std::cout << "estimated pose from IK is:\n" << transformationMatrixToPose(forward_kinematics(result)) <<std::endl;
    return 0;
}
