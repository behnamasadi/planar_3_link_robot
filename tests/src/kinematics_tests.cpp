#include <gtest/gtest.h>
#include "../../src/task.hpp"
#include <math.h>


TEST(Kinematics, FK)
{

    Eigen::VectorXd goalJointValues(3);
    goalJointValues(0) = M_PI;
    goalJointValues(1) = M_PI/2;
    goalJointValues(2) = M_PI/2;
    trafo2d_t goal = forward_kinematics(goalJointValues);
    Eigen::VectorXd pose=transformationMatrixToPose(goal);
    Eigen::VectorXd expectedPose(3);
    expectedPose<<1.0,0.0,0.0;
    ASSERT_TRUE(pose.isApprox(expectedPose));

}

TEST(Kinematics, IK)
{
    vector_t q_start(3);
    q_start.setConstant(-0.1);

    trafo2d_t goal = trafo2d_t::Identity();
    goal.translation()(0) = 1.0;

    vector_t result = inverse_kinematics(q_start, goal);

    // IK has multiple valid solutions for redundant or symmetric configurations.
    // Verify the end-effector position the IK reaches, not the joint values.
    Eigen::Vector2d goal_xy    = goal.translation();
    Eigen::Vector2d reached_xy = forward_kinematics(result).translation();
    ASSERT_LT((goal_xy - reached_xy).norm(), 1e-2);
}


