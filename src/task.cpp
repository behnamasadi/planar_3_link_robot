#include "task.hpp"

typedef Eigen::VectorXd vector_t;
typedef Eigen::MatrixXd matrix_t;
typedef Eigen::Transform<double,2,Eigen::Affine> trafo2d_t;

/**************************************************
 * A function to compute the forward kinematics of
 * a planar 3-link robot.
 *************************************************/
trafo2d_t forward_kinematics(vector_t const & q ) {
    // check that the joint angle vector has the correct size
    assert( q.size() == 3 );

    // define a constant offset between two joints
    trafo2d_t link_offset = trafo2d_t::Identity();
    link_offset.translation()(1) = 1.;

    // define the start pose
    trafo2d_t trafo = trafo2d_t::Identity();

    for(int joint_idx = 0; joint_idx < 3 ; joint_idx++ ) {
        // add the rotation contributed by this joint
        trafo *= Eigen::Rotation2D<double>(q(joint_idx));
        // add the link offset to this position
        trafo = trafo * link_offset;
    }
    return trafo;
}

template<typename T>
T pseudoInverse(const T &a, double epsilon)
{
    //Eigen::DecompositionOptions flags;
    int flags;
    // For a non-square matrix
    if(a.cols()!=a.rows())
    {
        flags=Eigen::ComputeThinU | Eigen::ComputeThinV;
    }
    else
    {
        flags=Eigen::ComputeFullU | Eigen::ComputeFullV;
    }
    Eigen::JacobiSVD< T > svd(a ,flags);

    //double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

Eigen::MatrixXd numericalDifferentiationFK(const Eigen::VectorXd &q)
{
    numericalDifferentiationFKFunctor functor;
    Eigen::NumericalDiff<numericalDifferentiationFKFunctor> numDiff(functor);
    Eigen::MatrixXd fjac(3,3);
    numDiff.df(q,fjac);
    return fjac;
}

Eigen::VectorXd transformationMatrixToPose(trafo2d_t const &m)
{
    double theta=atan2( m.rotation()(1,0),m.rotation()(0,0));
    double x = m.translation()(0);
    double y = m.translation()(1);

    Eigen::VectorXd fvec(3);
    fvec(0) = x;
    fvec(1) = y;
    fvec(2) = theta;
    return fvec;
}

Eigen::VectorXd distanceError(trafo2d_t const &golesStart, trafo2d_t const &poseStart)
{
     return transformationMatrixToPose(golesStart)-transformationMatrixToPose(poseStart);
}

template <typename T> inline constexpr
int signum(T x, std::false_type is_signed) {
    return T(0) < x;
}

template <typename T> inline constexpr
int signum(T x, std::true_type is_signed) {
    return (T(0) < x) - (x < T(0));
}

template <typename T> inline constexpr
int signum(T x) {
    return signum(x, std::is_signed<T>());
}

template <typename  T>
void normaliseAngle(T &q)
{
    int sign=signum(q);
    q=fabs(q);
    q=sign*remainder(q,2*M_PI);
    if(sign<0)
        q=q+2*M_PI;
}

template <typename  T>
void normaliseAngle2(T &q)
{
    int sign=signum(q);
    q=remainder(q,sign*2*M_PI);
    if(-2*M_PI<=q && q <=-M_PI)
        q=q+2*M_PI;

    else if(+M_PI<=q && q <=2*M_PI)
        q=2*M_PI-q;
}

void normaliseAngle(Eigen::VectorXd &q)
{
    for(int i=0;i<q.rows();i++)
    {
        normaliseAngle(q(i));
    }
}

void normaliseAngle2(Eigen::VectorXd &q)
{
    for(int i=0;i<q.rows();i++)
    {
        normaliseAngle2(q(i));
    }
}



/*************************************************
 * Task:
 * Complete this inverse kinematics function for the robot defined by
 * the forward kinematics function defined above.
 * It should return the joint angles q for the given goal specified in the
 * corresponding parameter.
 * Only the translation (not rotation) part of the goal has to match.
 * 
 *
 * Hints:
 * - This is an non-linear optimization problem which can be solved by using
 *   an iterative algorithm.
 * - To obtain the jacobian, use numerical differentiation
 * - To invert the jacobian use Eigen::JacobiSVD
 * - The algorithm should stop when norm of the error is smaller than 1e-3
 * - The algorithm should also stop when 200 iterations are reached
 ************************************************/
vector_t inverse_kinematics(vector_t const & q_start, trafo2d_t const & goal )
{
    vector_t q=q_start;
    vector_t delta_q(3);


    double epsilon=1e-3;

    int i=0;
    double gamma;
    double stepSize=10;

    while( (distanceError(goal,forward_kinematics(q)).squaredNorm()>epsilon)  && (i<200)  )
    {
        Eigen::MatrixXd jacobian=numericalDifferentiationFK(q);
        Eigen::MatrixXd j_pinv=pseudoInverse(jacobian);
        Eigen::VectorXd delta_p=transformationMatrixToPose(goal)-transformationMatrixToPose(forward_kinematics(q) );

        gamma=sqrt(pow(delta_p(0),2) +pow(delta_p(1),2));
        //std::cout<<"gamma" <<gamma <<std::endl;
        if(gamma >2)
        {
            delta_p(0)=delta_p(0)/stepSize*gamma;
            delta_p(1)=delta_p(1)/stepSize*gamma;
        }
        else
        {
            delta_p(0)=delta_p(0)/stepSize;
            delta_p(1)=delta_p(1)/stepSize;

        }
        delta_q=j_pinv*delta_p;
        q=q+delta_q;
        normaliseAngle2(q);
        i++;
    }
    return q;
}

vector_t inverse_kinematics(trafo2d_t const & goal )
{
    vector_t q_start(3);
    q_start.setConstant(0.0);
    return inverse_kinematics( q_start,  goal );
}


/**
 * An example how the inverse kinematics can be used.
 * It should not be required to change this code.
 */


