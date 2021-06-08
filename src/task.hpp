#include <math.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Geometry>
#include <eigen3/unsupported/Eigen/NumericalDiff>


typedef Eigen::VectorXd vector_t;
typedef Eigen::MatrixXd matrix_t;
typedef Eigen::Transform<double,2,Eigen::Affine> trafo2d_t;

trafo2d_t forward_kinematics(vector_t const & q ) ;

template<typename T>
T pseudoInverse(const T &a, double epsilon = std::numeric_limits<double>::epsilon());


template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>

struct Functor
{
    // Information that tells the caller the numeric type (eg. double) and size (input / output dim)
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
};

typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;


int m_inputs, m_values;

Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

// Get methods for users to determine function input and output dimensions
int inputs() const { return m_inputs; }
int values() const { return m_values; }

};

struct numericalDifferentiationFKFunctor : Functor<double>
{
    // Simple constructor
    numericalDifferentiationFKFunctor(): Functor<double>(3,3) {}

    // Implementation of the objective function
    int operator()(const Eigen::VectorXd &q, Eigen::VectorXd &fvec) const
    {
        trafo2d_t t= forward_kinematics(q);
        double theta=atan2( t.rotation()(1,0),t.rotation()(0,0));
        double x = t.translation()(0);
        double y = t.translation()(1);

        fvec(0) = x;
        fvec(1) = y;
        fvec(2) = theta;

        return 0;
    }
};

Eigen::MatrixXd numericalDifferentiationFK(const Eigen::VectorXd &q);

Eigen::VectorXd transformationMatrixToPose(trafo2d_t const &m);

Eigen::VectorXd distanceError(trafo2d_t const &golesStart, trafo2d_t const &poseStart);

template <typename T> inline constexpr
int signum(T x, std::false_type is_signed);

template <typename T> inline constexpr
int signum(T x, std::true_type is_signed);

template <typename  T>
void normaliseAngle(T &q);

template <typename  T>
void normaliseAngle2(T &q);

void normaliseAngle(Eigen::VectorXd &q);

void normaliseAngle2(Eigen::VectorXd &q);

vector_t inverse_kinematics(vector_t const & q_start, trafo2d_t const & goal );

vector_t inverse_kinematics(trafo2d_t const & goal );

