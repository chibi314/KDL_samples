#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include <Eigen/Core>

using namespace KDL;
 
 
int main( int argc, char** argv )
{
    //Definition of a kinematic chain & add segments to the chain
    KDL::Chain chain;
    // chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
    // chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
    // chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
    // chain.addSegment(Segment(Joint(Joint::RotZ)));
    // chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
    // chain.addSegment(Segment(Joint(Joint::RotZ)));
    chain.addSegment(Segment(Joint(Joint::None), Frame(Vector(0.25, 0.0, 0.0))));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.01, 0.0, 0.0))));
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.5, 0.0, 0.0))));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(0.01, 0.0, 0.0))));
    chain.addSegment(Segment(Joint(Joint::RotY), Frame(Vector(0.5, 0.0, 0.0))));

    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    // Create solver based on kinematic chain
    Eigen::Matrix<double, 6, 1> weight;
    weight << 1.0, 1.0, 1.0, 0.0, 0.0, 1.0;
    ChainIkSolverPos_LMA iksolver(chain, weight, 1e-8, 1000, 1e-20);//Maximum 100 iterations, stop at accuracy 1e-6

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray q(nj);
    KDL::JntArray q_init(nj);
    for (int i = 0; i < nj; ++i) {
      q_init(i) = 0.01;
    }

    KDL::Frame cartpos;
    fksolver.JntToCart(q_init, cartpos);

    for(unsigned int i=0;i<3;i++){
        float myinput;
        printf ("Enter the diff pos %i: ",i);
        scanf ("%e",&myinput);
        cartpos.p[i] = (cartpos.p[i] + myinput);
    }

    std::cout << "ref" << std::endl;
    std::cout << cartpos << std::endl;
    double r, p, y;
    cartpos.M.GetEulerZYX(y, p, r);
    std::cout << r << " " << p << " " << y << std::endl;

    int kinematics_status = iksolver.CartToJnt(q_init, cartpos, q);

    for (int i = 0; i < nj; ++i) {
      std::cout << q(i) << std::endl;
    }
    std::cout << "result" << std::endl;
    KDL::Frame result;
    fksolver.JntToCart(q, result);
    std::cout << result << std::endl;
    result.M.GetEulerZYX(y, p, r);
    std::cout << r << " " << p << " " << y << std::endl;
    

    std::cout << kinematics_status << std::endl;
}
