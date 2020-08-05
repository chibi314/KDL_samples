// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
 
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

    // Create solver based on kinematic chain
    ChainFkSolverPos_recursive fksolver1 = ChainFkSolverPos_recursive(chain);
    ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
    ChainIkSolverPos_NR iksolver1(chain,fksolver1,iksolver1v,1000,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6

    // Create joint array
    unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray q(nj);
    KDL::JntArray q_init(nj);
    for (int i = 0; i < nj; ++i) {
      q_init(i) = 0.1;
    }

    KDL::Frame cartpos;
    fksolver1.JntToCart(q_init, cartpos);

    for(unsigned int i=0;i<3;i++){
        float myinput;
        printf ("Enter the diff pos %i: ",i);
        scanf ("%e",&myinput);
        cartpos.p[i] = (cartpos.p[i] + myinput);
    }

    std::cout << "ref" << std::endl;
    std::cout << cartpos << std::endl;

    bool kinematics_status = iksolver1.CartToJnt(q_init, cartpos, q);
    if(kinematics_status>=0){
      for (int i = 0; i < nj; ++i) {
        std::cout << q(i) << std::endl;
      }
      std::cout << "result" << std::endl;
      KDL::Frame result;
      fksolver1.JntToCart(q, result);
      std::cout << result << std::endl;
      printf("%s \n","Succes, thanks KDL!");
    }else{
      printf("%s \n","Error: could not calculate forward kinematics :(");
    }
}
