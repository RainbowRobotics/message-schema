#ifndef RBQUADPROG_H
#define RBQUADPROG_H

#include <rbdl/rbdl.h>
#include "utils/QuadProg++.hh"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

class RBQuadProg
{
public:
    RBQuadProg();


    void setNums(int _xlength, int numEqConstraints, int numIneqConstraints);
    void make_EQ(MatrixNd A, MatrixNd b);
    void make_IEQ(MatrixNd A, MatrixNd b);
    void make_HF(MatrixNd A, MatrixNd b);

    MatrixNd solve_QP();


private:
    int NUMCOLS;//length of X
    int NUMEQ;
    int NUMINEQ;


    MatrixNd H,F;
    MatrixNd X;

    MatrixNd A_ineq,B_ineq;
    MatrixNd A_eq,B_eq;

    MatrixNd eps;

};

#endif // RBQUADPROG_H
