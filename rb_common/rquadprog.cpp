#include "rquadprog.h"

RBQuadProg::RBQuadProg()
{

}


void RBQuadProg::setNums(int _xlength, int numEqConstraints, int numIneqConstraints){
    NUMCOLS = _xlength;//should be 18+12+3*contact
    NUMEQ = numEqConstraints;//NUMCOLS(dynamics) + NUMCOLS(contactconstraints + swingleg)

    NUMINEQ = numIneqConstraints;//friction cone approximation -ufz<fx<ufz, -ufz<fy<ufz ->4

    //maybe matrix assignment here
    X = MatrixNd::Zero(NUMCOLS,1);
    A_eq = MatrixNd::Zero(NUMEQ, NUMCOLS);
    B_eq = MatrixNd::Zero(NUMEQ, 1);

    A_ineq = MatrixNd::Zero(NUMINEQ, NUMCOLS);
    B_ineq = MatrixNd::Zero(NUMINEQ, 1);


    H = MatrixNd::Zero(NUMCOLS,NUMCOLS);
    F = MatrixNd::Zero(NUMCOLS,1);

    eps = MatrixNd::Identity(NUMCOLS,NUMCOLS)*1e-3;
    eps(0,0) = eps(1,1) = eps(2,2) = 1e-4;//ori
    eps(3,3) = eps(4,4) = eps(5,5) = 1e-4;//pos
}


void RBQuadProg::make_EQ(MatrixNd A, MatrixNd b){
    //Aeq*X = Beq
    A_eq = A;
    B_eq = b;
}

void RBQuadProg::make_IEQ(MatrixNd A, MatrixNd b){
    // Aineq*X < Bineq
    A_ineq = A;
    B_ineq = b;
}

void RBQuadProg::make_HF(MatrixNd A, MatrixNd b){
    //min (0.5* x H x + F x)
//    H = A.transpose() * A + MatrixNd::Identity(NUMCOLS,NUMCOLS)*1e-3;
    H = A.transpose() * A + eps;
    F = -A.transpose() * b;
}


MatrixNd RBQuadProg::solve_QP(){
    quadprogpp::Vector<double> outX;
    quadprogpp::Matrix<double> G;
    quadprogpp::Vector<double> g0;
    quadprogpp::Matrix<double> CE;
    quadprogpp::Vector<double> ce0;
    quadprogpp::Matrix<double> CI;
    quadprogpp::Vector<double> ci0;
    //min 0.5 * x G x + g0 x
    //CE^T x + ce0 = 0
    //CI^T x + ci0 >= 0

    G.resize(NUMCOLS,NUMCOLS);
    g0.resize(NUMCOLS);
    for(int i=0;i<NUMCOLS;i++){
        for(int j=0;j<NUMCOLS;j++){
            G[i][j] = H(i,j);
        }
        g0[i] = F(i,0);
    }
    CE.resize(NUMCOLS,NUMEQ);
    ce0.resize(NUMEQ);
    for(int i=0;i<NUMCOLS;i++){
        for(int j=0;j<NUMEQ;j++){
            CE[i][j] = -A_eq(j,i);
        }
    }
    for(int j=0;j<NUMEQ;j++){
        ce0[j] = B_eq(j,0);
    }
    CI.resize(NUMCOLS,NUMINEQ);
    ci0.resize(NUMINEQ);
    for(int i=0;i<NUMCOLS;i++){
        for(int j=0;j<NUMINEQ;j++){
            CI[i][j] = -A_ineq(j,i);
        }
    }
    for(int j=0;j<NUMINEQ;j++){
        ci0[j] = B_ineq(j,0);
    }
    outX.resize(NUMCOLS);

    solve_quadprog(G, g0, CE, ce0, CI, ci0, outX);
    for(int i=0;i<NUMCOLS;i++){
        X(i,0) = outX[i];
    }

    return X;

}
