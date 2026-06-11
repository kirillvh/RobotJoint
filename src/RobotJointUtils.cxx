/*
Copyright 2012 Kirill Van Heerden
SPDX-License-Identifier: MIT

Licensed under the MIT License. See LICENSE for details.
*/

#include "RobotJointUtils.h"
#include <iostream>
#include <iomanip>
using namespace std;
using namespace Eigen;
// using namespace RobotJoint;


RJVectorX RobotJoint::SolveQp(const RJMatrixX &A, const RJVectorX &B, const RJMatrixX &C, const RJVectorX &D)
/**     @brief Minimizes a quadratic cost function subjected to a linear set of constraints
*	This function is a backend: See QP_Thiel_VanDePanne() for a more general QP solver
*       cost function: \f[ f(x)=0.5 \cdot X^T A  X + B^T X + c \f]
*       constraints  : \f[ C X = D \f]
*
*
*       This is done by solving 
*       \f[ \left[\begin{array}{cc}
*       A & C^{T}\\
*       C & 0
*       \end{array}\right]Y=\left[\begin{array}{c}
*       -B\\
*       D
*       \end{array}\right] \f]
*       where \f[ Y = \left[\begin{array}{c}
*       X\\
*       \lambda
*       \end{array}\right] \f]
*       and \f$ \lambda \f$ is the lagrange multiplier.
*
*       Example of forming A and B matrices \f[ f(x)= \sum^N_{i=1} k_i(x_i-x^{ref}_i)^2 \f]
*       can be expressed as: \f[ f(x) = X^T K X - 2 \cdot (X^{ref})^T X+ (X^{ref})^2 \f]
*       with: \f[ A = K = Diag(k_1,...,k_N) = A\f]
*       \f[ B = -2 \cdot (X^{ref})^T\f]
*/
{
        RJMatrixX lhs = RJMatrixX::Zero(A.cols() + C.rows(), A.cols() + C.rows());
        RJVectorX rhs = RJVectorX::Zero(B.size() + D.size());
        RJVectorX result;//= RJVectorX::Zero(B.size() + D.size());

        lhs.block(0, 0, A.rows(),A.cols()) = A;
        lhs.block(0, A.cols(), C.cols(),C.rows()) = C.transpose();
        lhs.block(A.rows(), 0,C.rows(),C.cols()) = C;  

        rhs.segment(0, B.size()) = -B;
        rhs.segment(B.size(), D.size()) = D;
//         result = lhs.fullPivHouseholderQr().solve(rhs); //slower but more numerically stable
	result = lhs.householderQr().solve(rhs); //faster but less numerically stable

        return result;
}

/**
        \brief Checks if a n-dimensional point lies below a set of hyperplanes
*/
bool RobotJoint::BelowHyperPlanes(const RJVectorX &x,const RJMatrixX &C,const RJVectorX &D)
{
//This epsilon value is hard to define becaouse it has to take the neumerical stability of SolveQp into account
#define RJ_epsilon_BHP std::numeric_limits<float>::epsilon()
	RJVectorX z = C*x;
        for(int i=0; i<D.size(); i++)
	{
		if((z(i)-D(i) > RJ_epsilon_BHP) && (fabs(z(i)-D(i)) > RJ_epsilon_BHP))
		return false;
	}
	return true;
}


RJVectorX RobotJoint::QP_Thiel_VanDePanne(const RJMatrixX &A, const RJVectorX &B, const RJMatrixX &C, const RJVectorX &D)
/**
        \brief Implements the basic Thiel & Van De Panne algorithim of finding active constraints and solving a QP problem with them.
	Uses SolveQp as a backend. This function generates and tries out the possible combinations of active constraints.
*/
{
        int N=D.size();
        #define nbits 32
        RJMatrixX At,Ct;
        RJVectorX Bt,Dt;
	//this function uses bitsets to store and generate(converting decimal to binary) the possible constraint combinations
        std::vector<std::bitset<nbits> > bst;
        std::vector<RJVectorX, Eigen::aligned_allocator<RJVectorX> > sol;
        std::vector<std::vector<std::bitset<nbits> > > ordered_bst;
//         std::vector<std::bitset<32>> row
        RJVectorX soli(A.rows());
	RJVectorX solisub;
	RJVectorX base_input_vector;

	//first check if the current problem requires no optimization, return if thats the case
	base_input_vector = -B;//*-0.5;
	if(BelowHyperPlanes(base_input_vector,C,D))
{
// cout<<"optimization not needed"<<base_input_vector.transpose()<<endl;
// cout<< "C:"<<endl<<C<<endl;
// cout<< "D:"<<endl<<D<<endl;
 return base_input_vector;
}
        
        for(int i=1; i<N*N; i++)
        {//Make bitset of possible combinations
                bitset<nbits> bs((long)i);
                bst.push_back(bs);                
        }
        
        
        for(int i=0; i<N+1; i++)
        { //Make 2-d vector of bitsets
                ordered_bst.push_back( std::vector<std::bitset<nbits> >() );
        }
        
        //now order the bitsets according to the number of ones they contain
        int nOnes=0;
        for(int i=0; i<bst.size(); i++)
        {
                nOnes = 0;
//                 first check how many ones
                for(int j=0; j<N; j++)
                {
                        if(bst[i].test(j)) nOnes++;
                }
//                 next order bst according to the number of ones
                (ordered_bst[nOnes]).push_back(bst[i]);     //BUG there may be some memory curruption somewhere around here...
        }
        

RJMatrixX At_temp, Ct_temp;
//         next go through each level of ordered_bst and check if it leads to a solution
        for(int i=1; i<=N; i++)
        {
// cout<< "iterating i="<<i<<endl;
                sol.clear(); //redundant step, clearing solutions that werent added to begin with
                for(int j=0; j<ordered_bst[i].size(); j++)
                {
// cout<< "iterating j="<<j<<endl;
			At = RJMatrixX::Zero(i,i);
//                         Ct = RJMatrixX::Zero(i,i);
			At_temp = RJMatrixX::Zero(A.rows(),i);
//                         Ct_temp = RJMatrixX::Zero(C.rows(),i);
			Ct = RJMatrixX::Zero(i,C.cols());
			Bt.resize(i);
                        Dt.resize(i);
                        int l=0;
//                         now construct the active set of constraints
                        for(int k=0; k<N; k++)
                        {
//                                 make the active set of constraints
                                if (ordered_bst[i][j].test(k))
                                {
//                                         Ct.block(l,0,1,C.cols()) = C.block(k,0,1,C.cols());
// 					At_temp.col(l) = A.col(k);
//                                         Ct_temp.col(l) = C.col(k);
					Ct.row(l) = C.row(k);
// 					Bt(l)   = B(k);
                                        Dt(l)   = D(k);
                                        l++;
                                }
                        }

// 			l=0;
//                         for(int k=0; k<N; k++)
//                         {
//                                 if (ordered_bst[i][j].test(k))
//                                 {
// 					At.row(l) = At_temp.row(k);
//                                         Ct.row(l) = Ct_temp.row(k);
// 					l++;
// 				}
// 
// 			}
//                         get a solution
//                         solisub = SolveQp(A,B,Ct,Dt);
// cout<< "A "<<A<<endl;
// cout<< "B "<<B<<endl;
// cout<< "Ct "<<Ct<<endl;
// cout<< "Dt "<<Dt<<endl;
			soli = SolveQp(A,B,Ct,Dt).head(A.rows());
// 			//construct solution vector
// 			l = 0;
//                         for(int k=0; k<N; k++)
//                         {
//                                 if (ordered_bst[i][j].test(k))
//                                 {
// 					soli(k) = solisub(l);
// 					l++;
// 				}
// 				else soli(k) = base_input_vector(k);
// 
// 			}
// cout<<"soli "<<soli<<endl;

//                         add this solution to the solution vector if its valid
                        if(BelowHyperPlanes(soli,C,D)){
 sol.push_back(soli);
// cout<<"added solution!"<<endl;
}
                }
//if a valid solution exists then theres no need to try a higher number of simultaneously active constraints
                if(sol.size() != 0) break; //exit the loop
        }
        
        RJReal fitc,fitbest;
        int fiti;
        fiti=0;
        fitc=0;
        fitbest=0;
        //if theres more than one valid solution then get the optimum one
        //TODO check if there are no solutions
        RJMatrixX solv;
        if(sol.size() != 0)
        {
                for(int i=0; i<sol.size(); i++)
                {
//  cout<<"solution["<<i<<"] = "<< sol[i].transpose() <<endl;
                        fitc = (sol[i].segment<2>(0).transpose()*A*sol[i].segment<2>(0) + B.transpose()*sol[i].segment<2>(0))[0]; //()[0] is abit of a hack to get eigen to convert to RJReal
                        if(fitc < fitbest) 
			{
				fitbest = fitc;
				fiti = i;
			}
                }

        }

	RJASSERT( ((fiti>=0)&&(fiti<sol.size())), "sol["<<fiti<<"] is an invalid index! Sol size is "<<sol.size()<<endl);
        return sol[fiti].head(A.rows());
}

/**
*       \brief Solves Continous Riccati equation
*       ref: Arnold, W.F., III and A.J. Laub, "Generalized Eigenproblem Algorithms and Software for Algebraic Riccati Equations,"
*                Proc.IEEE, 72 (1984), pp. 1746-1754
*/
RJMatrixX RobotJoint::RiccatiSolve(const RJMatrixX &A, const RJMatrixX &B, const RJMatrixX &Q, const RJMatrixX &R)
{
        if (A.rows() != A.cols()) throw "RiccatiSolve: A is not symetric!";
        if (A.rows() != B.rows()) throw "RiccatiSolve: A & B dont have equal number of rows!";
        if (Q.rows() != Q.cols()) throw "RiccatiSolve: Q is not symetric!";
        if (Q.rows() != A.rows()) throw "RiccatiSolve: A & B dont have equal number of rows!";
        
        MatrixXd ret;
        MatrixXd Rinv = (R.inverse()).cast<double>();

        //Make the Hamiltonian matrix: H=[A -B*inv(R)*B';-Q -A'];
        MatrixXd H(A.rows()+Q.rows(), A.cols()+Q.cols());
        H.block(0,0,A.rows(),A.cols())=A.cast<double>();
        H.block(0,A.cols(),A.rows(),A.rows())=-B.cast<double>()*Rinv.cast<double>()*B.cast<double>().transpose();
        H.block(A.rows(),0,Q.rows(),Q.cols())=-Q.cast<double>();
        H.block(A.rows(),Q.cols(),A.rows(),A.cols())=-A.cast<double>().transpose();

        //Get the Schur decomposition H=U*T*T', where U is a orthagonal matrix and T is a quasi-triangular matrix
        RealSchur<MatrixXd> schur(H);
        MatrixXd U = schur.matrixU();
        MatrixXd T = schur.matrixT();

        //get the ordered eigen values
        VectorXcd ordeigs;
        std::vector<int > select;
        ordeigs=ordeig(T);

        for(int i=0;i<ordeigs.size();i++)
        {
                //filter the eigen values according to those that lie in the left hand plane
                if (real(ordeigs[i])<0)                 select.push_back(1);
                else                                    select.push_back(0);
        }

        //reorder the Schur factorization
        if(!OrdSchur(T, U, &select[0])) cout<<"OrdSchur()  -- Fail!!"<<endl;
        else
        {
                //TODO throw a exception here or something
        }

        int nrows=A.rows();
        int ncols=A.cols();

        RJMatrixX U21=U.cast<RJReal>().block(nrows,0,nrows,ncols);
        RJMatrixX U11inv=U.cast<RJReal>().block(0,0,nrows,ncols);
        U11inv=U11inv.inverse();

//         ret= U21*U11inv;

// return ret.cast<RJReal>();
return U21*U11inv;
}

/**
*       \brief Solves Discrete Riccati equation
*       ref: Arnold, W.F., III and A.J. Laub, "Generalized Eigenproblem Algorithms and Software for Algebraic Riccati Equations,"
*                Proc.IEEE, 72 (1984), pp. 1746-1754
*/
RJMatrixX RobotJoint::RiccatiSolveDiscrete(const RJMatrixX &A, const RJMatrixX &B, const RJMatrixX &Q, const RJMatrixX &R)
{
        if (A.rows() != A.cols()) throw "RiccatiSolveDiscrete: A is not symetric!";
        if (A.rows() != B.rows()) throw "RiccatiSolveDiscrete: A & B dont have equal number of rows!";
        if (Q.rows() != Q.cols()) throw "RiccatiSolveDiscrete: Q is not symetric!";
        if (Q.rows() != A.rows()) throw "RiccatiSolveDiscrete: A & B dont have equal number of rows!";
        
//         RJMatrixX ret;
        MatrixXd Rinv = (R.inverse()).cast<double>();
        MatrixXd Ainv;
        MatrixXd Aintern = A.cast<double>();
        pseudoInverse(Aintern, Ainv);
        
        MatrixXd H(A.rows()+Q.rows(), A.cols()+Q.cols());
        H.block(0,0,A.rows(),A.cols())=A.cast<double>() + B.cast<double>()*Rinv.cast<double>()*B.cast<double>().transpose()*Ainv.cast<double>().transpose()*Q.cast<double>();
        H.block(0,A.cols(),A.rows(),A.rows())=-B.cast<double>()*Rinv.cast<double>()*B.cast<double>().transpose()*Ainv.cast<double>().transpose();
        H.block(A.rows(),0,Q.rows(),Q.cols())=-Ainv.cast<double>().transpose()*Q.cast<double>();
        H.block(A.rows(),Q.cols(),A.rows(),A.cols())=(Ainv.transpose()).cast<double>();
        
        //Get the Schur decomposition H=U*T*T', where U is a orthagonal matrix and T is a quasi-triangular matrix
        RealSchur<MatrixXd> schur(H);
        MatrixXd U = schur.matrixU();
        MatrixXd T = schur.matrixT();

        //get the ordered eigen values
        VectorXcd ordeigs;
        std::vector<int > select;
        ordeigs=ordeig(T);
        
        for(int i=0;i<ordeigs.size();i++)
        {
                //filter the eigen values according to those that lie inside the unit circle
                if (abs(ordeigs[i]) < 1.0)                 select.push_back(1);
                else                                       select.push_back(0);
                
        }
        
        //reorder the Schur factorization
        if(!OrdSchur(T, U, &select[0])) cout<<"OrdSchur()  -- Fail!!"<<endl;
        else
        {
                //TODO throw a exception here or something
        }

        int nrows=A.rows();
        int ncols=A.cols();

        RJMatrixX U21=U.cast<RJReal>().block(nrows,0,nrows,ncols);
        RJMatrixX U11inv=U.cast<RJReal>().block(0,0,nrows,ncols);
        U11inv=U11inv.cast<RJReal>().inverse();

//         ret= U21*U11inv;

// return ret;
return U21*U11inv;
}

/**
*        \brief Designs a matrix of feedback gains using the LQR method
*/
RJMatrixX RobotJoint::LQR(const RJMatrixX &A, const RJMatrixX &B, const RJMatrixX &Q, const RJMatrixX &R)
{
        if (A.rows() != A.cols()) throw "LQR: A is not symetric!";
        if (A.rows() != B.rows()) throw "LQR: A & B dont have equal number of rows!";
        if (Q.rows() != Q.cols()) throw "LQR: Q is not symetric!";
        if (Q.rows() != A.rows()) throw "LQR: A & B dont have equal number of rows!";
        
        RJMatrixX P = RJMatrixX::Zero(A.rows(),A.cols());
        P = RiccatiSolve(A, B, Q, R);
        return R.inverse()*B.transpose()*P;
}

RJMatrixX RobotJoint::LQRDiscrete(const RJMatrixX &A, const RJMatrixX &B, const RJMatrixX &Q, const RJMatrixX &R)
{
        if (A.rows() != A.cols()) throw "LQRDiscrete: A is not symetric!";
        if (A.rows() != B.rows()) throw "LQRDiscrete: A & B dont have equal number of rows!";
        if (Q.rows() != Q.cols()) throw "LQRDiscrete: Q is not symetric!";
        if (Q.rows() != A.rows()) throw "LQRDiscrete: A & B dont have equal number of rows!";
        
        RJMatrixX P = RJMatrixX::Zero(A.rows(),A.cols());
        P = RiccatiSolveDiscrete(A, B, Q, R);
        return R.inverse()*B.transpose()*P;
}
/**
*       \brief Designs Preview gains and feedback gains using the method by Katayama et al.
* Dimensions:  A - n*n , B - n*r , C - p*n
* Ref: "Design of an optimal controller for a discrete-time system subjected to previewable demand" by Touru Katayama, Takahira Ohki, Toshio Inoue and Tomoyuki Kato
*/
int RobotJoint::CalcPreviewGain(const RJMatrixX &A, const RJMatrixX &B, const RJMatrixX &C, const RJMatrixX &Q, const RJMatrixX &R,const  int &preview_window_size, std::vector<RJMatrixX> &preview_gain, RJMatrixX &GI, RJMatrixX &GX)
{
        if (A.rows() != A.cols()) throw "CalcPreviewGain: A is not symetric!";
        if (A.rows() != B.rows()) throw "CalcPreviewGain: A & B dont have equal number of rows!";
        if (Q.rows() != Q.cols()) throw "CalcPreviewGain: Q is not symetric!";
        if (Q.rows() != A.rows()) throw "CalcPreviewGain: A & B dont have equal number of rows!";
    
        int n = A.rows();
        int r = B.cols();
        int p = C.rows();
        RJMatrixX CB = C*B;
        RJMatrixX CA = C*A;
        RJMatrixX Ip = RJMatrixX::Ones(p,p);//Identity
        RJMatrixX Btilda = RJMatrixX::Zero(CB.rows()+B.rows(),B.cols());
        RJMatrixX Ftilda = RJMatrixX::Zero(CA.rows()+A.rows(),A.cols());
        RJMatrixX Itilda = RJMatrixX::Zero(CA.rows()+A.rows(),Ip.cols());
        RJMatrixX Atilda = RJMatrixX::Zero(Itilda.rows(),Itilda.cols()+Ftilda.cols());
        RJMatrixX Qtilda = RJMatrixX::Zero(Atilda.rows(),Atilda.cols());       

        Btilda.topLeftCorner(CB.rows(),CB.cols())  = CB;
        Btilda.bottomLeftCorner(B.rows(),B.cols()) = B;

        Ftilda.topLeftCorner(CA.rows(),CA.cols())  = CA;
        Ftilda.bottomLeftCorner(A.rows(),A.cols()) = A;

        Itilda.topLeftCorner(Ip.rows(), Ip.cols()) = Ip;

        Atilda.topLeftCorner (Itilda.rows(),Itilda.cols()) = Itilda;
        Atilda.topRightCorner(Ftilda.rows(),Ftilda.cols()) = Ftilda;

//         Qtilda.topLeftCorner(Q.rows(),Q.cols()) = Q; // implement Qx     Qtilda=[Qe 0;0 Qx]
//      ### HACK need to properly take care of this matrix
        Qtilda(0, 0) = 1.0;

        RJMatrixX Ktilda = RiccatiSolveDiscrete(Atilda,Btilda, Qtilda, R);
        RJMatrixX part0 = (R+Btilda.transpose()*Ktilda*Btilda);
        RJMatrixX part1;
        pseudoInverse(part0 , part1);
        RJMatrixX part3 = part1*Btilda.transpose();
        RJMatrixX part2 = part3*Ktilda;
        GI = part2*Itilda;
        GX = part2*Ftilda;
        
        //now generate preview gain
        preview_gain.push_back(-GI);
        RJMatrixX Actilda = Atilda - Btilda*part2*Atilda;
        RJMatrixX Xl = -1*Actilda.transpose()*Ktilda*Itilda;   
        for(int i=0; i<preview_window_size;i++)
        {
                preview_gain.push_back(part3*Xl);
                Xl = Actilda.transpose()*Xl;
        }
        return 0;
}

/**
        \brief uses LAPACKs "dtrsen" to rearrange the Schur decomposition A=Q*T*Q' so that the selected eigen values appear in the top right corder of T...
*/
bool RobotJoint::OrdSchur(MatrixXd& T, MatrixXd& Q, int * SELECT)
{
        int N = T.cols(); 

        int LDT = T.outerStride();
        int LDQ = Q.outerStride();

        double WORKDUMMY;
        int LWORK = -1; // Get optimum work size.
        int IWORK = 0;
        int LIWORK = 1;
        int INFO = 0;
  
        MatrixXd dont_care_WR = MatrixXd::Zero(N,N);
        MatrixXd dont_care_WI = MatrixXd::Zero(N,N);
        int dont_care_M;
        double dont_care_S;
        double dont_care_SEP;

        dtrsen_("N", "V", SELECT, &N, T.data(), &N, Q.data(), &N,
                           dont_care_WR.data(), dont_care_WI.data(), &dont_care_M, &dont_care_S, &dont_care_SEP, &WORKDUMMY, &LWORK, &IWORK,
                          &LIWORK, &INFO );

        LWORK = int(WORKDUMMY) + 32;
        IWORK=N;
        LIWORK=N*N;
        VectorXd WORK(LWORK);

        dtrsen_("N", "V", SELECT, &N, T.data(), &N, Q.data(), &N,
                           dont_care_WR.data(), dont_care_WI.data(), &dont_care_M, &dont_care_S, &dont_care_SEP, WORK.data(), &LWORK, &IWORK,
                          &LIWORK, &INFO );

        return INFO==0;
}

/**
        \brief Computes eigenvalues in the order they appear in a quazi triangluar matrix
*/
VectorXcd RobotJoint::ordeig(const MatrixXd &t)
{
        if (t.rows() != t.cols()) throw "OrdEig: input matrix not square!";
        
        int n = t.cols();
        VectorXcd eigs = VectorXcd::Zero(n);
        int i = 1;
        
        for(int j=0; j<n; j++)
        {
                if (j + 1 >= n) eigs(j) = t(j,j);
                else if (t(j+1,j+1) == 0)
                {
                        eigs(j) = t(j,j);
                }
                else
                {
                        eigs.segment<2>(j) = t.block<2,2>(j,j).eigenvalues();
                        j++;
                }
        }
        return eigs;
}