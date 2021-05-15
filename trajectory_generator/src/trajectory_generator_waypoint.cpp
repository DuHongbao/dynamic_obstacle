#include "trajectory_generator_waypoint.h"
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>

using namespace std;
using namespace Eigen;

#define inf 1 >> 30

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint() {}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
    const int d_order,           // the order of derivative
    const Eigen::MatrixXd &Path, // waypoints coordinates (3d)
    const Eigen::MatrixXd &Vel,  // boundary velocity
    const Eigen::MatrixXd &Acc,  // boundary acceleration
    const Eigen::VectorXd &Time) // time allocation in each segment
{
  // enforce initial and final velocity and accleration, for higher order
  // derivatives, just assume them be 0;
  cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%QPGeneration%%%%%%%%%%%%%%%%%%%"<<endl;
  int p_order = 2 * d_order - 1; // the order of polynomial
  int p_num1d = p_order + 1;     // the number of variables in each segment

  int m = Time.size();
  MatrixXd PolyCoeff(m, 3 * p_num1d);
  MatrixXd _C_T = MatrixXd::Zero(p_num1d * m, 3*m + 3);
  MatrixXd _C = MatrixXd::Zero(3*m + 3, p_num1d * m);
  _Q = MatrixXd::Zero(p_num1d * m,p_num1d * m);
  
  MatrixXd _M = MatrixXd::Zero(p_num1d * m,p_num1d * m);
  MatrixXd _R = MatrixXd::Zero(p_num1d * m,p_num1d * m);
  MatrixXd _R_PP = MatrixXd::Zero(2 * m - 2,2 * m - 2);
  MatrixXd _R_FP = MatrixXd::Zero(m + 5,2 * m - 2);
  MatrixXd _Q_i = MatrixXd::Zero(p_num1d, p_num1d);
  MatrixXd _M_i = MatrixXd::Zero(p_num1d, p_num1d);
  //cout<<"p_num1d:"<<p_num1d<<endl;
  //cout<<"m:"<<m<<endl;

  //VectorXd Px(p_num1d * m),Py(p_num1d * m),Pz(p_num1d * m);///////////////////
  MatrixXd _d(3*m+3,3),_d_P_star(2*m-2,3),_d_F(m+5,3);//

  /**
   *
   * STEP 3.2:  generate a minimum-jerk piecewise monomial polynomial-based
   * trajectory
   *
   * **/
  _d_F.topRows<1>() = Path.topRows<1>();
  cout<<" ***************1***************"<<endl;
  //_d_F.middleRows(1, 1) = MatrixXd::Zero(2,3);
  //_d_F.middleRows(1, 1) = MatrixXd::Zero(2,3);
  _d_F.middleRows(1, 1) = Vel.row(0);
  _d_F.middleRows(1, 1) = Acc.row(0);
  cout<<" **************2*start velocity&acceleration**************"<<endl;
  _d_F.bottomRows<2>() << MatrixXd::Zero(2,3);/////////////
  cout<<" ***************3*end velocity&acceleration**************"<<endl;
  _d_F.middleRows(m+2, 1) = Path.bottomRows<1>();
  cout<<" ***************4***************"<<endl;
  _d_F.middleRows(3,m-1) = Path.middleRows(1,m-1);
  cout<<" ***************5***************"<<endl;
  cout<<"vel:"<<Vel<<endl;
  cout<<"acc:"<<Acc<<endl;
  for(int i=0;i<m;i++){
    _Q_i.block<3,3>(3,3) << 6*6*Time[i],           6*24*pow(Time[i],2)/2,  6*60*pow(Time[i],3)/3,
                            24*6*pow(Time[i],2)/2, 24*24*pow(Time[i],3)/3, 24*60*pow(Time[i],4)/4,
                            60*6*pow(Time[i],3)/3, 60*24*pow(Time[i],4)/4, 60*60*pow(Time[i],5)/5;
    _M_i << 1,  0,  0,  0,  0,  0,
            0,  1,  0,  0,  0,  0,
            0,  0,  2,  0,  0,  0,
            1, Time[i], pow(Time[i],2), pow(Time[i],3),  pow(Time[i],4),   pow(Time[i],5),
            0, 1,       2*Time[i],      3*pow(Time[i],2),4*pow(Time[i],3), 5*pow(Time[i],4),
            0, 0,       2,              6*Time[i],       12*pow(Time[i],2),20*pow(Time[i],3);
    _Q.block(p_num1d * i,p_num1d * i,p_num1d, p_num1d) << _Q_i;
    _M.block(p_num1d * i,p_num1d * i,p_num1d, p_num1d) << _M_i;
  }
  //cout<<_C_T<<endl;
  // pick up positions constraints for dF
  _C_T.block<3,3>(0,0) << MatrixXd::Identity(3,3);
  cout<<"C size :"<<_C_T.rows()<<"x"<<_C_T.cols()<<endl;
  //cout<<_C_T<<endl;
  //_C.block(p_num1d*m-4,m+1,3,3)=MatrixXd::Identity(3,3);

  _C_T.bottomRows<3>().middleCols(m+2,3) <<  MatrixXd::Identity(3,3);

    /************************************************
  cout<<"Q size :"<<_Q.rows()<<"x"<<_Q.cols()<<endl;
  cout<<_Q<<endl;
  cout<<"M size :"<<_M.rows()<<"x"<<_M.cols()<<endl;
  cout<<_M<<endl;
  cout<<"C size :"<<_C_T.rows()<<"x"<<_C_T.cols()<<endl;

  ***********************************************/
  for(int j = 3; j <  m + 2; j++ ){
      _C_T((j-2)*6-3, j) = 1;
      _C_T((j-2)*6,   j) = 1;
  }
  cout<<"************for1**************"<<endl;

  // pick up velocity constraints for dP
  for(int j = m + 5; j < 3 * m + 3; j+=2 ){
      _C_T((j-(m+5))/2*6+4, j) = 1;
      _C_T((j-(m+5))/2*6+7, j) = 1;
  }
  cout<<"************for2**************"<<endl;

  // pick up  acceleration constrainsts for dP
  for(int j = m + 6; j < 3 * m + 3; j+=2 ){
      _C_T((j-(m+6))/2*6+5, j) = 1;
      _C_T((j-(m+6))/2*6+8, j) = 1;
  }
  cout<<"************for3**************"<<endl;
  //cout<<_C_T<<endl;
  _C = _C_T.transpose();

  _R = _C*_M.inverse().transpose()*_Q*_M.inverse()*_C.transpose();
  _R_PP = _R.block(m+5, m+5, 2*m-2, 2*m-2);
  _R_FP = _R.block(0, m+5, m+5, 2*m-2);
  _d_P_star.col(0) = -_R_PP.inverse()*_R_FP.transpose()*_d_F.col(0);
  _d_P_star.col(1) = -_R_PP.inverse()*_R_FP.transpose()*_d_F.col(1);
  _d_P_star.col(2) = -_R_PP.inverse()*_R_FP.transpose()*_d_F.col(2);
  _d << _d_F,
        _d_P_star;

  cout<<_d<<endl;
  _Px = _M.inverse()*_C.transpose()*_d.col(0);
  _Py = _M.inverse()*_C.transpose()*_d.col(1);
  _Pz = _M.inverse()*_C.transpose()*_d.col(2);
  for(int i = 0; i < m; i++){
    PolyCoeff.row(i).segment(0,p_num1d) = _Px.segment(p_num1d*i,p_num1d);
    PolyCoeff.row(i).segment(p_num1d,p_num1d) = _Py.segment(p_num1d*i,p_num1d);
    PolyCoeff.row(i).segment(2*p_num1d,p_num1d) = _Pz.segment(p_num1d*i,p_num1d);
  }

  return PolyCoeff;
}

double TrajectoryGeneratorWaypoint::getObjective() {
  _qp_cost = (_Px.transpose() * _Q * _Px + _Py.transpose() * _Q * _Py +
              _Pz.transpose() * _Q * _Pz)(0);
  return _qp_cost;
}

Vector3d TrajectoryGeneratorWaypoint::getPosPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 1.0;
      else
        time(j) = pow(t, j);

    ret(dim) = coeff.dot(time);
     //cout << "dim:" << dim << " coeff:" << coeff << endl;
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getVelPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0)
        time(j) = 0.0;
      else
        time(j) = j * pow(t, j - 1);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}

Vector3d TrajectoryGeneratorWaypoint::getAccPoly(MatrixXd polyCoeff, int k,
                                                 double t) {
  Vector3d ret;
  int _poly_num1D = (int)polyCoeff.cols() / 3;
  for (int dim = 0; dim < 3; dim++) {
    VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
    VectorXd time = VectorXd::Zero(_poly_num1D);

    for (int j = 0; j < _poly_num1D; j++)
      if (j == 0 || j == 1)
        time(j) = 0.0;
      else
        time(j) = j * (j - 1) * pow(t, j - 2);

    ret(dim) = coeff.dot(time);
  }

  return ret;
}