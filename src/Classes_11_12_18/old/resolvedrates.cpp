#include "resolvedrates.h"
#include <Eigen/Dense>

ResolvedRates::ResolvedRates():
{
  // Robot Setup
  auto cannula = defineRobot(Params_); // TODO: save this to class

  // Home Position
  CTR3ModelStateVector qHome; // Starting Configuration (HOME)
  qHome.PsiL_ = Eigen::Vector3d::Zero();
  qHome.Beta_ << -160.90E-3, -127.2E-3, -86.4E-3;
  qHome.Ftip_ = Eigen::Vector3d::Zero();
  qHome.Ttip_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d qHomeAlpha;
  qHomeAlpha << 0.0, 0.0, 0.0;
  QVec_.topRows(3) = qHome.PsiL_;
  QVec_.bottomRows(3) = qHome.Beta_;

  // Initial Kinematics Call
  auto ret1 = Kinematics_with_dense_output( cannula, QVec_, OType() );
  int nPts = ret1.arc_length_points.size();
  double* ptr = &ret1.arc_length_points[0];
  Eigen::Map<Eigen::VectorXd> s(ptr,nPts);

  // Forward Kinematics (Results in Base Frame)
  auto poseData = forwardKinematics(ret1); // TODO: save this to class

  // Interpolate Robot Backbone
  int nInterp = 200;
  InterpolatedBackboneCur_ = interpolateBackbone(s.reverse(), poseData, nInterp);  // do we want to sae this or poseData w/ interp?
  Eigen::MatrixXd poseDataOut(8,nInterp+nPts);
  poseDataOut = Eigen::MatrixXd::Zero(8,nInterp+nPts);
  Eigen::RowVectorXd ones(nInterp+nPts);
  ones.fill(1);
  Eigen::VectorXd sOut = InterpolatedBackboneCur_.s;
  poseDataOut.topRows(3) = InterpolatedBackboneCur_.p;
  poseDataOut.middleRows<4>(3) = InterpolatedBackboneCur_.q;
  poseDataOut.bottomRows(1) = ones;

  // ResolvedRates Setup
  PTip_ << poseDataOut(0,lastPosInterp_), poseDataOut(1,lastPosInterp_), poseDataOut(2,lastPosInterp_);
  QTip_ << poseDataOut(3,lastPosInterp_), poseDataOut(4,lastPosInterp_), poseDataOut(5,lastPosInterp_),
           poseDataOut(6,lastPosInterp_);

  PTip2_ = ret1.pTip; // pTip in tipFrame
  QTip2_ = ret1.qTip; // qTip in tipFrame

  // Eigen::Matrix6d J = CTR::GetTipJacobianForTube1(ret1.y_final); //note: this is in tipFrame

  // Setup InputDevice

  // Probably want to plot initial pose of robot before init()
  //ResolvedRates.init(); // or call from outside class?

}

ResolvedRates::~ResolvedRates()
{
  // Handle Deletion of Instance of ResolvedRates class
}

bool ResolvedRates::init(std::string Name)
{
  // Go Online..
}

bool ResolvedRates::setRRGains(double ScaleFactor, double LambdaTracking,
                               double LambdaDamping)
{
  WTracking_ = Eigen::Matrix6d::Zero();
  WTracking_(0,0) = LambdaTracking;
  WTracking_(1,1) = LambdaTracking;
  WTracking_(2,2) = LambdaTracking;
  WTracking_(3,3) = 0.1*LambdaTracking*(180.0/M_PI/2.0)*(180.0/M_PI/2.0);
  WTracking_(4,4) = 0.1*LambdaTracking*(180.0/M_PI/2.0)*(180.0/M_PI/2.0);
  WTracking_(5,5) = 0.1*LambdaTracking*(180.0/M_PI/2.0)*(180.0/M_PI/2.0);

  WDamping_ = Eigen::Matrix6d::Zero();
  double thetadeg = 2.0; // degrees to damp as much as 1mm
  WDamping_(0,0) = LambdaDamping*(180.0/thetadeg/M_PI)*(180.0/thetadeg/M_PI);
  WDamping_(1,1) = LambdaDamping*(180.0/thetadeg/M_PI)*(180.0/thetadeg/M_PI);
  WDamping_(2,2) = LambdaDamping*(180.0/thetadeg/M_PI)*(180.0/thetadeg/M_PI);
  WDamping_(3,3) = LambdaDamping*1.0E6;
  WDamping_(4,4) = LambdaDamping*1.0E6;
  WDamping_(5,5) = LambdaDamping*1.0E6;

  return true; // successfully set WTracking & WDamping

}

bool ResolvedRates::setInputDeviceTransform(Matrix4d TRegistration)
{
  // Do setup we are currently doing for Omni, this should populate InputDeviceReg Transforms -> member variables
}

// Math Methods
double ResolvedRates::deg2rad (double degrees) {
  return degrees * 4.0 * std::atan (1.0) / 180.0;
}
double ResolvedRates::sgn(double x)
{
  double s = (x > 0) - (x < 0);
  return s;
}
double ResolvedRates::vectornorm(Eigen::Vector3d v)
{
  double n = sqrt(v.transpose()*v);
  return n;
}
Eigen::Matrix3d ResolvedRates::orthonormalize(Eigen::Matrix3d R)
{
  Eigen::Matrix3d R_ortho;
  R_ortho.fill(0);
  // Normalize the first column:
  R_ortho.col(0) = R.col(0) / vectornorm(R.col(0));

  // Orthogonalize & normalize second column:
  R_ortho.col(1) = R.col(1);
  double c = (R_ortho.col(1).transpose()*R_ortho.col(0));
  c = c/(R_ortho.col(0).transpose()*R_ortho.col(0));
  R_ortho.col(1) = R_ortho.col(1) - c*R_ortho.col(0);
  R_ortho.col(1) = R_ortho.col(1)/vectornorm(R_ortho.col(1));

  // Orthogonalize & normalize third column:
  R_ortho.col(2) = R.col(2);
  double d = (R_ortho.col(2).transpose()*R_ortho.col(0));
  d = d/(R_ortho.col(0).transpose()*R_ortho.col(0));
  R_ortho.col(2) = R_ortho.col(2) - d*R_ortho.col(0);
  double e = (R_ortho.col(2).transpose()*R_ortho.col(1));
  e = e/(R_ortho.col(1).transpose()*R_ortho.col(1));
  R_ortho.col(2) = R_ortho.col(2) - e*R_ortho.col(1);
  R_ortho.col(2) = R_ortho.col(2)/vectornorm(R_ortho.col(2));
  return R_ortho;
}
Eigen::Matrix4d ResolvedRates::assembleTransformation(Eigen::Matrix3d Rot, Eigen::Vector3d Trans)
{
  Rot = orthonormalize(Rot);
  Eigen::Matrix4d T;
  T.fill(0);
  T.topLeftCorner(3,3) = Rot;
  T.topRightCorner(3,1) = Trans;
  T(3,3) = 1;
  return T;
}
Eigen::Matrix3d ResolvedRates::quat2rotm(Eigen::Vector4d Quat)
{
  // Agrees with Matlab
  // Quaternion order is wxyz
  Eigen::Matrix3d R;
  R.fill(0);

  R(0,0) = pow(Quat(0),2) + pow(Quat(1),2) - pow(Quat(2),2) - pow(Quat(3),2);
  R(0,1) = 2*Quat(1)*Quat(2) - 2*Quat(0)*Quat(3);
  R(0,2) = 2*Quat(1)*Quat(3) + 2*Quat(0)*Quat(2);

  R(1,0) = 2*Quat(1)*Quat(2) + 2*Quat(0)*Quat(3);
  R(1,1) = pow(Quat(0),2) - pow(Quat(1),2) + pow(Quat(2),2) - pow(Quat(3),2);
  R(1,2) = 2*Quat(2)*Quat(3) - 2*Quat(0)*Quat(1);

  R(2,0) = 2*Quat(1)*Quat(3) - 2*Quat(0)*Quat(2);
  R(2,1) = 2*Quat(2)*Quat(3) + 2*Quat(0)*Quat(1);
  R(2,2) = pow(Quat(0),2) - pow(Quat(1),2) - pow(Quat(2),2) + pow(Quat(3),2);
  return R;
}
Eigen::Vector4d ResolvedRates::rotm2quat(Eigen::Matrix3d R)
{
  R = orthonormalize(R);
  Eigen::Vector4d Q;
  Q.fill(0);

  double trace = R(0,0) + R(1,1) + R(2,2);
  if (trace > 0)
  {
      double s = 0.5*sqrt(trace+1.0);
      Q(0) = s; //w eqn
      Q(1) = (R(2,1)-R(1,2))/(4*s); //x eqn
      Q(2) = (R(0,2)-R(2,0))/(4*s); //y eqn
      Q(3) = (R(1,0)-R(0,1))/(4*s); //z eqn
  }
  else
  {
     if(R(0,0)>R(1,1) && R(0,0)>R(2,2))
     {
          double s = 0.5*sqrt(1.0 + R(0,0) - R(1,1) - R(2,2));
          Q(0) = (R(2,1) - R(1,2))*s; //w eqn
          Q(1) = s; //x eqn
          Q(2) = (R(0,1)+R(1,0))*s; //y eqn
          Q(3) = (R(0,2)+R(2,0))*s; //z eqn
      }
     else if (R(1,1)>R(2,2))
     {
          double s = 0.5*sqrt(1.0 + R(1,1) - R(0,0) - R(2,2));
          Q(0) = (R(0,2)-R(2,0))*s; //w eqn
          Q(1) = (R(0,1)+R(1,0))*s; //x eqn
          Q(2) = s; //y eqn
          Q(3) = (R(1,2)+R(2,1))*s; //z eqn
      }
     else
     {
          double s = 0.5*sqrt(1.0 + R(2,2) - R(0,0) - R(1,1));
          Q(0) = (R(1,0)-R(0,1))*s; //w eqn
          Q(1) = (R(0,2)+R(2,0))*s; //x eqn
          Q(2) = (R(1,2)+R(2,1))*s; //y eqn
          Q(3) = s; //z eqn
      }
  }

  return Q;
}
Eigen::Matrix3d ResolvedRates::hat3(Eigen::Vector3d X)
{
  Eigen::Matrix3d xhat;
  xhat.fill(0);
  xhat(0,1) = -X(2);
  xhat(0,2) = X(1);
  xhat(1,0) = X(2);
  xhat(1,2) = -X(0);
  xhat(2,0) = -X(1);
  xhat(2,1) = X(0);
  return xhat;
}
Eigen::Matrix6d ResolvedRates::MAdjoint(Eigen::Matrix4d T)
{
  Eigen::Matrix3d phat = hat3(T.topRightCorner(3,1));
  Eigen::Matrix3d R = T.topLeftCorner(3,3);

  Eigen::Matrix<double,6,6> AdjT;
  AdjT.fill(0);
  AdjT.topLeftCorner(3,3) = R;
  AdjT.topRightCorner(3,3) = phat*R;
  AdjT.bottomRightCorner(3,3) = R;

  return AdjT;
}
Eigen::Matrix6d ResolvedRates::Adjoint_pq(Eigen::Vector3d p, Eigen::Vector4d q)
{
  Eigen::Matrix3d R = quat2rotm(q);
  Eigen::Matrix3d phat = hat3(p);
  Eigen::Matrix6d Ad = Eigen::Matrix6d::Zero();
  Ad.topLeftCorner<3,3>() = R;
  Ad.bottomRightCorner<3,3>() = R;
  Ad.topRightCorner<3,3>() = phat*R;

  return Ad;
}
Eigen::Matrix4d ResolvedRates::inverseTransform(Eigen::Matrix4d T)
{
  Eigen::Matrix4d Tinv;
  Tinv.fill(0);
  Tinv.topLeftCorner(3,3) = T.topLeftCorner(3,3).transpose();
  Tinv.topRightCorner(3,1) = -1.0*T.topLeftCorner(3,3).transpose()*T.topRightCorner(3,1);
  Tinv(3,3) = 1.0;

  return Tinv;
}
Eigen::Matrix7d ResolvedRates::collapseTransform(Eigen::Matrix4d T)
{
  Eigen::Matrix7d X;
  X.fill(0);
  X.head(3) = T.topRightCorner(3,1);
  X.tail(4) = rotm2quat(T.topLeftCorner(3,3));
  return X;
}
Eigen::Vector4d ResolvedRates::slerp(Eigen::Vector4d Qa, Eigen::Vector4d Qb, double t)
{
  Eigen::Vector4d Qm;
  Qm.fill(0);

  double cosHalfTheta = Qa.transpose()*Qb;
  if (abs(cosHalfTheta) >= 1.0)
  {
    Qm = Qa;
    return Qm;
  }

  double halfTheta = acos(cosHalfTheta);
  double sinHalfTheta = sqrt(1.0-cosHalfTheta*cosHalfTheta);

  if (abs(sinHalfTheta) < 0.001)
  {
    Qm(0) = 0.5*Qa(0) + 0.5*Qb(0);
    Qm(1) = 0.5*Qa(1) + 0.5*Qb(1);
    Qm(2) = 0.5*Qa(2) + 0.5*Qb(2);
    Qm(3) = 0.5*Qa(3) + 0.5*Qb(3);
    return Qm;
  }

  double ratioA = sin((1-t)*halfTheta)/sinHalfTheta;
  double ratioB = sin(t*halfTheta) / sinHalfTheta;

  Qm(0) = ratioA*Qa(0) + ratioB*Qb(0);
  Qm(1) = ratioA*Qa(1) + ratioB*Qb(1);
  Qm(2) = ratioA*Qa(2) + ratioB*Qb(2);
  Qm(3) = ratioA*Qa(3) + ratioB*Qb(3);
  return Qm;
}
Eigen::Matrix<double,4,Eigen::Dynamic> ResolvedRates::quatInterp(Eigen::Matrix<double,4,Eigen::Dynamic> refQuat,
                                                                 Eigen::VectorXd refArcLengths,
                                                                 Eigen::VectorXd interpArcLengths)
{
  int count = 0;
  int N = interpArcLengths.size();
  Eigen::MatrixXd quatInterpolated(4,N);
  quatInterpolated.fill(0);

  for(int i=0; i<N; i+=1)
  {
      if(interpArcLengths(i) < refArcLengths(count+1)){
          count = count+1;
      }
      double L = refArcLengths(count) - refArcLengths(count+1);
      double t = (refArcLengths(count)-interpArcLengths(i))/L;
      quatInterpolated.col(i) = slerp(refQuat.col(count), refQuat.col(count+1), t);
  }

  return quatInterpolated;
}
void ResolvedRates::getCofactor(double A[6][6], double Temp[6][6], int P, int Q, int N)
{
  int i = 0, j = 0;

  // Looping for each element of the matrix
  for (int row = 0; row < N; row++)
  {
    for (int col = 0; col < N; col++)
    {
      //  Copying into temporary matrix only those element
      //  which are not in given row and column
      if (row != P && col != Q)
      {
        Temp[i][j++] = A[row][col];

        // Row is filled, so increase row index and
        // reset col index
        if (j == N - 1)
        {
          j = 0;
          i++;
        }
      }
    }
  }
}
double ResolvedRates::determinant(double A[6][6], int N)
{
  double D = 0; // Initialize result

  //  Base case : if matrix contains single element
  if (n == 1)
    return A[0][0];

  double temp[6][6]; // To store cofactors

  int sign = 1;  // To store sign multiplier

  // Iterate for each element of first row
  for (int f = 0; f < N; f++)
  {
    // Getting Cofactor of A[0][f]
    getCofactor(A, temp, 0, f, N);
    D += sign * A[0][f] * determinant(temp, N - 1);

    // terms are to be added with alternate sign
    sign = -sign;
  }

  return D;
}
void ResolvedRates::adjoint(double A[6][6], double Adj[6][6])
{
  // temp is used to store cofactors of A[][]
  int sign = 1;
  double temp[6][6];

  for (int i=0; i<6; i++)
  {
    for (int j=0; j<6; j++)
    {
      // Get cofactor of A[i][j]
      getCofactor(A, temp, i, j, 6);

      // sign of adj[j][i] positive if sum of row
      // and column indexes is even.
      sign = ((i+j)%2==0)? 1: -1;

      // Interchanging rows and columns to get the
      // transpose of the cofactor matrix
      Adj[j][i] = (sign)*(determinant(temp, 6-1));
    }
  }
}
void ResolvedRates::inverse(double A[6][6], double Inverse[6][6])
{
  // Find determinant of A[][]
  double det = determinant(A, 6);
  if (det != 0) //matrix is not singular
  {
    // Find adjoint
    double adj[6][6];
    adjoint(A, adj);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (int i=0; i<6; i++)
      for (int j=0; j<6; j++)
        Inverse[i][j] = adj[i][j]/double(det);
  }
}

// Robotics Methods
auto ResolvedRates::defineRobot(CTR3RobotParams params)
{
  constant_fun< Vector<2>::type > k_fun1( (1.0/params.k1)*Eigen::Vector2d::UnitX() );
  constant_fun< Vector<2>::type > k_fun2( (1.0/params.k2)*Eigen::Vector2d::UnitX() );
  constant_fun< Vector<2>::type > k_fun3( (1.0/params.k3)*Eigen::Vector2d::UnitX() );

  TubeType T1 = make_annular_tube( params.L1, params.Lt1, params.OD1, params.ID1,
                                  k_fun1, params.E, params.G);
  TubeType T2 = make_annular_tube( params.L2, params.Lt2, params.OD2, params.ID2,
                                  k_fun2, params.E, params.G);
  TubeType T3 = make_annular_tube( params.L3, params.Lt3, params.OD3, params.ID3,
                                  k_fun3, params.E, params.G);

  // cannula is input to Kinematics_with_dense_output()
  auto cannula = std::make_tuple( T1, T2, T3);

  return cannula;
}
auto ResolvedRates::callKinematics(CTR3ModelStateVector newStateVector)
{
    Eigen::Vector6d Q;
    Q.topRows(3) = newStateVector.PsiL_;
    Q.bottomRows(3) = newStateVector.Beta_;

    auto cannula = defineRobot(CTR3RobotParams); // TODO: save this to class so we don't define here everytime

    auto ret1 = Kinematics_with_dense_output( cannula, Q, OType() );
    int nPts = ret1.arc_length_points.size();
    double* ptr = &ret1.arc_length_points[0];
    Eigen::Map<Eigen::VectorXd> s(ptr,nPts);

    // Forward Kinematics (Results in Base Frame)
    auto poseData = forwardKinematics(ret1); // TODO: save this to class

}
Eigen::MatrixXd ResolvedRates::forwardKinematics(auto kin)
{
  int nPts = kin.arc_length_points.size();
  double* ptr = &kin.arc_length_points[0];
  Eigen::Map<Eigen::VectorXd> s(ptr,nPts);
  Eigen::VectorXd sAbs(nPts);
  for (int i = 0; i < nPts; i++)
  {
    sAbs(i) = fabs(s(i));
  }

  Eigen::MatrixXd pos(3,nPts);
  Eigen::MatrixXd quat(4,nPts);
  Eigen::MatrixXd psiAngles(3,nPts);
  for (int j = 0; j < nPts; j++)
  {
    double* pPtr = &ret1.dense_state_output[j].p[0];
    double* qPtr = &ret1.dense_state_output[j].q[o];
    double* psiPtr = &ret1.dense_state_output[j].Psi[o];
    Eigen::Map<Eigen::Vector3d> pj(pPtr,3);
    Eigen::Map<Eigen::Vector4d> qj(qPtr,4);
    Eigen::Map<Eigen::Vector3d> psij(psiPtr,3);

    pos.col(j) = pj;
    quat.col(j) = qj;
    psiAngles.col(j) = psij;
  }

  int basePlateIdx;
  sAbs.minCoeff(&basePlateIdx);

  Eigen::Matrix3d Rbt;
  Eigen::Vector3d pbt;
  Matrix4d Tbt;
  Eigen::Matrix3d Rjt;
  Matrix4d Tjt;
  Matrix4d Tjb;
  Vector7d tjb;
  Vector8d x;

  Eigen::Vector3d baseRotations;
  baseRotations << psiAngles(0,basePlateIdx), psiAngles(1,basePlateIdx), psiAngles(2,basePlateIdx);

  Rbt = quat2rotm(quat.col(nPts-1));
  pbt = pos.col(nPts-1) - qHome.Beta_(0)*Eigen::Vector3d::UnitZ();
  Tbt = assembleTransformation(Rbt,pos.col(nPts-1));

  Eigen::MatrixXd poseData(8,nPts);
  for (int j = 0; j < nPts; j++)
  {
    Rjt = quat2rotm(quat.col(nPts-j-1));
    Tjt = assembleTransformation(Rjt,pos.col(nPts-j-1));
    Tjb = inverseTransform(Tbt)*Tjt;
    tjb = collapseTransform(Tjb);

    x.fill(0);
    x.head<7>() = tjb;
    x(7) = 1.0;
    poseData.col(j) = x;
  }
  return poseData;
}
InterpRet ResolvedRates::interpolateBackbone(Eigen::VectorXd sRef, Eigen::MatrixXd poseDataRef, int nPts)
{
  Eigen::Matrix<double,4,Eigen::Dynamic> qRef;
  qRef = poseDataRef.middleRows<4>(3);

  // Create a zero to one list for ref arc lengths
  int nRef = sRef.size();
  double totalArcLength = sRef(nRef-1) - sRef(0);
  Eigen::VectorXd sRef0Vec(nRef);
  sRef0Vec.fill(sRef(0));
  Eigen::VectorXd zeroToOne = (1/totalArcLength)*(sRef - sRef0Vec);

  // Create a zero to one vector including ref arc lengths & interp arc lengths (evenly spaced)
  int nPtsTotal = nPts+nRef;
  lastPosInterp_ = nPtsTotal - 1;

  Eigen::VectorXd xxLinspace(nPts);
  xxLinspace.fill(0.0);
  xxLinspace.setLinSpaced(npts,0.0,1.0);
  Eigen::VectorXd xxUnsorted(nPtsTotal);
  xxUnsorted << xxLinspace, zeroToOne;
  std::sort(xxUnsorted.data(),xxUnsorted.data+xxUnsorted.size());
  Eigen::VectorXd xx = xxUnsorted.reverse(); //Rich's interpolation functions call for descending order

  // List of return arc lengths in the original scaling/offset
  Eigen::VectorXd xxSRef0Vec(nPtsTotal);
  xxSRef0Vec.fill(sRef(0));
  Eigen::VectorXd sInterp = totalArcLength*xx.reverse()+xxSRef0Vec;

  // Interpolate to find list of return quaternions
  Eigen::MatrixXd qInerp1 = quatInterp(qRef.rowwise().reverse(),zeroToOne.reverse(),xx);
  Eigen::MatrixXd qInterp = qInterp1.rowwise().reverse();

  // Interpolate to find list of return positions
  std::vector<double> sVec;
  sVec.resize(sRef.size());
  Eigen::VectorXd::Map(&sVec[0],sRef.size()) = sRef;

  Eigen::VectorXd X = poseDataRef.row(0); // interp x
  std::vector<double> xVec;
  xVec.resize(X.size());
  Eigen::VectorXd::Map(&xVec[0],x.size()) = X;
  tk::spline Sx;
  Sx.set_points(sVec,xVec);
  Eigen::VectorXd xInterp(nPtsTotal);
  xInterp.fill(0);
  for (int i = 0; i < nPtsTotal; i++)
  {
    xInterp(i) = Sx(sInterp(i));
  }

  Eigen::VectorXd Y = poseDataRef.row(1); // interp y
  std::vector<double> yVec;
  yVec.resize(Y.size());
  Eigen::VectorXd::Map(&yVec[0],Y.size()) = Y;
  tk::spline Sy;
  Sy.set_points(sVec,yVec);
  Eigen::VectorXd yInterp(nPtsTotal);
  yInterp.fill(0);
  for (int i = 0; i < nPtsTotal; i++)
  {
    yInterp(i) = Sy(sInterp(i));
  }

  Eigen::VectorXd Z = poseDataRef.row(2); // interp z
  std::vector<double> zVec;
  zVec.resize(Z.size());
  Eigen::VectorXd::Map(&zVec[0],Z.size()) = Z;
  tk::spline Sz;
  Sz.set_points(sVec,zVec);
  Eigen::VectorXd zInterp(nPtsTotal);
  for (int i = 0; i < nPtsTotal; i++)
  {
    zInterp(i) = Sz(sInterp(i));
  }

  Eigen::MatrixXd pInterp(3,nPtsTotal);
  pInterp.fill(0);
  pInterp.row(0) = xInterp.transpose();
  pInterp.row(1) = yInterp.transpose();
  pInterp.row(2) = zInterp.transpose();

  InterpRet interpResults;
  interpResults.s = sInterp;
  interpResults.p = pInterp;
  interpResults.q = qInterp;

  return interpResults;

}
Vector6d ResolvedRates::saturateJointVelocities(Vector6d DeltaQx, int NodeFreq)
{
  double max_rot_speed = 0.8; // rad/sec
  double max_trans_speed = 5.0; // mm/sec

  Vector6d commanded_speed = DeltaQx*NodeFreq;
  Vector6d saturated_speed = commanded_speed;
  Vector6d delta_qx_sat;

  // saturate tip rotations
  for (int i=0; i<3; i++)
  {
    if(fabs(commanded_speed(i)) > max_rot_speed)
    {
      saturated_speed(i) = (commanded_speed(i)/fabs(commanded_speed(i)))*max_rot_speed;
      std::cout << "Tube rotation speed saturated for tube" << i << std::endl << std::endl;
    }
    delta_qx_sat(i) = saturated_speed(i)/NodeFreq;
  }

  // saturate translations
  for (int i=3; i<6; i++)
  {
    if(fabs(commanded_speed(i)) > max_trans_speed)
    {
      saturated_speed(i) = (commanded_speed(i)/fabs(commanded_speed(i)))*max_trans_speed;
      std::cout << "Tube translation speed saturated for tube" << i << std::endl << std::endl;
    }
    delta_qx_sat(i) = saturated_speed(i)/node_freq;
  }

  return delta_qx_sat;
}
Vector6d ResolvedRates::transformBetaToX(Vector6d Qbeta, Eigen::Vector3d L)
{
  Vector6d qx;
  qx << Qbeta(0), Qbeta(1), Qbeta(2), 0, 0, 0;
  qx(3) = L(0) - L(1) + Qbeta(3) - Qbeta(4);
  qx(4) = L(1) - L(2) + Qbeta(4) - Qbeta(5);
  qx(5) = L(2) + Qbeta(5);
  return qx;
}
Vector6d ResolvedRates::transformXToBeta(Vector6d Qx, Eigen::Vector3d L)
{
  Vector6d qbeta;
  qbeta << Qx(0), Qx(1), Qx(2), 0, 0, 0;
  qbeta(3) = Qx(3) + Qx(4) + Qx(5) - L(0);
  qbeta(4) = Qx(4) + Qx(5) - L(1);
  qbeta(5) = Qx(5) - L(2);
  return qbeta;
}
double ResolvedRates::dhFunction(double Xmin, double Xmax, double X)
{
  double dh = fabs((Xmax-Xmin)*(Xmax-Xmin)*(2*X-Xmax-Xmin)/(4*(Xmax-X)*(Xmax-X)*(X-Xmin)*(X-Xmin)));

  return dh;
}
Eigen::Vector3d ResolvedRates::limitBetaValsSimple(Eigen::Vector3d XIn, Eigen::Vector3d L)
{
  Eigen::Vector3d x = XIn;
  double epsilon = 0.5e-3;  // keep a 0.5 mm minimum margin

  // check tube 3 first:
  if (x(2) < epsilon)
  {
    x(2) = epsilon;
    std::cout << "Tube 3 translation saturated (front)" << std::endl;
  }
  else if (x(2) > L(2)-epsilon)
  {
    x(2) = L(2)-epsilon;
    std::cout << "Tube 3 translation saturated (rear)" << std::endl;
  }

  // now check tube 2:
  if (x(1) < epsilon)
  {
    x(1) = epsilon;
    std::cout << "Tube 2 translation saturated (front)" << std::endl;
  }
  else if (x(1) > L(1)-L(2)-epsilon)
  {
    x(1) = L(1)-L(2)-epsilon;
    std::cout << "Tube 2 translation saturated (rear)" << std::endl;
  }

  // and last check tube 1:
  if (x(0) < epsilon)
  {
    x(0) = epsilon;
    std::cout << "Tube 1 translation saturated (front)" << std::endl;
  }
  else if (x(0) > L(0)-L(1)-epsilon)
  {
    x(0)=L(0)-L(1)-epsilon;
    std::cout << "Tube 1 translation saturated (rear)" << std::endl;
  }

  return x;
}
Eigen::Vector3d ResolvedRates::limitBetaValsHardware(Eigen::Vector3d BetaIn)
{
  Eigen::Matrix<double,3,2> enc_lims;
  enc_lims << 50000, -145000, 	// inner
      100000, -55000,		// middle
      83000, -17000;		// outer

  Eigen::Vector3d enc_in;
  enc_in << (int)((BetaIn(0) - qstart.Beta(0)) * scale_trans),  (int)((BetaIn(1) - qstart.Beta(1)) * scale_trans),  (int)((BetaIn(2) - qstart.Beta(2)) * scale_trans_outer);
  Eigen::Vector3d enc = enc_in;

  // check tube 3 first:
  if (enc(2) < enc_lims(2,1))
  {
    enc(2) = enc_lims(2,1);
    std::cout << "Tube 3 translation saturated (hardware)" << std::endl;
  }
  else if (enc(2) > enc_lims(2,0))
  {
    enc(2) = enc_lims(2,0);
    std::cout << "Tube 3 translation saturated (hardware)" << std::endl;
  }

  // now check tube 2:
  if (enc(1) < enc_lims(1,1))
  {
    enc(1) = enc_lims(1,1);
    std::cout << "Tube 2 translation saturated (hardware)" << std::endl;
  }
  else if (enc(1) > enc_lims(1,0))
  {
    enc(1) = enc_lims(1,0);
    std::cout << "Tube 2 translation saturated (hardware)" << std::endl;
  }

  // and last check tube 1:
  if (enc(0) < enc_lims(0,1))
  {
    enc(0) = enc_lims(0,1);
    std::cout << "Tube 1 translation saturated (hardware)" << std::endl;
  }
  else if (enc(0) > enc_lims(0,0))
  {
    enc(0)=enc_lims(0,0);
    std::cout << "Tube 1 translation saturated (hardware)" << std::endl;
  }

  Eigen::Vector3d beta;
  beta << enc(0)/scale_trans + qstart.Beta(0), enc(1)/scale_trans + qstart.Beta(1), enc(2)/scale_trans + qstart.Beta(2);
  return beta;
}
Eigen::Vector3d ResolvedRates::limitBetaValsBimanualAlgorithm(Eigen::Vector3d BetaIn, Eigen::Vector3d LIn)
{
  int nTubes = 3;

  // plate thicknesses (constant)
  // eventually we'll want to load them from elsewhere
  // these also need to be updated to correct values for the new robot
  // and this whole algorithm will need to be adapted
  Eigen::Matrix<double,4,1> tplus;
  tplus << 0.0, 10e-3, 10e-3, 10e-3;

  Eigen::Matrix<double,4,1> tminus;
  tminus << 0.0, 10e-3, 10e-3, 10e-3;

  Eigen::Matrix<double,4,1> extMargin;
  extMargin << 1e-3, 1e-3, 10e-3, 1e-3;
  // tube 2 is different because we have to make sure it doesn't knock off the end effector

  double tp = 25e-3;

  Eigen::Matrix<double,4,1> L;
  L << 3*LIn(0), LIn(0), LIn(1), LIn(2);

  Eigen::Matrix<double,4,1> beta;
  beta << -2*LIn(0), BetaIn(0), BetaIn(1), BetaIn(2);

  // starting with tube 1, correct any translational joint limit violations
  for (int i = 1; i <=nTubes; i++)
  {
    // if it's going to hit the carriage behind it, move it up:
    if (beta(i) - tminus(i) < beta(i-1) + tplus(i-1))
    {
      beta(i) = beta(i-1) + tplus(i-1) + tminus(i);
    }

    // if it's going to not leave space for the other carriages in front of it, move it back:
    double partial_sum = 0.0;
    for (int k = i+1; k <= nTubes; k++)
    {
      partial_sum += tplus(k) + tminus(k);
    }
    if (beta(i) + tplus(i) > -tp - partial_sum)
    {
      beta(i) = -tp - partial_sum - tplus(i);
    }

    // if the tube is getting too close to the end of the next tube out, move it up:
    if (beta(i) + L(i) > beta(i-1) + L(i-1) - extMargin(i))
    {
      beta(i) = beta(i-1) + L(i-1) - L(i) - extMargin(i);
    }

    // if the tube is going to retract all the way behind the front plate, move it up:
    if (beta(i) + L(i) - 0.001*(nTubes-i+1) < 0.0)
    {
      beta(i) = -L(i) + 0.001*(nTubes-i+1);
    }
  }

  Eigen::Vector3d Beta;
  Beta << beta(1), beta(2), beta(3);
  return Beta;
}
double ResolvedRates::getVMag(const Eigen::VectorXd& E, double VMax, double VMin, double EMax, double EMin,
               double ConvergeRadius)
{
  double vMag;
  double eNorm = E.norm();
  if (eNorm < ConvergeRadius) {
    vMag = 0.0;
  }
  else if (eNorm >= ConvergeRadius && eNorm < EMin) {
    vMag = VMin;
  }
  else if (eNorm >= EMin && eNorm < EMax) {
    double m = (VMax - VMin) / (EMax - EMin);
    vMag = m*(eNorm - EMax) + VMax;
  }
  else {
    vMag = VMax;
  }

  return vMag;
}
WeightingRet ResolvedRates::getWJointLims(Eigen::Vector3d X,
                                          Eigen::Vector3d dhPrev,
                                          Eigen::Vector3d L, double lambda)
{
  Eigen::Matrix<double,6,6> W = MatrixXd::Identity(6,6);

  // No penalties on the rotational degrees of freedom (they don't have any joint limits)
  // Therefore leave the first three entries in W as 1.

  double eps = 2e-3;

  // x1:
  double x1min = eps;
  double x1max = L(0)-L(1)-eps;
  double x1 = X(0);
  double dh1 = dhFunction(x1min,x1max,x1);
  W(3,3) = (dh1 >= dhPrev(0))*(1+dh1) + (dh1 < dhPrev(0))*1;
  //W(3,3) = 1+dh1;
  W(3,3) *= lambda;

  // x2:
  double x2min = eps;
  double x2max = L(1)-L(2)-eps;
  double x2 = X(1);
  double dh2 = dhFunction(x2min,x2max,x2);
  W(4,4) = (dh2 >= dhPrev(1))*(1+dh2) + (dh2 < dhPrev(1))*1;
  //W(4,4) = 1+dh2;
  W(4,4) *= lambda;

  // x3:
  double x3min = eps;
  double x3max = L(2)-eps;
  double x3 = X(2);
  double dh3 = dhFunction(x3min,x3max,x3);
  W(5,5) = (dh3 >= dhPrev(2))*(1+dh3) + (dh3 < dhPrev(2))*1;
  //W(5,5) = 1+dh3;
  W(5,5) *= lambda;

  Eigen::Vector3d dh;
  dh << dh1,dh2,dh3;

  weightingRet output;
  output.W = W;
  output.dh = dh;
  return output;
}
Eigen::Vector3d ResolvedRates::scaleInputDeviceVelocity(Vector3d DesTwistDelta)
{
  Vector3d scaledDesTwistDelta = DesTwistDelta;

  //double pStepMax = 2.0e-3; // 2 mm
  double vMax = 0.2; // [m/s]
  double pStepMax = vMax / rosLoopRate; // [mm]

  double pErr = DesTwistelta.topRows<3>().norm();
  double gain = 1.0;
  if (pErr > pStepMax)
  {
    gain = 	pStepMax / pErr;
  }
  scaledDesTwistDelta *= gain;

  return scaledDesTwistDelta;
}
