#include "resolvedrates.h"

ResolvedRates::ResolvedRates()
{

}

bool ResolvedRates::init(std::string Name)
{
  // do initialization stuff
  // -> spec robot & starting config, run kinematics the first time, set backbone, interp backbone, etc.
}

bool ResolvedRates::setRRGains(double ScaleFactor, double LambdaTracking,
                               double LambdaDamping, double LambdaJointLims)
{
  // set weighting matrices WTracking_ WDamping_ WJointLims_ -> these are member variables of ResolvedRates class
}

bool ResolvedRates::setInputDeviceTransform(Matrix4d TRegistration)
{
  // Do setup we are currently doing for Omni, this should populate InputDeviceReg Transforms -> member variables
}

// Math Methods
double deg2rad (double degrees) {
  return degrees * 4.0 * atan (1.0) / 180.0;
}
double vectornorm(Eigen::Vector3d v)
{
  double n = sqrt(v.transpose()*v);
  return n;
}
Eigen::Matrix3d orthonormalize(Eigen::Matrix3d R)
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
Eigen::Matrix4d assembleTransformation(Eigen::Matrix3d Rot, Eigen::Vector3d Trans)
{
  Rot = orthonormalize(Rot);
  Eigen::Matrix4d T;
  T.fill(0);
  T.topLeftCorner(3,3) = Rot;
  T.topRightCorner(3,1) = Trans;
  T(3,3) = 1;
  return T;
}
Eigen::Matrix3d quat2rotm(Eigen::Vector4d Quat)
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
Eigen::Matrix3d hat3(Eigen::Vector3d X)
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
Eigen::Matrix6d MAdjoint(Eigen::Matrix4d T)
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
void getCofactor(double A[6][6], double Temp[6][6], int P, int Q, int N)
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
double determinant(double A[6][6], int N)
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
void adjoint(double A[6][6], double Adj[6][6])
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
void inverse(double A[6][6], double Inverse[6][6])
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
auto forwardKinematics(auto kin);

Vector6d saturateJointVelocities(Vector6d DeltaQx, int NodeFreq)
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
Vector6d transformBetaToX(Vector6d Qbeta, Eigen::Vector3d L)
{
  Vector6d qx;
  qx << Qbeta(0), Qbeta(1), Qbeta(2), 0, 0, 0;
  qx(3) = L(0) - L(1) + Qbeta(3) - Qbeta(4);
  qx(4) = L(1) - L(2) + Qbeta(4) - Qbeta(5);
  qx(5) = L(2) + Qbeta(5);
  return qx;
}
Vector6d transformXToBeta(Vector6d Qx, Eigen::Vector3d L)
{
  Vector6d qbeta;
  qbeta << Qx(0), Qx(1), Qx(2), 0, 0, 0;
  qbeta(3) = Qx(3) + Qx(4) + Qx(5) - L(0);
  qbeta(4) = Qx(4) + Qx(5) - L(1);
  qbeta(5) = Qx(5) - L(2);
  return qbeta;
}
double dhFunction(double Xmin, double Xmax, double X)
{
  double dh = fabs((Xmax-Xmin)*(Xmax-Xmin)*(2*X-Xmax-Xmin)/(4*(Xmax-X)*(Xmax-X)*(X-Xmin)*(X-Xmin)));

  return dh;
}
Eigen::Vector3d limitBetaValsSimple(Eigen::Vector3d XIn, Eigen::Vector3d L)
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
Eigen::Vector3d limitBetaValsHardware(Eigen::Vector3d BetaIn)
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
Eigen::Vector3d limitBetaValsBimanualAlgorithm(Eigen::Vector3d BetaIn, Eigen::Vector3d LIn)
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
double getVMag(const Eigen::VectorXd& E, double VMax, double VMin, double EMax, double EMin,
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
WeightingRet getWeightingMatrix(Eigen::Vector3d X,
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
Eigen::Vector3d scaleInputDeviceVelocity(Vector3d DesTwistDelta)
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
