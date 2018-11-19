/********************************************************************

  kinematics.cpp

3-tube concentric tube robot kinematics node.
Subscribes to joint value command messages from resolved rates node.
Publishes Jacobian and tip poses for resolved rates, and discretized markers for RVIZ.

Andria Remirez & Lin Liu
revised:  6/14/2018
********************************************************************/

// HEADERS

// Qt headers
#include <QCoreApplication>
#include <QVector>

// Cannula kinematics headers
#include "Kinematics.h"
#include "BasicFunctions.h"
#include "Tube.h"
#include "Utility.h"

// Eigen headers
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS headers
#include <ros/ros.h>

// Message & service headers
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include "std_msgs/Bool.h"
#include <endonasal_teleop/matrix8.h>
#include <endonasal_teleop/config3.h>
#include <endonasal_teleop/matrix6.h>
#include <endonasal_teleop/vector7.h>
#include <endonasal_teleop/kinout.h>
#include "endonasal_teleop/getStartingConfig.h"
#include "endonasal_teleop/getStartingKin.h"

// Misc. headers
#include <iostream>
#include <random>
#include <vector>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "spline.h"
#include <cmath>

// NAMESPACES
using namespace CTR;
using namespace CTR::Functions;
using namespace std;
using std::tuple;
using Eigen::Vector2d;

//TYPEDEFS
typedef Eigen::Matrix<double,4,4> Matrix4d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef tuple < Tube< constant_fun< Vector2d > >,
   Tube< constant_fun< Vector2d > >,
   Tube< constant_fun< Vector2d > > > Cannula3;
typedef constant_fun<Eigen::Vector2d> CurvFun;
typedef std::tuple< Tube<CurvFun>, Tube<CurvFun>, Tube<CurvFun> > CannulaT;
typedef DeclareOptions< Option::ComputeJacobian, Option::ComputeGeometry, Option::ComputeStability, Option::ComputeCompliance>::options OType;

struct interpRet
{
    Eigen::VectorXd s;
    Eigen::MatrixXd p;
    Eigen::MatrixXd q;
};

struct Configuration3
{
   Eigen::Vector3d  PsiL;
   Eigen::Vector3d  Beta;
   Eigen::Vector3d  Ftip;
   Eigen::Vector3d  Ttip;
};



// GLOBAL VARIABLES NEEDED FOR KINEMATICS
double rosLoopRate = 200.0;
std_msgs::Bool kinUpdateStatusMsg;
endonasal_teleop::matrix8 markers_msg;
endonasal_teleop::kinout kin_msg;
bool startingConfigPublished;

Eigen::Vector3d ptip;
Eigen::Vector4d qtip;
Eigen::Vector4d qBishop;
Eigen::Matrix<double,3,3> RBishop;
Eigen::Matrix<double,3,3> Rtip;
Matrix6d J;
Matrix6d Jbody;
double Stability;

// SERVICE CALL FUNCTION DEFINITION ----------------
bool startingKin(endonasal_teleop::getStartingKin::Request &req, endonasal_teleop::getStartingKin::Response &res)
{

    std::cout << "Retrieving the starting pose & Jacobian..." << std::endl << std::endl;

    for (int i=0; i<6; i++)
    {
        res.J1[i] = J(0,i);
        res.J2[i] = J(1,i);
        res.J3[i] = J(2,i);
        res.J4[i] = J(3,i);
        res.J5[i] = J(4,i);
        res.J6[i] = J(5,i);
    }

    for (int i = 0; i <3; i++)
    {
        res.p[i] = ptip(i);
    }

    for (int i = 0; i <4; i++)
    {
        res.q[i] = qtip(i);
        res.qb[i] = qBishop(i);
    }

    res.Stability = Stability;


    return true;
}

endonasal_teleop::matrix8 Arr;
// Number of points/frames
int length=0;
bool new_q_msg=0;

double deg2rad (double degrees)
{
    return degrees * 4.0 * atan (1.0) / 180.0;
}

double vectornorm(Eigen::Vector3d v)
{
    return sqrt(v.transpose()*v);
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
    // Assuming convention q = [w x y z] (agrees w/Matlab)
    Eigen::Matrix3d R;
    R.fill(0);

    R(0,0) = 1 - 2*pow(Quat(2),2) - 2*pow(Quat(3),2);
    //R(0,0) = pow(Quat(0),2) + pow(Quat(1),2) - pow(Quat(2),2) - pow(Quat(3),2);
    R(0,1) = 2*Quat(1)*Quat(2) - 2*Quat(0)*Quat(3);
    R(0,2) = 2*Quat(1)*Quat(3) + 2*Quat(0)*Quat(2);

    R(1,0) = 2*Quat(1)*Quat(2) + 2*Quat(0)*Quat(3);
    //R(1,1) = pow(Quat(0),2) - pow(Quat(1),2) + pow(Quat(2),2) - pow(Quat(3),2);
    R(1,1) = 1 - 2*pow(Quat(1),2) - 2*pow(Quat(3),2);
    R(1,2) = 2*Quat(2)*Quat(3) - 2*Quat(0)*Quat(1);

    R(2,0) = 2*Quat(1)*Quat(3) - 2*Quat(0)*Quat(2);
    R(2,1) = 2*Quat(2)*Quat(3) + 2*Quat(0)*Quat(1);
    //R(2,2) = pow(Quat(0),2) - pow(Quat(1),2) - pow(Quat(2),2) + pow(Quat(3),2);
    R(2,2) = 1 - 2*pow(Quat(1),2) - 2*pow(Quat(2),2);

    return R;
}

Eigen::Matrix3d hat3(Eigen::Vector3d v)
{
    Eigen::Matrix3d H = Eigen::Matrix<double,3,3>::Zero();
    H(0,1) = -1*v(2);
    H(0,2) = v(1);
    H(1,0) = v(2);
    H(1,2) = -1*v(0);
    H(2,0) = -1*v(1);
    H(2,1) = v(0);

    return H;
}


Eigen::Matrix<double,6,6> Adjoint_p_q(Eigen::Vector3d p, Eigen::Vector4d q)
{
    Eigen::Matrix3d R = quat2rotm(q);
    Eigen::Matrix3d phat = hat3(p);
    Eigen::Matrix<double,6,6> Ad = Eigen::Matrix<double,6,6>::Zero();
    Ad.topLeftCorner<3,3>() = R;
    Ad.bottomRightCorner<3,3>() = R;
    Ad.topRightCorner<3,3>() = phat*R;

    return Ad;
}

Eigen::Matrix4d inverseTransform(Eigen::Matrix4d T)
{
    Eigen::Matrix4d Tinv;
    Tinv.fill(0);
    Tinv.topLeftCorner(3,3) = T.topLeftCorner(3,3).transpose();
    Tinv.topRightCorner(3,1) = -1*T.topLeftCorner(3,3).transpose()*T.topRightCorner(3,1);
    Tinv(3,3) = 1.0;
    return Tinv;
}

double sgn(double x)
{
    double s = (x > 0) - (x < 0);
    return s;
}

Eigen::Vector4d rotm2quat(Eigen::Matrix3d R)
{
    R = orthonormalize(R);
    Eigen::Vector4d Q;
    Q.fill(0);

    double trace = R(0,0) + R(1,1) + R(2,2);
    if (trace > 0)
    {
        double s = 2*sqrt(trace+1.0);
        Q(0) = 0.25*s; //w eqn
        Q(1) = (R(2,1)-R(1,2))/s; //x eqn
        Q(2) = (R(0,2)-R(2,0))/s; //y eqn
        Q(3) = (R(1,0)-R(0,1))/s; //z eqn
    }
    else
    {
       if(R(0,0)>R(1,1) && R(0,0)>R(2,2))
       {
            double s = 2*sqrt(1.0 + R(0,0) - R(1,1) - R(2,2));
            Q(0) = (R(2,1) - R(1,2))/s; //w eqn
            Q(1) = 0.25*s; //x eqn
            Q(2) = (R(0,1)+R(1,0))/s; //y eqn
            Q(3) = (R(0,2)+R(2,0))/s; //z eqn
        }
       else if (R(1,1)>R(2,2))
       {
            double s = 2*sqrt(1.0 + R(1,1) - R(0,0) - R(2,2));
            Q(0) = (R(0,2)-R(2,0))/s; //w eqn
            Q(1) = (R(0,1)+R(1,0))/s; //x eqn
            Q(2) = 0.25*s; //y eqn
            Q(3) = (R(1,2)+R(2,1))/s; //z eqn
        }
       else
       {
            double s = 2*sqrt(1.0 + R(2,2) - R(0,0) - R(1,1));
            Q(0) = (R(1,0)-R(0,1))/s; //w eqn
            Q(1) = (R(0,2)+R(2,0))/s; //x eqn
            Q(2) = (R(1,2)+R(2,1))/s; //y eqn
            Q(3) = 0.25*s; //z eqn
        }
    }

    return Q;
}

Eigen::Matrix<double,7,1> collapseTransform(Eigen::Matrix4d T)
{
    Eigen::Matrix<double,7,1> x;
    x.fill(0);
    x.head(3) = T.topRightCorner(3,1);
    x.tail(4) = rotm2quat(T.topLeftCorner(3,3));
    return x;
}

Eigen::Vector4d slerp(Eigen::Vector4d qa, Eigen::Vector4d qb, double t)
{
    Eigen::Vector4d qm;
    qm.fill(0);

    double cosHalfTheta = qa.transpose()*qb;
    if(abs(cosHalfTheta) >= 1.0)
    {
        qm = qa;
        return qm;
    }

    double halfTheta = acos(cosHalfTheta);
    double sinHalfTheta = sqrt(1.0-cosHalfTheta*cosHalfTheta);

    if(abs(sinHalfTheta)<0.001)
    {
        qm(0) = 0.5*qa(0) + 0.5*qb(0);
        qm(1) = 0.5*qa(1) + 0.5*qb(1);
        qm(2) = 0.5*qa(2) + 0.5*qb(2);
        qm(3) = 0.5*qa(3) + 0.5*qb(3);
        return qm;
    }

    double ratioA = sin((1-t)*halfTheta)/sinHalfTheta;
    double ratioB = sin(t*halfTheta) / sinHalfTheta;

    qm(0) = ratioA*qa(0) + ratioB*qb(0);
    qm(1) = ratioA*qa(1) + ratioB*qb(1);
    qm(2) = ratioA*qa(2) + ratioB*qb(2);
    qm(3) = ratioA*qa(3) + ratioB*qb(3);
    return qm;
}

Eigen::Matrix<double,4,Eigen::Dynamic> quatInterp(Eigen::Matrix<double,4,Eigen::Dynamic> refQuat, Eigen::VectorXd refArcLengths, Eigen::VectorXd interpArcLengths)
{
    int count = 0;
    int N = interpArcLengths.size();
    Eigen::MatrixXd quatInterpolated(4,N);
    quatInterpolated.fill(0);

    for(int i=0; i<N; i++)
    {
        // setLinSpaced sometimes returns very small negative number (-1e-15) instead of 0
        if(interpArcLengths(i) < 1e-10) {
          interpArcLengths(i) = 0.0;
        }
        if(interpArcLengths(i) < refArcLengths(count+1)){
            count = count+1;
        }
        double L = refArcLengths(count) - refArcLengths(count+1);
        double t = (refArcLengths(count)-interpArcLengths(i))/L;
        quatInterpolated.col(i) = slerp(refQuat.col(count), refQuat.col(count+1), t);
    }
    return quatInterpolated;

}

interpRet interpolateBackbone(Eigen::VectorXd s_ref, Eigen::MatrixXd posedata_ref, int npts)
{
    Eigen::Matrix<double,4,Eigen::Dynamic> q_ref;
    q_ref = posedata_ref.middleRows<4>(3);

    // Create a zero to one list for ref arc lengths;
    int Nref = s_ref.size();
    double totalArcLength = s_ref(0) - s_ref(Nref-1);
    Eigen::VectorXd sref0vec(Nref);
    sref0vec.fill(s_ref(Nref-1));
    Eigen::VectorXd zeroToOne = (1/totalArcLength)*(s_ref - sref0vec);

    // Create a zero to one vector including ref arc lengths & interp arc lengths (evenly spaced)
    int npts_total = npts+Nref;

    Eigen::VectorXd xx_linspace(npts);
    xx_linspace.fill(0.0);
    xx_linspace.setLinSpaced(npts, 1.0, 0.0);
    Eigen::VectorXd xx_unsorted(npts_total);
    xx_unsorted << xx_linspace, zeroToOne;
    std::sort(xx_unsorted.data(),xx_unsorted.data()+xx_unsorted.size());
    Eigen::VectorXd xx = xx_unsorted.reverse(); // Rich's interpolation functions call for descending order

    // List of return arc lengths in the original scaling/offset
    Eigen::VectorXd xx_sref0vec(npts_total);
    xx_sref0vec.fill(s_ref(Nref-1));
    Eigen::VectorXd s_interp = totalArcLength*xx+xx_sref0vec;

    // Interpolate to find list of return quaternions
    Eigen::MatrixXd q_interp = quatInterp(q_ref,zeroToOne,xx);

    // Interpolate to find list of return positions
    Eigen::VectorXd s_interp_spline = s_interp.reverse(); // Spline requires ascending order

    std::vector<double> svec;
    svec.resize(s_ref.size());
    Eigen::VectorXd::Map(&svec[0], s_ref.size()) = s_ref.reverse();

    Eigen::VectorXd x = posedata_ref.row(0).reverse();
    std::vector<double> xvec;
    xvec.resize(x.size());
    Eigen::VectorXd::Map(&xvec[0], x.size()) = x;
    tk::spline Sx;
    Sx.set_points(svec,xvec);
    Eigen::VectorXd x_interp(npts_total);
    x_interp.fill(0);
    for (int i = 0; i < npts_total; i++){
        x_interp(i) = Sx(s_interp_spline(i));
    }
    x_interp = x_interp.reverse().eval();

    Eigen::VectorXd y = posedata_ref.row(1).reverse();
    std::vector<double> yvec;
    yvec.resize(y.size());
    Eigen::VectorXd::Map(&yvec[0], y.size()) = y;
    tk::spline Sy;
    Sy.set_points(svec,yvec);
    Eigen::VectorXd y_interp(npts_total);
    y_interp.fill(0);
    for (int i = 0; i < npts_total; i++){
        y_interp(i) = Sy(s_interp_spline(i));
    }
    y_interp = y_interp.reverse().eval();

    Eigen::VectorXd z = posedata_ref.row(2).reverse();
    std::vector<double> zvec;
    zvec.resize(z.size());
    Eigen::VectorXd::Map(&zvec[0], z.size()) = z;
    tk::spline Sz;
    Sz.set_points(svec,zvec);
    Eigen::VectorXd z_interp(npts_total);
    z_interp.fill(0);
    for (int i = 0; i < npts_total; i++){
        z_interp(i) = Sz(s_interp_spline(i));
    }
    z_interp = z_interp.reverse().eval();

    Eigen::MatrixXd p_interp(3,npts_total);
    p_interp.fill(0);
    p_interp.row(0) = x_interp.transpose();
    p_interp.row(1) = y_interp.transpose();
    p_interp.row(2) = z_interp.transpose();

    interpRet interp_results;
    interp_results.s = s_interp;
    interp_results.p = p_interp;
    interp_results.q = q_interp;

    return interp_results;
}


Configuration3 q;
endonasal_teleop::config3 m;
void qcallback(const endonasal_teleop::config3 &msg)
{
    m = msg;

    for(int i=0; i<3; i++)
    {
        q.PsiL[i]=m.joint_q[i];
        q.Beta[i]=m.joint_q[i+3];
        q.Ftip[i]=m.joint_q[i+6];
        q.Ttip[i]=m.joint_q[i+9];
    }

//    std::cout << "joint update received by kinematics" << std::endl << std::endl;

    return;
}

std_msgs::Bool tmpBM;
void rrStatusCallback(const std_msgs::Bool &bmsg)
{
    tmpBM = bmsg;
    if(tmpBM.data==true)
    {
        new_q_msg = 1; // means we received the first joint values from res rates node
//        std::cout << "kinematics received a message from resolved rates" << std::endl << std::endl;
    }
}


int main(int argc, char *argv[])
{


/*******************************************************************************
                INITIALIZE ROS NODE
********************************************************************************/
    ros::init(argc,argv, "kinematics");
    ros::NodeHandle node;

/*******************************************************************************
                SET UP PUBLISHERS, SUBSCRIBERS, SERVERS & CLIENTS
********************************************************************************/

    // subscribers
    ros::Subscriber q_sub = node.subscribe("joint_q",1, qcallback);
    ros::Subscriber rr_status_sub = node.subscribe("rr_status",1,rrStatusCallback);

    // publishers
    ros::Publisher needle_pub = node.advertise<endonasal_teleop::matrix8>("needle_position",10);
    ros::Publisher kin_pub = node.advertise<endonasal_teleop::kinout>("kinematics_output",10);
    ros::Publisher kinematics_status_pub = node.advertise<std_msgs::Bool>("kinematics_status",10);

    // server (using a pointer, so it can be created/advertised within the while loop)
    std::shared_ptr<ros::ServiceServer> srv_getStartingKin;

    // client
//    ros::ServiceClient startingConfigClient = node.serviceClient<endonasal_teleop::getStartingConfig>("get_starting_config");

    // rate
    ros::Rate ra(rosLoopRate);


    startingConfigPublished = false;

/*******************************************************************************
                DEFINE CANNULA & IT'S STARTING/HOME POSE
********************************************************************************/

    // Cannula definition...

    typedef Tube< constant_fun< Vector<2>::type> >	T1_type;
    typedef Tube< constant_fun< Vector<2>::type> >	T2_type;
    typedef Tube< constant_fun< Vector<2>::type> >	T3_type;

    // NOTE: change so that tube dimensions are set by param server

    // Curvature of each tube
    constant_fun< Vector<2>::type > k_fun1( (1.0/63.5e-3)*Eigen::Vector2d::UnitX() );
    constant_fun< Vector<2>::type > k_fun2( (1.0/51.2e-3)*Eigen::Vector2d::UnitX() );
    constant_fun< Vector<2>::type > k_fun3( (1.0/71.4e-3)*Eigen::Vector2d::UnitX() );

    // Material properties
    double E = 60e9;
    double G = 60e9 / 2.0 / 1.33;
    // Tube 1 geometry
    double L1 = 222.5e-3;
    double Lt1 = L1 - 42.2e-3;
    double OD1 = 1.165e-3;
    double ID1 = 1.067e-3;
    // Tube 2 geometry
    double L2 = 163e-3;
    double Lt2 = L2 - 38e-3;
    double OD2 = 2.0574e-3;
    double ID2 = 1.6002e-3;
    //Tube 3 geometry
    double L3 = 104.4e-3;
    double Lt3 = L3 - 21.4e-3;
    double OD3 = 2.540e-3;
    double ID3 = 2.2479e-3;

    // Define tubes
    // Inputs: make_annular_tube( L, Lt, OD, ID, k_fun, E, G );
    T1_type T1 = make_annular_tube( L1, Lt1, OD1, ID1, k_fun1, E, G );
    T2_type T2 = make_annular_tube( L2, Lt2, OD2, ID2, k_fun2, E, G );
    T3_type T3 = make_annular_tube( L3, Lt3, OD3, ID3, k_fun3, E, G );

    // Assemble cannula
    CannulaT cannula = std::make_tuple( T1, T2, T3 );

    // Cannula starting configuration (home position):
    Configuration3 qstart;
    qstart.PsiL = Eigen::Vector3d::Zero();
    //qstart.PsiL << M_PI, M_PI, M_PI;
    qstart.Beta << -160.9e-3, -127.2e-3, -86.4e-3;
    qstart.Ftip = Eigen::Vector3d::Zero();
    qstart.Ttip = Eigen::Vector3d::Zero();
    Eigen::Vector3d qstartAlpha;
    //qstartAlpha << M_PI, M_PI, M_PI;
    qstartAlpha << 0.0, 0.0, 0.0;

    q = qstart;

    new_q_msg = 1;

    // Declarations
    int Npts;
    int nInterp = 50;
    double zeroIndex;
    Eigen::Vector3d pZero;
    Eigen::Vector4d qZero;
    Eigen::Matrix4d gZero;
    Eigen::Matrix4d gStarZero;
    Eigen::Matrix4d gStarL;


    while(ros::ok())
    {
        if(new_q_msg==1)
        {
            new_q_msg = 0;  // wait for kinematics to get called again

            // Run kinematics
            auto ret1 = Kinematics_with_dense_output( cannula, q, OType() );

            // Pick out the body Jacobian relating actuation to tip position
            J = CTR::GetTipJacobianForTube1(ret1.y_final);

            Stability = CTR::GetStability(ret1.y_final);

            // Transform it into the hybrid Jacobian:
//            Eigen::Matrix3d Rtip = quat2rotm(ret1.qTip);
//            Eigen::Matrix<double,6,6> RR;
//            RR.fill(0);
//            RR.topLeftCorner(3,3) = Rtip;
//            J = RR*Jbody;

            Npts = ret1.arc_length_points.size();
            double* ptr = &ret1.arc_length_points[0];
            Eigen::Map<Eigen::VectorXd> s(ptr, Npts);

            Eigen::VectorXd zeroIndexVec;
            int count = 0;
            for (int i=0; i < s.size(); ++i) {
                if (s(i) == 0) {
                    zeroIndexVec.resize(count+1);
                    zeroIndexVec(count) = (double) i;
                    count ++;
                }
            }
            zeroIndex = zeroIndexVec(count-1);
            pZero = ret1.dense_state_output.at( zeroIndex ).p;
            qZero = ret1.dense_state_output.at( zeroIndex ).q;
            gZero = assembleTransformation(quat2rotm(qZero),pZero);
            gStarZero = assembleTransformation(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
            gStarL = gStarZero*inverseTransform(gZero);

            Eigen::MatrixXd posedata(8,Npts);
            for (int i=0; i < Npts; ++i)
            {
              Eigen::Vector3d pi = ret1.dense_state_output.at( i ).p;
              Eigen::Vector4d qi = ret1.dense_state_output.at( i ).q;
              Eigen::Matrix4d gi = assembleTransformation(quat2rotm(qi),pi);
              Eigen::Matrix4d gStari = gStarL*gi;
              Eigen::Matrix<double,8,1> xi;
              xi.fill(0);
              xi.head<7>() = collapseTransform(gStari);
              xi(7) = 1.0;
              posedata.col(i) = xi;
            }

            // Interpolate points along the backbone
            interpRet interp_results = interpolateBackbone(s,posedata,nInterp);
            Eigen::MatrixXd posedata_out(8,nInterp+Npts);
            posedata_out = Eigen::MatrixXd::Zero(8,nInterp+Npts);
            Eigen::RowVectorXd ones(nInterp+Npts);
            ones.fill(1);
            Eigen::VectorXd s_out = interp_results.s;
            posedata_out.topRows(3) = interp_results.p;
            posedata_out.middleRows<4>(3) = interp_results.q;
            posedata_out.bottomRows(1) = ones;

            ptip = ret1.pTip;
            qBishop = ret1.qTip; // kinematics returns the Bishop frame
            RBishop = quat2rotm(qBishop);
            Eigen::Matrix<double,3,3> rotate_psiL = Eigen::Matrix<double,3,3>::Identity();
            rotate_psiL(0,0) = cos(q.PsiL(0));
            rotate_psiL(0,1) = -sin(q.PsiL(0));
            rotate_psiL(1,0) = sin(q.PsiL(0));
            rotate_psiL(1,1) = cos(q.PsiL(0));
            //std::cout << "PsiL(0) (deg): " << q.PsiL(0)*180/M_PI << std::endl;
            //std::cout << "rotate_psiL: " << std::endl << rotate_psiL << std::endl;
            Rtip = RBishop*rotate_psiL;
            //std::cout << "Rtip: " << std::endl << Rtip << std::endl;
            qtip = rotm2quat(Rtip);


            // tip pose message for resolved rates
            kin_msg.p[0] = ptip[0];
            kin_msg.p[1] = ptip[1];
            kin_msg.p[2] = ptip[2];
            kin_msg.q[0] = qtip[0];
            kin_msg.q[1] = qtip[1];
            kin_msg.q[2] = qtip[2];
            kin_msg.q[3] = qtip[3];
            kin_msg.qb[0] = qBishop[0];
            kin_msg.qb[1] = qBishop[1];
            kin_msg.qb[2] = qBishop[2];
            kin_msg.qb[3] = qBishop[3];
            kin_msg.alpha[0] = ret1.y_final.Psi[0];
            kin_msg.alpha[1] = ret1.y_final.Psi[1];
            kin_msg.alpha[2] = ret1.y_final.Psi[2];
            kin_msg.Stability = Stability;

            for(int i=0; i<6; i++)
            {
                kin_msg.J1[i]=J(0,i);
                kin_msg.J2[i]=J(1,i);
                kin_msg.J3[i]=J(2,i);
                kin_msg.J4[i]=J(3,i);
                kin_msg.J5[i]=J(4,i);
                kin_msg.J6[i]=J(5,i);
            }

            // "dense output" message for drawing the backbone
            for(int j=0; j<(nInterp+Npts); j++)
            {
                int p = 0;
                markers_msg.A1[j]=posedata_out(p,j);
                markers_msg.A2[j]=posedata_out(p+1,j);
                markers_msg.A3[j]=posedata_out(p+2,j);
                markers_msg.A4[j]=posedata_out(p+3,j); //w
                markers_msg.A5[j]=posedata_out(p+4,j); //x
                markers_msg.A6[j]=posedata_out(p+5,j); //y
                markers_msg.A7[j]=posedata_out(p+6,j); //z

                // choose color coding for each tube:
                if (q.Beta[1]>s_out[j] || L2+q.Beta[1]<s_out[j])
                {
                    markers_msg.A8[j] = 1; // inner tube - green
                }
                else if ((q.Beta[1]<=s_out[j] && q.Beta[2]>s_out[j]) || (L3+q.Beta[2]<s_out[j] && L2+q.Beta[1]>=s_out[j]))
                {
                    markers_msg.A8[j] = 2; // middle tube - red
                }
                else
                {
                    markers_msg.A8[j] = 3; // outer tube - blue
                }

            }


            // if this is the first kinematics pose computed,
            // advertise it via the server:
            if(!startingConfigPublished)
            {
                srv_getStartingKin = std::make_shared<ros::ServiceServer>(node.advertiseService("get_starting_kin",startingKin));
                startingConfigPublished = true;
            }

            // send new messages to other nodes
            kinematics_status_pub.publish(kinUpdateStatusMsg);
            kin_pub.publish(kin_msg);
            needle_pub.publish(markers_msg); //needle_display

            // tell resolved rates this node has updated
            kinUpdateStatusMsg.data = true;
            kinematics_status_pub.publish(kinUpdateStatusMsg);

        }

        ros::spinOnce();
        ra.sleep();
    }

    return 0;

}
