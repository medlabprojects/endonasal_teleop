// This should handle translational joint limits
// We don't really have any rotational joint limits

#include <QCoreApplication>
#include "Kinematics.h"
#include "BasicFunctions.h"
#include "Tube.h"
#include <iostream>
#include <random>
#include <vector>
#include <QVector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "spline.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include "std_msgs/Bool.h"
#include <endonasal_teleop/matrix8.h>
#include <endonasal_teleop/config3.h>
#include <endonasal_teleop/matrix6.h>

using namespace CTR;
using namespace CTR::Functions;
using namespace std;
using std::tuple;
using Eigen::Vector2d;


//update needle beta values
endonasal_teleop::matrix8 tmpBeta;
void kneedleCallback(const endonasal_teleop::matrix8 &beta)
{
    tmpBeta = beta;
    for(int i=0; i<3; i++)
    {
        carrierBeta(i) = tmpBeta.data(i+3);
    }
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "jointLimits");
    ros::NodeHandle nH;

    ros::Publisher limit_needle = nH.advertise<endonasal_teleop::matrix8>("limited_needle",1000);
    //get beta from kinematics
    ros::Subscriber kinematics_needle = nH.subscribe("needle_position", 1000, kneedleCallback());

    ros::Rate r(10);


    //dimension of concern is the beta

    //for one carrier
    double carrierThickness = 100; //mm

    //carrier thickness + some extra distance for a safety factor
    double safetyThickness = 50; //mm

    //minimum distance before hitting walls
    double minDistFront = 20; //mm
    double minDistBack = 20; //mm

    double minDistBetween = 20; //mm

    //number of total tubes
    int numTubes = 3;


    //carrier 1 dimensions:
    //carrier 2 dimensions:
    //carrier 3 dimensions:

    //carriers will start out at some starting configuration

    //carrier thickness behind where the tube is held
    //first tube is closest to back wall
    Eigen::Vector3d carrierBack;
    for(int j=0; j<numTubes; j++)
    {
        if(j==0) //first tube
        {
            carrierBack(j) = minDistBack +carrierThickness;
        } else
        {
            carrierBack(j) = carrierBack(j-1) + carrierThickness;
        }
    }

    //carrier thickness in front of where the tube is held
    Eigen::Vector3d carrierFront;
    for(int j=0; j<numTubes; j++)
    {
        if(j==0) //first tube
        {
            carrierFront(j) = minDistFront;
        } else
        {
            carrierFront(j) = carrierFront(j-1) + carrierThickness + minDistBetween;
        }
    }

    Eigen::Vector3d carrierBeta;


    while(ros::ok())
    {
        //read desired position - carrierBeta


        //prevent carriers from colliding
        //based on Caleb's actuator fixing algorithm
        //prevents collisions between the carriers and moves the needle to the desired position
        for (size_t i = 0; i < numTubes; i++)
        {
            if(i==0) //backmost carriage, check if it'll hit the wall
            {
                if(carrierFront(i) - carrierBeta(i) >= minDistBack) // won't hit back wall
                {
                    //move to desired position
                    carrierBeta(i) += carrierBack(i);
                }
            }else
            {
                //check if carrier will collide with the one behind it
                if (carrierBeta(i) - carrierBack(i) < carrierBeta(i-1) + carrierFront(i-1))
                {  // if the carrier would hit the one behind it

                    carrierBeta(i) = carrierBeta(i-1) + carrierFront(i-1) + carrierBack(i);
                }


                if(i = numTubes - 1) //front most carriage
                {
                    //make sure front most tube doesn't hit front wall
                    if(carrierFront(i) >= minDistFront) //won't hit front wall
                    {
                        //move to desired position
                        carrierBeta(i) += carrierFront(i);
                    }
                }
            }


            double partial_sum = 0.0;
            for (unsigned int k = i+1; k <= numTubes; ++k)
            {
                partial_sum += carrierFront(k) + carrierBack(k);
            }
            if (carrierBeta(i) + carrierFront(i) > -tp - partial_sum)
            {
                carrierBeta(i) = -tp - partial_sum - carrierFront(i);
            }

            if (i != 2)
            {
                if (carrierBeta(i) + m_L(i) > carrierBeta(i-1) + m_L(i-1)-0.001)
                {
                    carrierBeta(i) = carrierBeta(i-1) + m_L(i-1) - m_L(i) - 0.001;
                }
            }else
            { //keep the second tube from knocking the end-effector off the end
                if (carrierBeta(i) + m_L(i) > carrierBeta(i-1) + m_L(i-1)-0.020)
                {
                    carrierBeta(i) = carrierBeta(i-1) + m_L(i-1) - m_L(i) - 0.020;
                }
            }

            if (carrierBeta(i) + m_L(i) - 0.001*(numTubes-i+1) < 0.0)
            {
                carrierBeta(i) = -m_L(i) + 0.001*(numTubes-i+1);
            }

        }





        m_q.block(numTubes,0,numTubes,1) = carrierBeta.block(1,0,numTubes,1);
        m_mutex.unlock();

        //Check this actuator step for validity before sending them
        //to the controller -- all it does is check that it's finite
        bool validStep = true;
        QString errormsg;
        for (int i = 0; i < m_q.rows(); ++i)
        {
            if (m_q(i) != m_q(i))
            {
                validStep = false;
                errormsg = "Actuator value is NaN or inf";
            }
        }

        //for (int i = 0; i < m_q.rows()/2; ++i) {      // We would uncomment this if we wanted to limit rotation
        //    if (fabs(m_q(i)) > 10.0*M_PI) {
        //        validStep = false;
        //        errormsg = "Wound up Rotation";
        //    }
        //}

        for (int i = m_q.rows()/2; i < m_q.rows(); ++i)
        {
            if (m_q(i) > 0 || m_q(i) < m_acr->GetBackPlateArcLength())
            {
                validStep = false;
                errormsg = "bad translation command";
            }
        }

        if (validStep == true && m_SimulationOnly == false && m_hardwareEnabled == true)
        {
            //Send the translations to the robot
            for (unsigned int i = 0; i < numTubes; ++i)
            {
                ControllerConnection id = m_acr->GetTranslationControllerConnection(i+1);
                double gearRatio = m_acr->GetTubeCarrier(i+1).GetTranslationGearRatio();
                double pos = m_q(i+numTubes)-m_acr->GetTubeCarrier(i+1).GetHomePosition();
                pos *= gearRatio;
                id.GetController()->SetSetpoint(id.GetControllerAxis(), pos);
                //m_lControllers[id.controllerID]->setPosition(id.axis, pos);
            }

            //Send the rotations to the robot
            for (unsigned int i = 0; i < numTubes; ++i)
            {
                ControllerConnection id = m_acr->GetRotationControllerConnection(i+1);
                double gearRatio = m_acr->GetTubeCarrier(i+1).GetRotationGearRatio();
                double pos = m_q(i);
                pos *= gearRatio;
                id.GetController()->SetSetpoint(id.GetControllerAxis(), pos);
                //m_lControllers[id.controllerID]->setPosition(id.axis, pos);
            }
        } else if (validStep == false)
        {
            stop_reason = "Invalid actuator values found.  Resetting.";
            stopping_for_emergency = true;
            m_isRunning = false;
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
