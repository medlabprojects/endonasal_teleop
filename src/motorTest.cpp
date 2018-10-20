// %Tag(FULLTEXT)%

// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%

// %Tag(MSG_HEADER)%
#include "medlab_motor_control_board/McbEncoders.h"
// %EndTag(MSG_HEADER)%

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nodeCommandSender");

  ros::NodeHandle n;

  ros::Publisher pubEncoderCommand = n.advertise<medlab_motor_control_board::McbEncoders>("encoder_command", 1);

  double amplitude = 10000.0;
  double signal_hertz = 0.25;
  double loop_hertz = 220.0;
  ros::Rate loop_rate(loop_hertz);
  double current_time = 0.0;
  double count = 0;

  while (ros::ok())
  {
    medlab_motor_control_board::McbEncoders enc;
    enc.count[0] = (int)(amplitude*sin((2.0*M_PI*signal_hertz/loop_hertz)*count)+amplitude+1000);
    enc.count[1] = (int)(amplitude*sin((2.0*M_PI*signal_hertz/loop_hertz)*count + 0.25)+amplitude+1000);
    enc.count[2] = (int)(amplitude*sin((2.0*M_PI*signal_hertz/loop_hertz)*count + 0.5)+amplitude+1000);
    enc.count[3] = (int)(amplitude*sin((2.0*M_PI*signal_hertz/loop_hertz)*count + 0.75)+amplitude+1000);
    enc.count[4] = (int)(amplitude*sin((2.0*M_PI*signal_hertz/loop_hertz)*count + 1.0)+amplitude+1000);
    enc.count[5] = (int)(amplitude*sin((2.0*M_PI*signal_hertz/loop_hertz)*count + 1.25)+amplitude+1000);

    ROS_INFO("message sent");

    pubEncoderCommand.publish(enc);

    ros::spinOnce();

    loop_rate.sleep();

    count += 1.0;
  }


  return 0;
}
// %EndTag(FULLTEXT)%

