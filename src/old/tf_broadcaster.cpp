#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>

// define tf stuff

tf::Transform transform;
//float z_value;
float z_value =float();

//subscriber callback function
void Callback(const geometry_msgs::Pose& msg)
{
// Define the frame
//double x= msg.position.x;
transform.setOrigin( tf::Vector3(msg.position.x/10,msg.position.y/10,msg.position.z/10));
tf::Quaternion q(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
q.normalize();
transform.setRotation(q);
z_value=msg.position.z;
}


int main(int argc, char** argv){
ros::init(argc,argv, "my_tf_broadcaster");
ros::NodeHandle node;

// use the tf library to broadcast tf frames to Rviz
static tf::TransformBroadcaster br;

// define frame positions
float x_value =float(); 
float y_value =float();


// Force feedback stuff
// Set the grid on xy plan. 
float threshold = 0;
std_msgs::Int32 force_flag;

//
ros::Subscriber sub = node.subscribe("Omnipos", 1000, Callback);
ros::Publisher pub = node.advertise<std_msgs::Int32>("Omniforce", 1000);
ros::Rate r(1000);

while(ros::ok()){
// send transform
br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world","tip_pose"));
// send force feedback back to windows & omni
if (z_value < threshold) { force_flag.data = 1; }
else { force_flag.data = 0; }
pub.publish(force_flag);
ros::spinOnce();
r.sleep();
}

return 0;
}
