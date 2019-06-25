#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

enum Joint {
    link_00, link_01, link_02, link_03, link_04, link_05, link_06
};

double PI = 3.1415;

bool canMove = true;

tf::Transform getTransform(Joint joint, double angle){
  tf::Transform transform;
  tf::Quaternion q_rot;
  switch (joint){
     case link_00:
         q_rot = tf::createQuaternionFromRPY(0, 0, angle);
         transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
         transform.setRotation(q_rot);
         return transform;
     case link_01:
         q_rot = tf::createQuaternionFromRPY(0, angle, 0);
         transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
         transform.setRotation(q_rot);
         return transform;
     case link_02:
         q_rot = tf::createQuaternionFromRPY(0, 0, angle);
         transform.setOrigin( tf::Vector3(0.0, 0.0, 0.4));
         transform.setRotation(q_rot);
         return transform;
       }
       return transform;

}

void moveArmToPos (double th1, double th2, double th3){
    tf::Transform transform1 = getTransform(link_00, th1);
    tf::Transform transform2 = getTransform(link_01, th2);
    tf::Transform transform3 = getTransform(link_02, th3);

    tf::Transform transform4;
    transform4.setOrigin( tf::Vector3(0.0, 0.0, 0.4));
    transform4.setRotation( tf::createQuaternionFromRPY(0, 0, 0));


    static tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(
        tf::StampedTransform(
            transform1,
            ros::Time::now(), "base_link", "link_00"));

    broadcaster.sendTransform(
        tf::StampedTransform(
            transform2,
            ros::Time::now(), "link_00", "link_01"));

    broadcaster.sendTransform(
        tf::StampedTransform(
            transform3,
            ros::Time::now(), "link_01", "link_02"));

    broadcaster.sendTransform(
        tf::StampedTransform(
            transform4,
            ros::Time::now(), "link_05", "link_06"));
}

void moveArm(double x, double y, double z){
  double thetas[] = {0.78,1.5805, 0.5};
  std::cout << "Move Arm to : x = " << x << ", y = " << y << ", z = " << z << "\n";
  for (int i = 1; i <= 20; i++) {
        double dth1 = (thetas[0] / 20) * i;
        double dth2 = (thetas[1] / 20) * i;
        double dth3 = (thetas[2] / 20) * i;

        moveArmToPos(dth1, dth2, dth3);
      }
}

void initArm() {
    moveArm(0, 0, 0);
}


void chatterCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    moveArm(msg->x,msg->y,msg->z);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cmd_vel_pos/ball_tf", 100, chatterCallback);
    ros::spin();
    initArm();
    return 0;
}
