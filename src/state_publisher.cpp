#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double yaw = 0, pitch = 0, r1 = 0, r2 = 0, r3 = 0, l1 = 0, l2 = 0, l3 = 0;

    // message declarations
    // geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "axis";

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(8);
        joint_state.position.resize(8);
        joint_state.name[0] ="joint_head_yaw";
        joint_state.position[0] = yaw;
        joint_state.name[1] ="joint_head_pitch";
        joint_state.position[1] = pitch;
        joint_state.name[2] ="joint_right_1";
        joint_state.position[2] = r1;
        joint_state.name[3] ="joint_right_2";
        joint_state.position[3] = r2;
        joint_state.name[4] ="joint_right_3";
        joint_state.position[4] = r3;
        joint_state.name[5] ="joint_left_1";
        joint_state.position[5] = l1;
        joint_state.name[6] ="joint_left_2";
        joint_state.position[6] = l2;
        joint_state.name[7] ="joint_left_3";
        joint_state.position[7] = l3;


        // update transform
        // (moving in a circle with radius=2)
        // odom_trans.header.stamp = ros::Time::now();
        // odom_trans.transform.translation.x = cos(angle)*2;
        // odom_trans.transform.translation.y = sin(angle)*2;
        // odom_trans.transform.translation.z = .7;
        // odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        // broadcaster.sendTransform(odom_trans);

        // Create new robot state
        yaw += 0.005;
        if (yaw >= 1.47)
            yaw = -1.47;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    return 0;
}

