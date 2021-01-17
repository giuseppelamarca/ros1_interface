#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include "topics_struct.h"
#include <sys/time.h>

#define FLAG IPC_NOWAIT

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

int msgid = 0;
struct odom odom_;
void *odom_ptr = &odom_;

//used to check the performance
struct timeval t;

int get_pid(char *proc_name){
    FILE *f; 
    char cmd[100]={0};
    int pid = -1;
    if (proc_name != 0){
        strcpy(cmd, "pidof ");
        strcat(cmd, proc_name);
        strcat(cmd, "> /tmp/pidof");
        system(cmd);
        f = fopen("/tmp/pidof", "r");
        fscanf(f, "%d", &pid);
    }
    return pid;
}

void odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_.header_.seq = msg->header.seq;
    odom_.header_.sec = msg->header.stamp.sec;
    odom_.header_.nsec = msg->header.stamp.nsec;
    memset(odom_.header_.frame_id, 0, sizeof(odom_.header_.frame_id));
    memset(odom_.child_frame_id, 0, sizeof(odom_.child_frame_id));
    msg->header.frame_id.copy(odom_.header_.frame_id, msg->header.frame_id.length());
    msg->child_frame_id.copy(odom_.child_frame_id, msg->child_frame_id.length());
    odom_.pose_.position.x = msg->pose.pose.position.x;
    odom_.pose_.position.y = msg->pose.pose.position.y;
    odom_.pose_.position.z = msg->pose.pose.position.z;
    odom_.pose_.orientation.x = msg->pose.pose.orientation.x;
    odom_.pose_.orientation.y = msg->pose.pose.orientation.y;
    odom_.pose_.orientation.z = msg->pose.pose.orientation.z;
    odom_.pose_.orientation.w = msg->pose.pose.orientation.w;
    odom_.twist_.linear.x = msg->twist.twist.linear.x;
    odom_.twist_.linear.y = msg->twist.twist.linear.y;
    odom_.twist_.linear.z = msg->twist.twist.linear.z;
    odom_.twist_.angular.x = msg->twist.twist.angular.x;
    odom_.twist_.angular.y = msg->twist.twist.angular.y;
    odom_.twist_.angular.z = msg->twist.twist.angular.z;
    for (int i = 0; i < 36; i++){
        odom_.pose_covariance[i] = msg->pose.covariance[i];
        odom_.twist_covariance[i] = msg->twist.covariance[i];
    }
    gettimeofday(&t, NULL);
    odom_.t = t.tv_usec;
    msgsnd(msgid, odom_ptr, sizeof(odom_), FLAG);
}

int main(int argc, char **argv)
{
    odom_.mtype = 1;

    //assign the address
    msgid = msgget(IPC_PRIVATE, IPC_CREAT | S_IRUSR | S_IWUSR);
    if (msgid == -1)
        return -1; 

    //get the pid of the receiver
    int pid = get_pid("odom_receiver");

    //send the address that contain all the addresses
    union sigval sig;
    sig.sival_int = msgid;
    if (sigqueue(pid, SIGUSR2, sig))
        return -1;


    ros::init(argc, argv, "odom_2");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("odom", 1000, odom_Callback);
    ros::spin();

  return 0;
}
