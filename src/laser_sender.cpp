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
#include <vector>

#define FLAG IPC_NOWAIT

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

int msgid = 0;
struct laser_scan msg_;
void *msg_ptr = &msg_;

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

void Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    msg_.header_.seq = msg->header.seq;
    msg_.header_.sec = msg->header.stamp.sec;
    msg_.header_.nsec = msg->header.stamp.nsec;
    memset(msg_.header_.frame_id, 0, sizeof(msg_.header_.frame_id));
    msg->header.frame_id.copy(msg_.header_.frame_id, msg->header.frame_id.length());
    msg_.angle_min = msg->angle_min;
    msg_.angle_max = msg->angle_max;
    msg_.angle_increment = msg->angle_increment;
    msg_.time_increment = msg->time_increment;
    msg_.scan_time = msg->scan_time;
    msg_.range_min = msg->range_min;
    msg_.range_max = msg->range_max;
    //printf("%d\n", (msg->intensities).size());
    for (int i = 0; i < sizeof(msg_.ranges)/sizeof(float); i++){
        msg_.ranges[i] = msg->ranges[i];
        msg_.intensities[i] = msg->intensities[i];
    }
    gettimeofday(&t, NULL);
    msg_.t = t.tv_usec;
    msgsnd(msgid, msg_ptr, sizeof(msg_), FLAG);
}

int main(int argc, char **argv)
{
    msg_.mtype = 2;

    //assign the address
    msgid = msgget(IPC_PRIVATE, IPC_CREAT | S_IRUSR | S_IWUSR);
    if (msgid == -1)
        return -1; 

    //get the pid of the receiver
    int pid = get_pid("laser_receiver");

    //send the address that contain all the addresses
    union sigval sig;
    sig.sival_int = msgid;
    if (sigqueue(pid, SIGUSR2, sig))
        return -1;


    ros::init(argc, argv, "laser_2");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("sick_safetyscanners/scan", 1000, Callback);
    ros::spin();

  return 0;
}
