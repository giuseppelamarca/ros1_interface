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
#include "diagnostic_msgs/DiagnosticArray.h"

int msgid = 0;
struct diagnostics_agg msg_;
struct diagnostics_status s_;

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

void Callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
    ROS_INFO("msg received");
    ROS_INFO("%d\n", msg->header.seq);
    msg_.header_.seq = msg->header.seq;
    msg_.header_.sec = msg->header.stamp.sec;
    msg_.header_.nsec = msg->header.stamp.nsec;
    memset(msg_.header_.frame_id, 0, sizeof(msg_.header_.frame_id));
    msg->header.frame_id.copy(msg_.header_.frame_id, msg->header.frame_id.length());

    for (int i = 0; i < sizeof(msg_.status)/sizeof(s_); i++){
        //name
        memset(msg_.status[i].name, 0, sizeof(msg_.status[i].name));
        msg->status[i].name.copy(msg_.status[i].name, msg->status[i].name.length());
        //message
        memset(msg_.status[i].message, 0, sizeof(msg_.status[i].message));
        msg->status[i].message.copy(msg_.status[i].message, msg->status[i].message.length());
        //hardware_id
        memset(msg_.status[i].hardware_id, 0, sizeof(msg_.status[i].hardware_id));
        msg->status[i].hardware_id.copy(msg_.status[i].hardware_id, msg->status[i].hardware_id.length());                
    }
    gettimeofday(&t, NULL);
    msg_.t = t.tv_usec;
    printf("output %d\n", msgsnd(msgid, msg_ptr, sizeof(msg_), FLAG));
    printf("errno: %d\n", errno);
}

int main(int argc, char **argv)
{
    msg_.mtype = 2;

    //assign the address
    msgid = msgget(IPC_PRIVATE, IPC_CREAT | S_IRUSR | S_IWUSR);
    if (msgid == -1)
        return -1; 

    ROS_INFO("MSGID: %d\n", msgid);  
    //get the pid of the receiver
    int pid = get_pid("diagnostics_receiver");
    ROS_INFO("PID: %d\n", pid);
    //send the address that contain all the addresses
    union sigval sig;
    sig.sival_int = msgid;
    if (sigqueue(pid, SIGUSR2, sig))
        return -1;


    ros::init(argc, argv, "diagnostics_agg_2");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("diagnostics_agg", 1000, Callback);
    ros::spin();

  return 0;
}
