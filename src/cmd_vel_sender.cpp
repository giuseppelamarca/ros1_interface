#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/stat.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include "topics_struct.h"

#define FLAG IPC_NOWAIT

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int msgid = 0;
struct cmd_vel cmd_vel_;
void * cmd_vel_ptr = &cmd_vel_;

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

void cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel_.twist_.linear.x = msg->linear.x;
    cmd_vel_.twist_.linear.y = msg->linear.z;
    cmd_vel_.twist_.linear.z = msg->linear.y;
    cmd_vel_.twist_.angular.x = msg->angular.x;
    cmd_vel_.twist_.angular.y = msg->angular.y;
    cmd_vel_.twist_.angular.z = msg->angular.z;
    msgsnd(msgid, cmd_vel_ptr, sizeof(cmd_vel_), FLAG);
}

int main(int argc, char **argv)
{
    //assign the address
    msgid = msgget(IPC_PRIVATE, IPC_CREAT | S_IRUSR | S_IWUSR);
    if (msgid == -1)
        return -1; 

    //get the pid of the receiver
    int pid = get_pid("cmd_vel_receiver");

    //send the address that contain all the addresses
    union sigval sig;
    sig.sival_int = msgid;
    if (sigqueue(pid, SIGUSR2, sig))
        return -1;


    ros::init(argc, argv, "cmd_vel_2");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmd_vel_Callback);
    ros::spin();

  return 0;
}
