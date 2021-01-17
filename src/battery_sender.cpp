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
#include "sensor_msgs/BatteryState.h"

int msgid = 0;
struct battery msg_;
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

void Callback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    msg_.header_.seq = msg->header.seq;
    msg_.header_.sec = msg->header.stamp.sec;
    msg_.header_.nsec = msg->header.stamp.nsec;
    memset(msg_.header_.frame_id, 0, sizeof(msg_.header_.frame_id));
    msg->header.frame_id.copy(msg_.header_.frame_id, msg->header.frame_id.length());

    msg_.voltage = msg->voltage; 
    msg_.current = msg->current; 
    msg_.charge = msg->charge; 
    msg_.capacity = msg->capacity; 
    msg_.design_capacity = msg->design_capacity; 
    msg_.percentage = msg->percentage; 
    msg_.power_supply_status = msg->power_supply_status; 
    msg_.power_supply_health = msg->power_supply_health; 
    msg_.power_supply_technology = msg->power_supply_technology; 
    msg_.present = msg->present; 
    for (int i = 0; i < N_CELLS; i++){
        msg_.cell_voltage[i] = msg->cell_voltage[i];
    }
    msg->location.copy(msg_.location, msg->location.length());
    msg->serial_number.copy(msg_.serial_number, msg->serial_number.length());

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
    int pid = get_pid("battery_receiver");

    //send the address that contain all the addresses
    union sigval sig;
    sig.sival_int = msgid;
    if (sigqueue(pid, SIGUSR2, sig))
        return -1;


    ros::init(argc, argv, "battery_2");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("EDC/BatterySlotInfo", 1000, Callback);
    ros::spin();

  return 0;
}
