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
#include "shm_lib.h"



#define FLAG IPC_NOWAIT

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "diagnostic_msgs/DiagnosticArray.h"

int msgid = 0;
//struct diagnostics_agg msg_;
//struct diagnostics_status s_;

//void *msg_ptr = &msg_;

int semid, shmid, bytes, xfrs;
struct diagnostics_agg *msg_;
union semun dummy;
//used to check the performance
struct timeval t;

void Callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
    if (reserveSem(semid, WRITE_SEM) == -1)         /* Wait for our turn */
        printf("reserveSem");
        memset(msg_->status, 0, sizeof(msg_->status));
        memset(msg_->status, 0, sizeof(msg_->status));
    ROS_INFO("msg received");
    ROS_INFO("%d\n", msg->header.seq);
    msg_->header_.seq = msg->header.seq;
    msg_->header_.sec = msg->header.stamp.sec;
    msg_->header_.nsec = msg->header.stamp.nsec;
    memset(msg_->header_.frame_id, 0, sizeof(msg_->header_.frame_id));
    msg->header.frame_id.copy(msg_->header_.frame_id, msg->header.frame_id.length());

    for (int i = 0; i < sizeof(msg_->status)/sizeof(struct diagnostics_status); i++){
        //name
        memset(msg_->status[i].name, 0, sizeof(msg_->status[i].name));
        msg->status[i].name.copy(msg_->status[i].name, msg->status[i].name.length());
        //message
        memset(msg_->status[i].message, 0, sizeof(msg_->status[i].message));
        msg->status[i].message.copy(msg_->status[i].message, msg->status[i].message.length());
        //hardware_id
        memset(msg_->status[i].hardware_id, 0, sizeof(msg_->status[i].hardware_id));
        msg->status[i].hardware_id.copy(msg_->status[i].hardware_id, msg->status[i].hardware_id.length());                

        msg_->status[i].kv_size = msg->status[i].values.size();
        printf("msg size: %d\n", msg_->status[i].kv_size);
        for (int j = 0; j < msg_->status[i].kv_size; j++){
            //key
            msg->status[i].values[j].key.copy(msg_->status[i].kv[j].key, msg->status[i].values[j].key.length()); 
            //value
            msg->status[i].values[j].value.copy(msg_->status[i].kv[j].value, msg->status[i].values[j].value.length());             
        }
    }
    gettimeofday(&t, NULL);
    msg_->t = t.tv_usec;

    if (releaseSem(semid, READ_SEM) == -1)          /* Give reader a turn */
        printf("releaseSem");
}

int main(int argc, char **argv)
{
    /* Create two semaphores and create shm; initialize so that
       writer has first access to shared memory. */
    semid = semget(SEM_KEY, 2, IPC_CREAT | OBJ_PERMS);
    if (semid == -1)  printf("semget");
    if (initSemAvailable(semid, WRITE_SEM) == -1) printf("initSemAvailable");
    if (initSemInUse(semid, READ_SEM) == -1) printf("initSemInUse");

    /* Create shared memory; attach at address chosen by system */

    shmid = shmget(SHM_KEY, sizeof(struct diagnostics_agg), IPC_CREAT | OBJ_PERMS);
    if (shmid == -1) printf("shmget");
    msg_ = (struct diagnostics_agg *)shmat(shmid, NULL, 0);
    if (msg_ == (void *) -1) printf("shmat");

    ros::init(argc, argv, "diagnostics_agg_2");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("diagnostics_agg", 1000, Callback);
    ros::spin();

    if (semctl(semid, 0, IPC_RMID, dummy) == -1) printf("semctl");
    if (shmdt(msg_) == -1) printf("shmdt");
    if (shmctl(shmid, IPC_RMID, 0) == -1) printf("shmctl");
  return 0;
}
