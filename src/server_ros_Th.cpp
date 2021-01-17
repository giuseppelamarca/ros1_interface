// Server side C/C++ program to demonstrate Socket programming 
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <pthread.h>
#define PORT 8090 
#define MAX_CLIENTS 5 
#define MAX_LEN 100

//ROS 
#include "ros/ros.h"
#include "ros/spinner.h"
#include "geometry_msgs/QuaternionStamped.h"
#include <sstream>

struct Topic{
    char topic[MAX_LEN];
    int socket;
};

struct Quaternion{
    double x;
    double y;
    double z; 
    double w; 
    uint32_t seq;
    char *frame_id;
    time_t stamp;
};

class Ros_Class{
    private:
        int socket;
        char topic[MAX_LEN];
    public:
        void chatterCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
        Ros_Class(void *ptr);
};

void Ros_Class::chatterCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    struct Quaternion q;

    q.x = msg->quaternion.x;
    q.y = msg->quaternion.y;
    q.z = msg->quaternion.z;
    q.w = msg->quaternion.w;
    q.seq = msg->header.seq;
    //q.frame_id = msg->header.frame_id;
    //q.stamp = msg->header.stamp;
    
    void *p = &q;
    send(socket, p, sizeof(struct Quaternion), 0 );
    ROS_INFO("X: %lf \tY:%lf\tZ: %lf\tW: %lf\n", (*(struct Quaternion *)p).x,(*(struct Quaternion *)p).y,(*(struct Quaternion *)p).z,(*(struct Quaternion *)p).w );
}

Ros_Class::Ros_Class(void *ptr){
    socket =(*(struct Topic *)ptr).socket;
    strcpy(topic , (*(struct Topic *)ptr).topic);
    int argc = 1;
    ros::Subscriber sub;
    ros::NodeHandle n;
    printf("starting node\n");
    printf("Ros: %s\n", topic);
    printf("charcter: %d\n", topic[strlen(topic)-1]);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    sub =  n.subscribe<geometry_msgs::QuaternionStamped>(topic, 1, boost::bind(&Ros_Class::chatterCallback, this, _1));
    ros::waitForShutdown();
}

void * client_fun(void *ptr_strct){
    Ros_Class ros(ptr_strct);
    while(true){
        sleep(3);
    }
}

int main(int argc, char **argv)
{ 
    int server_fd; 
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
    char buffer[1024] = {0}; 
    pthread_t client_th[MAX_CLIENTS]; 
    pthread_t *client_th_ptr = client_th;
    int new_socket[MAX_CLIENTS];
    int *new_socket_ptr = new_socket;

    // Creating socket file descriptor 
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0){ 
        printf("error creating socket");
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    } 
       
    // Forcefully attaching socket to the port 8080 
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))){ 
        printf("error attach port");
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 
       
    printf("waiting for new clients...");
    // Forcefully attaching socket to the port 8080 
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0){ 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    printf("waiting for new clients...");
    if (listen(server_fd, 3) < 0){ 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 
    ros::init(argc, argv, "server");
    while(true){
        if ((*new_socket_ptr = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0){ 
            perror("accept"); 
            exit(EXIT_FAILURE); 
        }
        else{
            printf("\nconnection accepted\n");
            char buff[100];
            char topic_n[100];
            ssize_t n = read(*new_socket_ptr, buff, 100);
            for (int i=0; i<n; i++){
                topic_n[i] = buff[i];
            }
            printf("topic /%s socket: %d \n", topic_n, *new_socket_ptr);
            struct Topic ros_strct, *ros_strct_ptr;
            ros_strct_ptr = &ros_strct;
            strcpy(ros_strct_ptr->topic , topic_n);
            ros_strct.socket = (*new_socket_ptr)++;
            void *p = &ros_strct;

            pthread_create((client_th_ptr)++, NULL, client_fun, p);
            //pthread_create((client_th_ptr)++, NULL, client_fun, (new_socket_ptr)++);
        }
    }
    return 0; 
} 
