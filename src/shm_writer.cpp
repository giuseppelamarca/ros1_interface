#include <sys/types.h>
#include <sys/stat.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include "binary_sems.h"
#include "tlpi_hdr.h"


#define SHM_KEY 0X1234
#define SEM_KEY 0X5678
#define OBJ_PERMS (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP)
#define WRITE_SEM 0
#define READ_SEM 1
#define BUF_SIZE 1024

struct shmseg{
    int cnt;
    char buf[BUF_SIZE];
}



int main(int argc, char *argv[]){
    semid = semget(SEM_KEY, 2, IPC_CREAT | OBJ_PERMS);
    if(semid == -1){
        errExit("Error getting semaphore");
    }
    if (initSemAvailable(semid, WRITE_SEM) == -1 ){
        errExit("Error initializating the semaphore");
    }
    if (initSemInUse(semid, READ_SEM) == -1 ){
        errExit("Error in semaphore use");
    }
    shmid = shmget(SHM_KEY, sizeof(struct shmseg), IPC_CREAT | OBJ_PERMS);
    if (shmid == -1){
        errExit("Error in get shared memory");
    }
    shmp = shmat(shmid, NULL, 0);
    if (shmp == (void *) -1){
        errExit("Shmat error");
    }
    for (int i = 0; i < 10; i++){
        if (reserveSem(semid, WRITE_SEM) == -1){
            errExit("Error in semaphore reservation");
        }

        shmp->cnt = read(STDIN_FILENO, shmp->buf, BUF_SIZE);
        if ( shmp->cnt == -1){
            errExit("Error in reading");
        }
        if (releaseSem(semid, READ_SEM) == -1){
            errExit("Error in releasing the semaphore");
        }
    }
    if (shmdt(shmp) == -1){
        errExit("Shmdt error");
    }
    if (shmctl(shmid, IPC_RMID, 0) == -1){
        errExit("Shmctl error");
    }    
    exit(EXIT_SUCCES);
}