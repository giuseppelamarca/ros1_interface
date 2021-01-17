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
    semid = semget(SEM_KEY,0, 0);
    if(semid == -1){
        errExit("Error getting semaphore");
    }
    shmid = shmget(SHM_KEY, 0, 0);
    if (shmid == -1){
        errExit("Error in get shared memory");
    }
    shmp = shmat(shmid, NULL, SHM_RDONLY);
    if (shmp == (void *) -1){
        errExit("Shmat error");
    }
    for (int i = 0; i < 10; i++){
        if (reserveSem(semid, READ_SEM) == -1){
            errExit("Error in semaphore reservation");
        }

        if (write(STDOUT_FILENO, shmp->buf, shmp->cnt) != shmp->cnt){
            errExit("Error in writing");
        }
        if (releaseSem(semid, WRITE_SEM) == -1){
            errExit("Error in releasing the semaphore");
        }
    }
    if (shmdt(shmp) == -1){
        errExit("Shmdt error");
    }
    exit(EXIT_SUCCES);
}