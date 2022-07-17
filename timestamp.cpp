#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <iomanip>
#include <unistd.h>
#include <sys/timeb.h>


char*   log_Time(void)
{
        struct  tm      *ptm;
        struct  timeb   stTimeb;
        static  char    szTime[19];
 
        ftime(&stTimeb);
        ptm = localtime(&stTimeb.time);
        sprintf(szTime, "%02d-%02d %02d:%02d:%02d.%03d",
                ptm->tm_mon+1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm);
        szTime[18] = 0;
        return szTime;
}

void signalHandler(int signo)
{
    switch (signo){
        case SIGALRM:
            printf("[%s]\n", log_Time());
            printf("Caught the SIGALRM signal!\n");
            break;
   }
}

int main(int argc, char *argv[])
{
    signal(SIGALRM, signalHandler);

    struct itimerval new_value, old_value;
    new_value.it_value.tv_sec = 0;
    new_value.it_value.tv_usec = 1;
    new_value.it_interval.tv_sec = 0;
    new_value.it_interval.tv_usec = 1000*30;
    setitimer(ITIMER_REAL, &new_value, &old_value);
    
    for(;;);
     
    return 0;
}