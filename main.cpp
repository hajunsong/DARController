#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include "ControlMain/controlmain.h"

static struct sigaction sigIntHandler;
static int sig = 0;
static ControlMain *controlMain;

void my_handler(int s)
{
    if (sig == 0)
    {
        sig = s;
        delete controlMain;
        printf("\n Finished DAR Program \n");
        exit(1);
    }
}

int main()
{
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);

    printf("\n Start DAR Program \n");

    controlMain = new ControlMain();
    controlMain->start();
    usleep(1000000);

    pause();

    return 0;
}

