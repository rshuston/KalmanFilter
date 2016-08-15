#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app.h"
#include "PVKalmanFilter.h"



int app_exec(int argc, char *argv[])
{
    int returnValue = 1;

    if (argc == 2)
    {
        if ( strcmp("impulse", argv[1]) == 0 )
        {
            returnValue = 0;
        }
        else if ( strcmp("step", argv[1]) == 0 )
        {
            returnValue = 0;
        }
        else if ( strcmp("noisyramp", argv[1]) == 0 )
        {
            returnValue = 0;
        }
    }

    return returnValue;
}
