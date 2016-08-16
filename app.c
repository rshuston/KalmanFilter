#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app.h"
#include "PVKalmanFilter.h"


#define INPUT_LINE_LENGTH   128


int app_exec(int argc, char *argv[])
{
    int returnValue = 1;

    if (argc == 2)
    {
        FILE *fp;

        if ( (fp = fopen(argv[1], "r")) != NULL )
        {
            PVKalmanFilterState kf;
            double  t;
            double  z;
            char line[INPUT_LINE_LENGTH + 1];

            if ( fgets(line, INPUT_LINE_LENGTH, fp) != NULL )
            {
                line[INPUT_LINE_LENGTH] = '\0';

                if ( sscanf(line, "%lf, %lf", &t, &z) == 2 )
                {
                    if ( PVKalmanFilterInit(&kf, 1, t, z) == PVKF_SUCCESS )
                    {
                        printf("%lf, %lf, %lf\n", t, z, kf.x[0]);

                        while ( fgets(line, INPUT_LINE_LENGTH, fp) != NULL )
                        {
                            line[INPUT_LINE_LENGTH] = '\0';

                            sscanf(line, "%lf, %lf", &t, &z);

                            if ( PVKalmanFilterUpdate(&kf, t, z) == PVKF_ERROR )
                            {
                                break;
                            }
                            printf("%lf, %lf, %lf\n", t, z, kf.x[0]);
                        }
                    }
                }
            }

            fclose(fp);
            returnValue = 0;
        }
    }

    return returnValue;
}
