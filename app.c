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
            PVKalmanFilterState kf_a;
            PVKalmanFilterState kf_b;
            double  t;
            double  z;
            char line[INPUT_LINE_LENGTH + 1];

            if ( fgets(line, INPUT_LINE_LENGTH, fp) != NULL )
            {
                line[INPUT_LINE_LENGTH] = '\0';

                if ( sscanf(line, "%lf, %lf", &t, &z) == 2 )
                {
                    double P_0[2][2] = {
                        { 1.0, 0.0 },
                        { 0.0, 0.5 }
                    };

                    int success_a;
                    int success_b;

                    success_a = PVKalmanFilterInit(&kf_a, 1, t, z, P_0, 0.0001, 1.0);
                    success_b = PVKalmanFilterInit(&kf_b, 1, t, z, P_0, 0.01, 1.0);

                    if (success_a == PVKF_SUCCESS && success_b == PVKF_SUCCESS)
                    {
                        puts("t, z, kf_a.x, kf_b.x");
                        printf("%lf, %lf, %lf, %lf\n", t, z, kf_a.x[0], kf_b.x[0]);

                        while ( fgets(line, INPUT_LINE_LENGTH, fp) != NULL )
                        {
                            line[INPUT_LINE_LENGTH] = '\0';

                            sscanf(line, "%lf, %lf", &t, &z);

                            success_a = PVKalmanFilterUpdate(&kf_a, t, z);
                            success_b = PVKalmanFilterUpdate(&kf_b, t, z);

                            if (success_a == PVKF_ERROR || success_b == PVKF_ERROR)
                            {
                                break;
                            }
                            printf("%lf, %lf, %lf, %lf\n", t, z, kf_a.x[0], kf_b.x[0]);
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
