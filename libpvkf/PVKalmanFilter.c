#include "PVKalmanFilter.h"



int PVKalmanFilterInit(struct PVKalmanFilterState *state, unsigned id, double t, double z)
{
    int returnCode = PVKF_ERROR;

    if (state)
    {
        state->Phi[0][0] = 1;
        state->Phi[0][1] = 0;  /* will be set to dt during update */
        state->Phi[1][0] = 0;
        state->Phi[1][1] = 1;

        state->G[0] = 0.5;  /* will be set to 0.5 * dt * dt during update */
        state->G[1] = 1;    /* will be set to dt during update */

        state->Q = 0.0001;

        state->H[0] = 1;
        state->H[1] = 0;

        state->R = 1;

        state->t = t;

        state->x[0] = z;  /* can initialized directly since H = [ 1 , 0 ] */
        state->x[1] = 0.0;

        state->id = id;

        returnCode = PVKF_SUCCESS;
    }

    return returnCode;
}



int PVKalmanFilterUpdate(struct PVKalmanFilterState *state, double t, double z)
{
    return PVKF_ERROR;
}
