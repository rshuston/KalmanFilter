#include "PVKalmanFilter.h"

#include "testable.h"



STATIC int _predict(struct PVKalmanFilterState *state, double t);
STATIC int _correct(struct PVKalmanFilterState *state, double z);



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
    int returnCode = PVKF_ERROR;

    if (state)
    {
        /*
         * Prediction:
         */

        /* x(k|k-1) = Phi(k|k-1) x(k-1) */

        /* P(k|k-1) = Phi(k|k-1) P(k-1|k-1) Phi(k|k-1)' + G(k) Q(k) G(k)' */

        returnCode = _predict(state, t);

        /*
         * Correction:
         */

        /* dz = z(k) - H(k) x(k|k-1) */

        /* S(k) = H(k) P(k|k-1) H(k)' + R(k) */

        /* K(k) = P(k|k-1) H(k)' / S(k) */

        /* x(k|k) = x(k|k-1) + K(k) dz */

        /* P(k|k) = [I - K(k) H(k)] P(k|k-1) */

        if (returnCode == PVKF_SUCCESS)
        {
            returnCode = _correct(state, z);
        }
    }

    return returnCode;
}



STATIC int _predict(struct PVKalmanFilterState *state, double t)
{
    /*
     * Prediction:
     */

    /* x(k|k-1) = Phi(k|k-1) x(k-1) */

    /* P(k|k-1) = Phi(k|k-1) P(k-1|k-1) Phi(k|k-1)' + G(k) Q(k) G(k)' */

    return PVKF_ERROR;
}



STATIC int _correct(struct PVKalmanFilterState *state, double z)
{
    /*
     * Correction:
     */

    /* dz = z(k) - H(k) x(k|k-1) */

    /* S(k) = H(k) P(k|k-1) H(k)' + R(k) */

    /* K(k) = P(k|k-1) H(k)' / S(k) */

    /* x(k|k) = x(k|k-1) + K(k) dz */

    /* P(k|k) = [I - K(k) H(k)] P(k|k-1) */

    return PVKF_ERROR;
}
