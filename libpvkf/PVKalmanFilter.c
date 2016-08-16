#include "PVKalmanFilter.h"



/* These functions are intended to be used internally,
   but need to be externally accessible for testing */
int _PVKalmanFilter_predict(struct PVKalmanFilterState *state, double t);
int _PVKalmanFilter_correct(struct PVKalmanFilterState *state, double z);



int PVKalmanFilterInit(struct PVKalmanFilterState *state, unsigned id, double t, double z)
{
    int returnCode = PVKF_ERROR;

    if (state)
    {
        state->id = id;

        state->t = t;

        state->x[0] = z;  /* can initialized directly since H = [ 1 , 0 ] */
        state->x[1] = 0.0;

        state->P[0][0] = 1.0;
        state->P[0][1] = 0.0;
        state->P[1][0] = 0.0;
        state->P[1][1] = 0.5;

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

        state->predict = _PVKalmanFilter_predict;
        state->correct = _PVKalmanFilter_correct;

        returnCode = PVKF_SUCCESS;
    }

    return returnCode;
}



int PVKalmanFilterUpdate(struct PVKalmanFilterState *state, double t, double z)
{
    int returnCode = PVKF_ERROR;

    if (state)
    {
        if (state->predict && state->correct)
        {
            if ( state->predict(state, t) == PVKF_SUCCESS )
            {
                returnCode = state->correct(state, z);
            }
        }
    }

    return returnCode;
}



int _PVKalmanFilter_predict(struct PVKalmanFilterState *state, double t)
{
    double  dt;
    double  M2x2[2][2];
    double  M2x1[2];
    int     i, j, k;

    dt = t - state->t;
    state->t = t;

    state->Phi[0][1] = dt;

    state->G[0] = 0.5 * dt * dt;
    state->G[1] = dt;

    /*
     * x(k|k-1) = Phi(k|k-1) x(k-1)
     */

    /* The structure of the Phi matrix allows us to simply this calculation */
    state->x[0] += state->x[1] * dt;

    /*
     * P(k|k-1) = Phi(k|k-1) P(k-1|k-1) Phi(k|k-1)' + G(k) Q(k) G(k)'
     */

    /* M2x2 = Phi(k|k-1) P(k-1|k-1) */
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 2; j++)
        {
            double sum = 0;
            for (k = 0; k < 2; k++)
            {
                sum += state->Phi[i][k] * state->P[k][j];
            }
            M2x2[i][j] = sum;
        }
    }
    /* P(k|k-1) = M2x2 Phi(k|k-1)' */
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 2; j++)
        {
            double sum = 0;
            for (k = 0; k < 2; k++)
            {
                sum += M2x2[i][k] * state->Phi[j][k];
            }
            state->P[i][j] = sum;
        }
    }
    /* M2x1 = G(k) Q(k) */
    M2x1[0] = state->G[0] * state->Q;
    M2x1[1] = state->G[1] * state->Q;
    /* P(k|k-1) += M2x1 G(k)' */
    state->P[0][0] += M2x1[0] * state->G[0];
    state->P[0][1] += M2x1[0] * state->G[1];
    state->P[1][0] += M2x1[1] * state->G[0];
    state->P[1][1] += M2x1[1] * state->G[1];

    return PVKF_SUCCESS;
}



int _PVKalmanFilter_correct(struct PVKalmanFilterState *state, double z)
{
    double  dz;
    double  M1x2[2];
    double  S;
    double  K[2];
    double  M2x2[2][2];
    double  P_k[2][2];
    int     i, j, k;

    /*
     * dz = z(k) - H(k) x(k|k-1)
     */

    dz = z - state->H[0] * state->x[0] + state->H[1] * state->x[1];

    /*
     * S(k) = H(k) P(k|k-1) H(k)' + R(k)
     */

    M1x2[0] = state->H[0] * state->P[0][0] + state->H[1] * state->P[1][0];
    M1x2[1] = state->H[0] * state->P[0][1] + state->H[1] * state->P[1][1];
    S = M1x2[0] * state->H[0] + M1x2[1] * state->H[1] + state->R;
    if (S <= 0)
    {
        return PVKF_ERROR;
    }

    /*
     * K(k) = P(k|k-1) H(k)' / S(k)
     */

    K[0] = (state->P[0][0] * state->H[0] + state->P[0][1] * state->H[1]) / S;
    K[1] = (state->P[1][0] * state->H[0] + state->P[1][1] * state->H[1]) / S;

    /*
     * x(k|k) = x(k|k-1) + K(k) dz
     */

    state->x[0] += K[0] * dz;
    state->x[1] += K[1] * dz;

    /*
     * P(k|k) = [I - K(k) H(k)] P(k|k-1)
     */

    M2x2[0][0] = 1.0 - (K[0] * state->H[0]);
    M2x2[0][1] = - (K[0] * state->H[1]);
    M2x2[1][0] = - (K[1] * state->H[0]);
    M2x2[1][1] = 1.0 - (K[1] * state->H[1]);
    for (i = 0; i < 2; i++)
    {
        for (j = 0; j < 2; j++)
        {
            double sum = 0;
            for (k = 0; k < 2; k++)
            {
                sum += M2x2[i][k] * state->P[k][j];
            }
            P_k[i][j] = sum;
        }
    }
    state->P[0][0] = P_k[0][0];
    state->P[0][1] = P_k[0][1];
    state->P[1][0] = P_k[1][0];
    state->P[1][1] = P_k[1][1];

    return PVKF_SUCCESS;
}
