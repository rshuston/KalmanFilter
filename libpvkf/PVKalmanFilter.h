#ifndef _PVKalmanFilter_h_
#define _PVKalmanFilter_h_


#define PVKF_SUCCESS    0
#define PVKF_ERROR      (-1)


struct PVKalmanFilterState
{
    unsigned id;    /* an id tag to distinguish this track from others */
    double   t;     /* most recent time of update, seconds */
    double   x[2];  /* estimated state vector, [ pos, vel ] */

    /* ... any additional parameters needed to maintain track state ... */

};


extern int PVKalmanFilterInit(struct PVKalmanFilterState *state);

extern int PVKalmanFilterUpdate(struct PVKalmanFilterState *state, double t, double z);


#endif
