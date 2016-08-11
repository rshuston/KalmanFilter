#ifndef _PVKalmanFilter_h_
#define _PVKalmanFilter_h_


#ifdef __cplusplus
extern "C" {
#endif


#define PVKF_SUCCESS    0
#define PVKF_ERROR      (-1)


typedef struct PVKalmanFilterState
{

    /*
     * Essential properties
     */

    unsigned id;    /* an id tag to distinguish this track from others */
    double   t;     /* most recent time of update, seconds */
    double   x[2];  /* estimated state vector, [ pos, vel ] */

    /*
     * Additional properties
     */

     /* Process model */
     double Phi[2][2];
     double G[2];
     double Q;

     /* Measurement model */
     double H[2];
     double R;

     /* State covariance */
     double P[2][2];

} PVKalmanFilterState;


extern int PVKalmanFilterInit(struct PVKalmanFilterState *state, unsigned id, double t, double z);

extern int PVKalmanFilterUpdate(struct PVKalmanFilterState *state, double t, double z);


#ifdef __cplusplus
}
#endif


#endif
