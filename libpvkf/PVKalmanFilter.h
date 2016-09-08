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

    unsigned id;        /* id tag to distinguish this track from others */
    double   t;         /* most recent time of update, seconds */
    double   z;         /* most recent measurement */
    double   x[2];      /* estimated state vector, [ pos, vel ] */
    double   P[2][2];   /* estimated state error covariance: */
                        /* [   var_pos ,  covar_posvel ] */
                        /* [ covar_posvel , var_vel    ] */

    /*
     * Additional properties to be used internally by PVKalmanFilter
     */

    /* Process noise (scalar for our model) */
    double Q;

    /* Measurement noise (scalar) */
    double R;

    /* Prediction and correction function pointers - offers testability */
    int (*predict)(struct PVKalmanFilterState *state, double t);
    int (*correct)(struct PVKalmanFilterState *state, double z);

} PVKalmanFilterState;


/*
 * PVKalmanFilterInit()
 *
 * Initializes a PVKalmanFilterState structure
 *
 * state = filter state to initialize for accepting subsequent updates
 * id = id tag to be assigned to filter state
 * t = time of initial measurement
 * z = initial measurement to be used to initialize the state
 * P = initial state error covariance:
 *     [   var_pos ,  covar_posvel ]
 *     [ covar_posvel , var_vel    ]
 */

extern int PVKalmanFilterInit(struct PVKalmanFilterState *state, unsigned id, double t, double z, double P[2][2]);


/*
 * PVKalmanFilterUpdate()
 *
 * Updates a PVKalmanFilterState structure with a new measurement
 *
 * state = filter state to initialize for accepting subsequent updates
 * t = time of new measurement
 * z = the new measurement
 */

extern int PVKalmanFilterUpdate(struct PVKalmanFilterState *state, double t, double z);


#ifdef __cplusplus
}
#endif


#endif
