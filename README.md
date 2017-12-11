# KalmanFilter

This kata exercises your ability to successfully test-drive a software solution that employs techniques from control engineering and signal processing.

Assume you are part of a team that is developing a monitoring system that needs to track an object whose single-axis position parameter typically changes at a constant rate (e.g., a remote-controlled rocket sled, a remote-controlled drag race car, etc.).  The system will measure the object's position values along the distance axis at discrete time points, nominally every second, and the measurements will typically be contaminated by noise due to sensor jitter and hardware acquisition timing.  The object position dynamics can be modeled using a linear state transition model with a zero-mean Gaussian random acceleration input, and the hardware position measurement noise can be modeled by a Gaussian noise model of zero mean and unity variance.  All position units are in meters, and all time units are in seconds.

As part of your role on the team, you are to implement a two-state Kalman filter library that can be used to initiate and maintain a position-velocity estimate on a moving object as described in the preceding paragraph.  The completed library will need to be integrated into another application project to which you may or may not have access rights (e.g., its access may be closed due to security clearance reasons).  Therefore, you must deliver a complete and tested code library package.

The code must be written in ANSI C.  The library public module must be named PVKalmanFilter.c along with a companion PVKalmanFilter.h header file.

The PVKalmanFilter.h header must utilize multiple include guards, and it should also use C++ guards to allow the library to be used in C++ projects at some point in the future:
```
#ifndef _PVKalmanFilter_h_
#define _PVKalmanFilter_h_

#ifdef __cplusplus
extern "C" {
#endif

/* Header definitions here */

#ifdef __cplusplus
}
#endif

#endif
```
The PVKalmanFilter.h header must define the two return codes:
```
#define PVKF_SUCCESS    0
#define PVKF_ERROR      (-1)
```
The library must use a structure, defined in the PVKalmanFilter.h header, that contains at least the following fields:
```
typedef struct PVKalmanFilterState
{
    unsigned id;            /* id tag to distinguish this track from others */
    double   t;             /* most recent time of update, seconds */
    double   z;             /* most recent measurement */
    double   x[2];          /* estimated state vector, [ pos, vel ] */
    double   P[2][2];       /* estimated state error covariance: */
                            /* [   var_pos ,  covar_posvel ] */
                            /* [ covar_posvel , var_vel    ] */
    double   q;             /* process noise variance (scalar) */
    double   r;             /* measurement noise variance (scalar) */

    /* ... any additional fields needed to support filter operation ... */

} PVKalmanFilterState;
```
The library must provide at least two public functions:
1. An initialization function that has the following signature:
    ```
    int PVKalmanFilterInit(
            struct PVKalmanFilterState *state,
            unsigned id,
            double t,
            double z,
            double P[2][2],
            double q,
            double r
        )
    /* state = filter state to initialize for accepting subsequent updates */
    /* id = id tag to be assigned to the filter state */
    /* t = time of initial measurement */
    /* z = initial measurement to be used to initialize the state */
    /* P = initial state error covariance: */
    /*     [   var_pos ,  covar_posvel ] */
    /*     [ covar_posvel , var_vel    ] */
    /* q = process noise variance (scalar) */
    /* r = measurement noise variance (scalar) */
    ```
The return code must be PVKF_SUCCESS (0) for success, and PVKF_ERROR (-1) for any errors that may be detected during the initialization (e.g., a NULL state pointer).
1. A one-step update driver function that has the following signature:
    ```
    int PVKalmanFilterUpdate(
            struct PVKalmanFilterState *state,
            double t,
            double z
        )
    /* state = filter state to update with new measurement */
    /* t = time of new measurement */
    /* z = the new measurement */
    ```
The return code must be PVKF_SUCCESS (0) for success, and PVKF_ERROR (-1) for any errors that may be detected during the update (e.g., a NULL state pointer).  If an error does occur, the filter state values listed above must be preserved to what they were prior to the PVKalmanFilterUpdate() invocation.

You must design your filter so that it has an impulse response of no more than 30% once the filter has achieved a sufficiently steady state (i.e., it rejects at least 70% of a noise spike).  For this kata, you can assume sufficient steady state behavior after 10 filter updates for a suitable choice of process noise variance and unity noise variance.

Do not assume that the time points will be regularly spaced;  they could jitter by 0.05 s or more due to sensor acquisition timing.

The library code must be implemented such that its source files can be integrated into an application's makefile build procedures such that the application needs to rely only on the information available in PVKalmanFilter.h in order to use the library.  The application will allocate a new PVKalmanFilterState structure, initiate a new track using PVKalmanFilterInit(), and update it with PVKalmanFilterUpdate().  As part of your submission, you must specify the initial values you have chosen for P[][], q, and r for the call to PVKalmanFilterInit().  In addition, PVKalmanFilterInit() must initialize the x[] field of the filter state from the initial measurement, z.

You are allowed to amend the required PVKalmanFilterState data fields as necessary to support filter operation across successive PVKalmanFilterUpdate() invocations.

You may implement additional source files in your library in addition to PVKalmanFilter.c and PVKalmanFilter.h, but PVKalmanFilter.h must be the interface file that the application uses to run your filter.  All source files must be provided so that they can be integrated into the main application's makefile.

The library implementation must be readable and understandable such that it is easy to follow the logic and the mechanics of the Kalman filter equations.  (Comments are not evil, but use them judiciously.)

The library code must be architected so that it does not perform any block memory management (i.e., no malloc()/free() cycles), and the library cannot throw asserts.  In addition, you must not use file-scope global variables to maintain filter state; assume that more than one tracking filter may be running at a time.

The library should be implemented as simply as needed to provide proper functionality. There exist a number of advanced Kalman filtering techniques and formulations that are suited for mission-critical and/or high-end applications, but they are outside the scope of this kata.  Don't over-think or over-complicate your solution.

The library should be implemented efficiently and without wasteful execution overhead. Matrix multiplication loops, if used, should be concise;  you should assume that the compiler can optimize and even vectorize the matrix operations.

Please provide a README file that identifies the essential source files that need to be integrated into the application makefile.  The README file should also present a brief summary or description of the activities you performed to develop the library, how you determined test data, etc.  It doesn’t need to be elaborate, but it should highlight the steps you’ve taken to ensure the filter is correct.  Think of the application integration engineer as your primary customer;  you are providing him/her with a completed Kalman filter package to be integrated into the application and, once added to the project makefile, the application should compile and run with your integrated library.

The integration development environment is as follows:
* The code must build and run on Ubuntu 14.04 LTS or Ubuntu 16.04 LTS.
* The code must build and run using GCC as the compiler toolchain and GNU make as the project build environment.
* You must use Git for version control.
* You must use the libcheck (https://github.com/libcheck/check) testing framework to develop the PVKalmanFilter library, and you must include the testing files and project files with your submission to demonstrate proper behavior.
