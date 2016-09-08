#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <check.h>
#include <math.h>

#include "PVKalmanFilter.h"

#define DOUBLE_TOLERANCE    1e-5

/*
 * Testable functions
 */

int _PVKalmanFilter_predict(struct PVKalmanFilterState *state, double t);
int _PVKalmanFilter_correct(struct PVKalmanFilterState *state, double z);

/*
 * Mock functions
 */

static int _mock_predict(struct PVKalmanFilterState *state, double t)
{
    return PVKF_ERROR;
}

static int _mock_correct(struct PVKalmanFilterState *state, double z)
{
    return PVKF_ERROR;
}

/* ===== PVKalmanFilterInit() =============================================== */

START_TEST (PVKalmanFilter_test_PVKalmanFilterInit_handles_NULL_state)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    int result;

    result = PVKalmanFilterInit(NULL, 0, 0, 0, P_0);

    ck_assert_int_eq(result, PVKF_ERROR);
}
END_TEST

START_TEST (PVKalmanFilter_test_PVKalmanFilterInit_initializes_state)
{
    unsigned id = 1;
    double t = 2.0;
    double z = 3.0;
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, id, t, z, P_0);

    ck_assert_int_eq(result, PVKF_SUCCESS);

    ck_assert_uint_eq(state.id, id);

    ck_assert(state.t == t);
    ck_assert(state.z == z);
    ck_assert(state.x[0] == z);
    ck_assert(state.x[1] == 0.0);

    ck_assert(state.P[0][0] == P_0[0][0]);
    ck_assert(state.P[0][1] == P_0[0][1]);
    ck_assert(state.P[1][0] == P_0[1][0]);
    ck_assert(state.P[1][1] == P_0[1][1]);

    ck_assert(state.Phi[0][0] == 1.0);
    ck_assert(state.Phi[0][1] == 0.0);
    ck_assert(state.Phi[1][0] == 0.0);
    ck_assert(state.Phi[1][1] == 1.0);

    ck_assert(state.G[0] == 0.5);
    ck_assert(state.G[1] == 1.0);

    ck_assert(state.Q == 0.0001);

    ck_assert(state.H[0] == 1.0);
    ck_assert(state.H[1] == 0.0);

    ck_assert(state.R == 1.0);
}
END_TEST

/* ===== _PVKalmanFilter_predict() ========================================== */

START_TEST (PVKalmanFilter_test_predict_updates_intermediate_state_values_initial_state_mode)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0, P_0);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    /* Set time stamp to a priori value */
    state.t = 0.0;

    /* Set state vector to a priori state value */
    state.x[0] = 0.0;
    state.x[1] = 0.0;

    /* Set state error covariance to initial state value */
    state.P[0][0] = 1.0;
    state.P[0][1] = 0.0;
    state.P[1][0] = 0.0;
    state.P[1][1] = 0.5;

    result = _PVKalmanFilter_predict(&state, 1);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    ck_assert(state.t == 1.0);
    ck_assert(fabs(0.0 - state.x[0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0 - state.x[1]) <= DOUBLE_TOLERANCE);

    ck_assert(fabs(1.50002 - state.P[0][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.50005 - state.P[0][1]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.50005 - state.P[1][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.50010 - state.P[1][1]) <= DOUBLE_TOLERANCE);

    ck_assert(state.Phi[0][0] == 1.0);
    ck_assert(state.Phi[0][1] == 1.0);
    ck_assert(state.Phi[1][0] == 0.0);
    ck_assert(state.Phi[1][1] == 1.0);

    ck_assert(state.G[0] == 0.5);
    ck_assert(state.G[1] == 1.0);
}
END_TEST

START_TEST (PVKalmanFilter_test_predict_updates_intermediate_state_values_steady_state_mode)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0, P_0);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    /* Set time stamp to a priori value */
    state.t = 0.0;

    /* Set state vector to a priori state value */
    state.x[0] = 0.0;
    state.x[1] = 0.0;

    /* Set state error covariance to steady state value */
    state.P[0][0] = 0.1318503;
    state.P[0][1] = 0.0093175;
    state.P[1][0] = 0.0093175;
    state.P[1][1] = 0.0013651;

    result = _PVKalmanFilter_predict(&state, 1);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    ck_assert(state.t == 1.0);
    ck_assert(fabs(0.0 - state.x[0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0 - state.x[1]) <= DOUBLE_TOLERANCE);

    ck_assert(fabs(0.1518751 - state.P[0][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0107326 - state.P[0][1]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0107326 - state.P[1][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0014651 - state.P[1][1]) <= DOUBLE_TOLERANCE);

    ck_assert(state.Phi[0][0] == 1.0);
    ck_assert(state.Phi[0][1] == 1.0);
    ck_assert(state.Phi[1][0] == 0.0);
    ck_assert(state.Phi[1][1] == 1.0);

    ck_assert(state.G[0] == 0.5);
    ck_assert(state.G[1] == 1.0);
}
END_TEST

/* ===== _PVKalmanFilter_correct() ========================================== */

START_TEST (PVKalmanFilter_test_correct_updates_final_state_values_initial_state_mode)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    double z = 1.0;
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0, P_0);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    /* Set time stamp */
    state.t = 1.0;

    /* Set state transition matrix */
    state.P[0][0] = 1.0;
    state.P[0][1] = 1.0;
    state.P[1][0] = 0.0;
    state.P[1][1] = 1.0;

    /* Set process transition matrix */
    state.G[0] = 0.5;
    state.G[1] = 1.0;

    /* Set state vector to intermediate state value */
    state.x[0] = 0.0;
    state.x[1] = 0.0;

    /* Set state error covariance to intermediate state value */
    state.P[0][0] = 1.50002;
    state.P[0][1] = 0.50005;
    state.P[1][0] = 0.50005;
    state.P[1][1] = 0.50010;

    result = _PVKalmanFilter_correct(&state, z);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    ck_assert(state.z == z);
    
    ck_assert(fabs(0.60000 - state.x[0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.20002 - state.x[1]) <= DOUBLE_TOLERANCE);

    ck_assert(fabs(0.60000 - state.P[0][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.20002 - state.P[0][1]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.20002 - state.P[1][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.40008 - state.P[1][1]) <= DOUBLE_TOLERANCE);
}
END_TEST

START_TEST (PVKalmanFilter_test_correct_updates_final_state_values_steady_state_mode)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0, P_0);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    /* Set time stamp */
    state.t = 1.0;

    /* Set state transition matrix */
    state.P[0][0] = 1.0;
    state.P[0][1] = 1.0;
    state.P[1][0] = 0.0;
    state.P[1][1] = 1.0;

    /* Set process transition matrix */
    state.G[0] = 0.5;
    state.G[1] = 1.0;

    /* Set state vector to intermediate state value */
    state.x[0] = 0.0;
    state.x[1] = 0.0;

    /* Set state error covariance to intermediate state value */
    state.P[0][0] = 0.1518751;
    state.P[0][1] = 0.0107326;
    state.P[1][0] = 0.0107326;
    state.P[1][1] = 0.0014651;

    result = _PVKalmanFilter_correct(&state, 1);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    ck_assert(fabs(0.1318503 - state.x[0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0093175 - state.x[1]) <= DOUBLE_TOLERANCE);

    ck_assert(fabs(0.1318503 - state.P[0][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0093175 - state.P[0][1]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0093175 - state.P[1][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0013651 - state.P[1][1]) <= DOUBLE_TOLERANCE);
}
END_TEST

/* ===== PVKalmanFilterUpdate() ============================================= */

START_TEST (PVKalmanFilter_test_PVKalmanFilterUpdate_handles_NULL_state)
{
    int result;

    result = PVKalmanFilterUpdate(NULL, 0, 0);

    ck_assert_int_eq(result, PVKF_ERROR);
}
END_TEST

START_TEST (PVKalmanFilter_test_PVKalmanFilterUpdate_handles_predict_error_conditions)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;
    PVKalmanFilterState apriori_state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0, P_0);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    state.predict = _mock_predict;

    apriori_state = state;

    result = PVKalmanFilterUpdate(&state, 1, 1);

    ck_assert_int_eq(result, PVKF_ERROR);

    result = memcmp(&state, &apriori_state, sizeof(PVKalmanFilterState));
    ck_assert_int_eq(result, 0);
}
END_TEST

START_TEST (PVKalmanFilter_test_PVKalmanFilterUpdate_handles_correct_error_conditions)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;
    PVKalmanFilterState apriori_state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0, P_0);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    state.correct = _mock_correct;

    apriori_state = state;

    result = PVKalmanFilterUpdate(&state, 1, 1);

    ck_assert_int_eq(result, PVKF_ERROR);

    result = memcmp(&state, &apriori_state, sizeof(PVKalmanFilterState));
    ck_assert_int_eq(result, 0);
}
END_TEST

START_TEST (PVKalmanFilter_test_PVKalmanFilterUpdate_updates_state_for_unit_input_initial_state_mode)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0, P_0);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    /* Set state error covariance to initial state value */
    state.P[0][0] = 1.0;
    state.P[0][1] = 0.0;
    state.P[1][0] = 0.0;
    state.P[1][1] = 0.5;

    result = PVKalmanFilterUpdate(&state, 1, 1);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    ck_assert(state.t == 1.0);
    ck_assert(fabs(0.60000 - state.x[0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.20002 - state.x[1]) <= DOUBLE_TOLERANCE);

    ck_assert(fabs(0.60000 - state.P[0][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.20002 - state.P[0][1]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.20002 - state.P[1][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.40008 - state.P[1][1]) <= DOUBLE_TOLERANCE);
}
END_TEST

START_TEST (PVKalmanFilter_test_PVKalmanFilterUpdate_updates_state_for_unit_input_steady_state_mode)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0, P_0);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    /* Set state error covariance to steady state value */
    state.P[0][0] = 0.1318503;
    state.P[0][1] = 0.0093175;
    state.P[1][0] = 0.0093175;
    state.P[1][1] = 0.0013651;

    result = PVKalmanFilterUpdate(&state, 1, 1);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    ck_assert(state.t == 1.0);
    ck_assert(fabs(0.1318503 - state.x[0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0093175 - state.x[1]) <= DOUBLE_TOLERANCE);

    ck_assert(fabs(0.1318503 - state.P[0][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0093175 - state.P[0][1]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0093175 - state.P[1][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0013651 - state.P[1][1]) <= DOUBLE_TOLERANCE);
}
END_TEST

/* ===== PVKalmanFilter Behavior ============================================ */

START_TEST (PVKalmanFilter_test_PVKalmanFilter_impulse_response_test)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;
    int i;

    PVKalmanFilterInit(&state, 1, 0.0, 0.0, P_0);

    /* before impulse */
    PVKalmanFilterUpdate(&state, 1.0, 0.0);

    for (i = 2; i < 10; i++)
    {
        PVKalmanFilterUpdate(&state, (double) i, 0.0);
    }

    /* at impulse */
    PVKalmanFilterUpdate(&state, 10.0, 1.0);
    ck_assert(state.t == 10.0);
    ck_assert(state.x[0] <= 1.3);  /* not more than 30% overshoot after 10 updates */
}
END_TEST

START_TEST (PVKalmanFilter_test_PVKalmanFilter_handles_step_input_update_cycle)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;
    int i;

    PVKalmanFilterInit(&state, 1, 0.0, 0.0, P_0);

    /* before step jump */
    PVKalmanFilterUpdate(&state, 1.0, 0.0);

    for (i = 2; i < 10; i++)
    {
        PVKalmanFilterUpdate(&state, (double) i, 0.0);
    }

    /* at and after step jump */
    for (i = 10; i <= 20; i++)
    {
        PVKalmanFilterUpdate(&state, (double) i, 1.0);
    }

    ck_assert(state.t == 20.0);
    ck_assert(fabs(1.233112 - state.x[0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.070204 - state.x[1]) <= DOUBLE_TOLERANCE);

    ck_assert(fabs(0.183695 - state.P[0][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.014845 - state.P[0][1]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.014845 - state.P[1][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.002013 - state.P[1][1]) <= DOUBLE_TOLERANCE);
}
END_TEST

START_TEST (PVKalmanFilter_test_PVKalmanFilter_handles_irregular_time_steps)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;

    PVKalmanFilterInit(&state, 1, 0.0, 0.0, P_0);

    PVKalmanFilterUpdate(&state, 1.029114, 0.000000);
    PVKalmanFilterUpdate(&state, 2.025499, 0.000000);
    PVKalmanFilterUpdate(&state, 2.912959, 0.000000);
    PVKalmanFilterUpdate(&state, 4.032272, 0.000000);
    PVKalmanFilterUpdate(&state, 4.913566, 0.000000);
    PVKalmanFilterUpdate(&state, 6.009876, 0.000000);
    PVKalmanFilterUpdate(&state, 6.993264, 0.000000);
    PVKalmanFilterUpdate(&state, 8.087805, 0.000000);
    PVKalmanFilterUpdate(&state, 8.948990, 0.000000);
    PVKalmanFilterUpdate(&state, 10.058553, 0.029276);
    PVKalmanFilterUpdate(&state, 10.984303, 0.492151);
    PVKalmanFilterUpdate(&state, 11.962983, 0.981491);
    PVKalmanFilterUpdate(&state, 12.991874, 1.495937);
    PVKalmanFilterUpdate(&state, 13.893794, 1.946897);
    PVKalmanFilterUpdate(&state, 15.073435, 2.536717);
    PVKalmanFilterUpdate(&state, 15.906594, 2.953297);
    PVKalmanFilterUpdate(&state, 17.001624, 3.500812);
    PVKalmanFilterUpdate(&state, 17.929510, 3.964755);
    PVKalmanFilterUpdate(&state, 18.978909, 4.489455);
    PVKalmanFilterUpdate(&state, 19.966398, 4.983199);

    ck_assert(fabs(19.966398 - state.t) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(3.834134 - state.x[0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.260854 - state.x[1]) <= DOUBLE_TOLERANCE);
}
END_TEST

START_TEST (PVKalmanFilter_test_PVKalmanFilter_covariance_settles_to_steady_state_value)
{
    double P_0[2][2] = {
        { 1.0, 0.0 },
        { 0.0, 0.5 }
    };
    PVKalmanFilterState state;
    int i;

    PVKalmanFilterInit(&state, 1, 0.0, 0.0, P_0);

    for (i = 1; i < 128; i++)
    {
        PVKalmanFilterUpdate(&state, (double) i, 0.0);
    }

    ck_assert(fabs(0.1318503 - state.P[0][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0093175 - state.P[0][1]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0093175 - state.P[1][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0013651 - state.P[1][1]) <= DOUBLE_TOLERANCE);
}
END_TEST

/* ===== Test Suite ========================================================= */

Suite * PVKalmanFilter_test_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("PVKalmanFilter");

    /* Core test case */
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilterInit_handles_NULL_state);
    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilterInit_initializes_state);

    tcase_add_test(tc_core, PVKalmanFilter_test_predict_updates_intermediate_state_values_initial_state_mode);
    tcase_add_test(tc_core, PVKalmanFilter_test_predict_updates_intermediate_state_values_steady_state_mode);

    tcase_add_test(tc_core, PVKalmanFilter_test_correct_updates_final_state_values_initial_state_mode);
    tcase_add_test(tc_core, PVKalmanFilter_test_correct_updates_final_state_values_steady_state_mode);

    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilterUpdate_handles_NULL_state);
    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilterUpdate_handles_predict_error_conditions);
    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilterUpdate_handles_correct_error_conditions);
    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilterUpdate_updates_state_for_unit_input_initial_state_mode);
    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilterUpdate_updates_state_for_unit_input_steady_state_mode);

    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilter_impulse_response_test);
    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilter_handles_step_input_update_cycle);
    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilter_handles_irregular_time_steps);
    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilter_covariance_settles_to_steady_state_value);

    suite_add_tcase(s, tc_core);

    return s;
}

/* ===== Test Driver ======================================================== */

int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    putchar('\n');
    puts("-------------------------------------------------------------------------------");

    s = PVKalmanFilter_test_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    puts("-------------------------------------------------------------------------------");
    putchar('\n');

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
