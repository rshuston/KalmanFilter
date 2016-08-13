#include <stdio.h>
#include <stdlib.h>
#include <check.h>
#include <math.h>

#include "PVKalmanFilter.h"

#define DOUBLE_TOLERANCE    1e-5

/*
 * TESTABLE functions
 */

int _predict(struct PVKalmanFilterState *state, double t);
int _correct(struct PVKalmanFilterState *state, double z);

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
    int result;

    result = PVKalmanFilterInit(NULL, 0, 0, 0);

    ck_assert_int_eq(result, PVKF_ERROR);
}
END_TEST

START_TEST (PVKalmanFilter_test_PVKalmanFilterInit_initializes_state)
{
    unsigned id = 1;
    double t = 2.0;
    double z = 3.0;
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, id, t, z);

    ck_assert_int_eq(result, PVKF_SUCCESS);

    ck_assert_uint_eq(state.id, id);

    ck_assert(state.t == t);
    ck_assert(state.x[0] == z);
    ck_assert(state.x[1] == 0.0);

    ck_assert(state.P[0][0] == 1.0);
    ck_assert(state.P[0][1] == 0.0);
    ck_assert(state.P[1][0] == 0.0);
    ck_assert(state.P[1][1] == 0.5);

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

/* ===== _predict() ========================================================= */

START_TEST (PVKalmanFilter_test_predict_updates_intermediate_state_values)
{
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0);
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

    result = _predict(&state, 1);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    ck_assert(state.t == 1.0);
    ck_assert(fabs(0.0 - state.x[0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.0 - state.x[1]) <= DOUBLE_TOLERANCE);

    ck_assert(fabs(0.1518754 - state.P[0][0]) <= DOUBLE_TOLERANCE);
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

/* ===== _correct() ========================================================= */


START_TEST (PVKalmanFilter_test_correct_updates_final_state_values)
{
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0);
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
    state.P[0][0] = 0.1518754;
    state.P[0][1] = 0.0107326;
    state.P[1][0] = 0.0107326;
    state.P[1][1] = 0.0014651;

    result = _correct(&state, 1);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    ck_assert(fabs(0.131851 - state.x[0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.009318 - state.x[1]) <= DOUBLE_TOLERANCE);

    ck_assert(fabs(0.131851 - state.P[0][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.009318 - state.P[0][1]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.009318 - state.P[1][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.001365 - state.P[1][1]) <= DOUBLE_TOLERANCE);
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
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    state.predict = _mock_predict;

    result = PVKalmanFilterUpdate(&state, 0, 0);

    ck_assert_int_eq(result, PVKF_ERROR);
}
END_TEST

START_TEST (PVKalmanFilter_test_PVKalmanFilterUpdate_handles_correct_error_conditions)
{
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    state.predict = _mock_correct;

    result = PVKalmanFilterUpdate(&state, 0, 0);

    ck_assert_int_eq(result, PVKF_ERROR);
}
END_TEST

START_TEST (PVKalmanFilter_test_PVKalmanFilterUpdate_updates_state_for_unit_input)
{
    PVKalmanFilterState state;
    int result;

    result = PVKalmanFilterInit(&state, 1, 0, 0);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    /* Set state error covariance to steady state value */
    state.P[0][0] = 0.13185;
    state.P[0][1] = 0.0093175;
    state.P[1][0] = 0.0093175;
    state.P[1][1] = 0.0013651;

    result = PVKalmanFilterUpdate(&state, 1, 1);
    ck_assert_int_eq(result, PVKF_SUCCESS);

    ck_assert(state.t == 1.0);
    ck_assert(fabs(0.131851 - state.x[0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.009318 - state.x[1]) <= DOUBLE_TOLERANCE);

    ck_assert(fabs(0.131851 - state.P[0][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.009318 - state.P[0][1]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.009318 - state.P[1][0]) <= DOUBLE_TOLERANCE);
    ck_assert(fabs(0.001365 - state.P[1][1]) <= DOUBLE_TOLERANCE);
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

    tcase_add_test(tc_core, PVKalmanFilter_test_predict_updates_intermediate_state_values);
    tcase_add_test(tc_core, PVKalmanFilter_test_correct_updates_final_state_values);

    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilterUpdate_handles_NULL_state);
    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilterUpdate_handles_predict_error_conditions);
    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilterUpdate_handles_correct_error_conditions);
    tcase_add_test(tc_core, PVKalmanFilter_test_PVKalmanFilterUpdate_updates_state_for_unit_input);

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
