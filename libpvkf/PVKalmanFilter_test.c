#include <stdio.h>
#include <stdlib.h>
#include <check.h>

#include "PVKalmanFilter.h"

/* ===== PVKalmanFilterInit() =============================================== */

START_TEST (test_PVKalmanFilterInit_handles_NULL_state)
{
    int result;

    result = PVKalmanFilterInit(NULL, 0, 0, 0);

    ck_assert_int_eq(result, PVKF_ERROR);
}
END_TEST

START_TEST (test_PVKalmanFilterInit_initializes_state)
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

/* ===== PVKalmanFilterUpdate() ============================================= */

START_TEST (test_PVKalmanFilterUpdate_handles_NULL_state)
{
    int result;

    result = PVKalmanFilterUpdate(NULL, 0, 0);

    ck_assert_int_eq(result, PVKF_ERROR);
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

    tcase_add_test(tc_core, test_PVKalmanFilterInit_handles_NULL_state);
    tcase_add_test(tc_core, test_PVKalmanFilterInit_initializes_state);

    tcase_add_test(tc_core, test_PVKalmanFilterUpdate_handles_NULL_state);

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
