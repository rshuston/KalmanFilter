#include <stdlib.h>
#include <check.h>

#include "PVKalmanFilter.h"



START_TEST (test_PVKalmanFilterInit_handles_NULL_state)
{
    int result;

    result = PVKalmanFilterInit(NULL);

    ck_assert_int_eq(result, PVKF_ERROR);
}
END_TEST



START_TEST (test_PVKalmanFilterUpdate_handles_NULL_state)
{
    int result;

    result = PVKalmanFilterUpdate(NULL, 0, 0);

    ck_assert_int_eq(result, PVKF_ERROR);
}
END_TEST



Suite * PVKalmanFilter_test_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("PVKalmanFilter");

    /* Core test case */
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_PVKalmanFilterInit_handles_NULL_state);
    tcase_add_test(tc_core, test_PVKalmanFilterUpdate_handles_NULL_state);

    suite_add_tcase(s, tc_core);

    return s;
}



int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    s = PVKalmanFilter_test_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
