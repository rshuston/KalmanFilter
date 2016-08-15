#include <stdio.h>
#include <stdlib.h>
#include <check.h>

#include "app.h"

/* ===== app_exec() ========================================================= */

START_TEST (test_app_exec_accepts_impulse_command)
{
    char    *exename;
    char    *input_type;
    char    *argv[2];
    int     returnValue;

    exename = "app_test.exe";
    input_type = "impulse";

    argv[0] = exename;
    argv[1] = input_type;

    returnValue = app_exec(2, argv);  /* 0 = success */

    ck_assert_int_eq(returnValue, 0);
}
END_TEST

START_TEST (test_app_exec_accepts_step_command)
{
    char    *exename;
    char    *input_type;
    char    *argv[2];
    int     returnValue;

    exename = "app_test.exe";
    input_type = "step";

    argv[0] = exename;
    argv[1] = input_type;

    returnValue = app_exec(2, argv);  /* 0 = success */

    ck_assert_int_eq(returnValue, 0);
}
END_TEST

START_TEST (test_app_exec_accepts_noisyramp_command)
{
    char    *exename;
    char    *input_type;
    char    *argv[2];
    int     returnValue;

    exename = "app_test.exe";
    input_type = "noisyramp";

    argv[0] = exename;
    argv[1] = input_type;

    returnValue = app_exec(2, argv);  /* 0 = success */

    ck_assert_int_eq(returnValue, 0);
}
END_TEST

/* ===== Test Suite ========================================================= */

Suite * test_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("app");

    /* Core test case */
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_app_exec_accepts_impulse_command);
    tcase_add_test(tc_core, test_app_exec_accepts_step_command);
    tcase_add_test(tc_core, test_app_exec_accepts_noisyramp_command);

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

    s = test_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    puts("-------------------------------------------------------------------------------");
    putchar('\n');

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
