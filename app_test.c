#include <stdio.h>
#include <stdlib.h>
#include <check.h>

#include "app.h"

/* ===== app_exec() ========================================================= */

START_TEST (test_app_exec_rejects_unknown_filename_argument)
{
    char    *exename;
    char    *filename;
    char    *argv[2];
    int     returnValue;

    exename = "app_test.exe";
    filename = "foo.bar";

    argv[0] = exename;
    argv[1] = filename;

    returnValue = app_exec(2, argv);  /* 0 = success */

    ck_assert_int_eq(returnValue, 1);
}
END_TEST

START_TEST (test_app_exec_accepts_known_filename_argument)
{
    char    *exename;
    char    *filename;
    char    *argv[2];
    FILE    *file;
    char    *data;
    int     returnValue;

    exename = "app_test.exe";
    filename = "/tmp/kalman_filter_test_input.dat";

    argv[0] = exename;
    argv[1] = filename;

    file = fopen(filename, "w");
    ck_assert_ptr_ne(file, NULL);

    data = "0.000000, 0.000000\n"
           "1.000000, 1.000000\n";
    fputs(data, file);
    fclose(file);

    returnValue = app_exec(2, argv);  /* 0 = success */

    remove(filename);

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

    tcase_add_test(tc_core, test_app_exec_rejects_unknown_filename_argument);
    tcase_add_test(tc_core, test_app_exec_accepts_known_filename_argument);

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
