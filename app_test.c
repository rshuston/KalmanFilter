#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <check.h>

#include "app.h"

#if 1
#define CAPTURE_STDOUT
#endif

#if defined(CAPTURE_STDOUT)
#define PIPED_BUFFER_SIZE   1024
#endif

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
#if defined(CAPTURE_STDOUT)
    char    *expected_stdout;
    int     stdout_orig;
    int     pipe_filedes[2];
    char    captured_stdout[PIPED_BUFFER_SIZE + 1];
#endif

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

#if defined(CAPTURE_STDOUT)
    expected_stdout = "t, z, kf_a.x, kf_b.x\n"
                      "0.000000, 0.000000, 0.000000, 0.000000\n"
                      "1.000000, 1.000000, 0.600004, 0.600400\n";

    stdout_orig = dup(fileno(stdout));
    pipe(pipe_filedes);
    dup2(pipe_filedes[1], fileno(stdout));
#endif

    returnValue = app_exec(2, argv);  /* 0 = success */

#if defined(CAPTURE_STDOUT)
    fflush(stdout);
    write(pipe_filedes[1], 0, 1); /* write ending '\0' */
    close(pipe_filedes[1]);
    dup2(stdout_orig, fileno(stdout));
#endif

    remove(filename);

    ck_assert_int_eq(returnValue, 0);

#if defined(CAPTURE_STDOUT)
    read(pipe_filedes[0], captured_stdout, PIPED_BUFFER_SIZE);
    captured_stdout[PIPED_BUFFER_SIZE] = '\0';
    ck_assert_str_eq(captured_stdout, expected_stdout);
#endif
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
