#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <check.h>

#include "app.h"

#if 1
#define CAPTURE_STDOUT
#endif

#if defined(CAPTURE_STDOUT)

#define MAX_PIPED_BUFFER_SIZE   1024

typedef struct stdout_capture_s
{
    int     stdout_orig;
    int     pipe_filedes[2];
    char    captured_stdout[MAX_PIPED_BUFFER_SIZE + 1];
} stdout_capture_t;

static int _stdout_capture_enable(stdout_capture_t *output_capture)
{
    int success = !0;

    output_capture->stdout_orig = dup(fileno(stdout));
    if ( pipe(output_capture->pipe_filedes) == 0 )
    {
        dup2(output_capture->pipe_filedes[1], fileno(stdout));
    }
    else
    {
        success = 0;
    }

    return success;
}

static int _stdout_capture_disable(stdout_capture_t *output_capture)
{
    int success = !0;

    fflush(stdout);
    close(output_capture->pipe_filedes[1]);
    dup2(output_capture->stdout_orig, fileno(stdout));

    return success;
}

static int _stdout_capture_retrieve(stdout_capture_t *output_capture, int size)
{
    int success = !0;

    if (size > MAX_PIPED_BUFFER_SIZE)
    {
        success = 0;
    }
    else
    {
        if ( read(output_capture->pipe_filedes[0], output_capture->captured_stdout, size) == 0 )
        {
            success = 0;
        }
        else
        {
            output_capture->captured_stdout[size] = '\0';
        }
    }

    return success;
}

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
    stdout_capture_t    output_capture;
    char                *expected_stdout;
    int                 expected_stdout_size;
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
    ck_assert( _stdout_capture_enable(&output_capture) );
#endif

    returnValue = app_exec(2, argv);  /* 0 = success */

#if defined(CAPTURE_STDOUT)
    ck_assert( _stdout_capture_disable(&output_capture) );
#endif

    remove(filename);

    ck_assert_int_eq(returnValue, 0);

#if defined(CAPTURE_STDOUT)
    expected_stdout = "t, z, kf_a.x, kf_b.x\n"
    "0.000000, 0.000000, 0.000000, 0.000000\n"
    "1.000000, 1.000000, 0.600004, 0.600400\n";
    expected_stdout_size = strlen(expected_stdout);
    if (expected_stdout_size > MAX_PIPED_BUFFER_SIZE)
    {
        expected_stdout_size = MAX_PIPED_BUFFER_SIZE;
    }

    ck_assert( _stdout_capture_retrieve(&output_capture, expected_stdout_size) );

    ck_assert_str_eq(output_capture.captured_stdout, expected_stdout);
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
