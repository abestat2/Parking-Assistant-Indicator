#include <cstdio>
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils.h"
#include "sdkconfig.h"

// The RMT driver allocates some resources lazily, so allow a small amount of heap growth per test
#define TEST_MEMORY_LEAK_THRESHOLD (400)

extern "C" void setUp(void)
{
    unity_utils_record_free_mem();
}

extern "C" void tearDown(void)
{
    unity_utils_evaluate_leaks_direct(TEST_MEMORY_LEAK_THRESHOLD);
}

extern "C" void app_main(void)
{
    printf("MB1013 target tests\n");
#if CONFIG_MB1013_TEST_QEMU
    // QEMU has no RMT peripheral, so only the decoding tests can run there.
    // run_qemu_tests.ps1 watches for the summary line below and then stops QEMU.
    UNITY_BEGIN();
    unity_run_tests_by_tag("[decode]", false);
    int failures = UNITY_END();
    printf("MB1013_QEMU_DONE failures=%d\n", failures);
#else
    unity_run_menu();
#endif
}
