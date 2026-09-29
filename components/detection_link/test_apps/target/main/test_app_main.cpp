#include <cstdio>
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils.h"
#include "sdkconfig.h"

// Protocol and link tests allocate only inside each test, so hold them to a small allowance.
// The [hw] tests raise it: starting Wi-Fi for the first time keeps driver buffers allocated.
static constexpr size_t TEST_MEMORY_LEAK_THRESHOLD = 400;
size_t g_test_leak_threshold = TEST_MEMORY_LEAK_THRESHOLD;

extern "C" void setUp(void)
{
    g_test_leak_threshold = TEST_MEMORY_LEAK_THRESHOLD;
    unity_utils_record_free_mem();
}

extern "C" void tearDown(void)
{
    unity_utils_evaluate_leaks_direct(g_test_leak_threshold);
}

extern "C" void app_main(void)
{
    printf("Detection link target tests\n");
#if CONFIG_DETECTION_LINK_TEST_QEMU
    // QEMU has no Wi-Fi radio, so run everything except the [hw] tests.
    // run_qemu_tests.ps1 / .sh watch for the summary line below and then stop QEMU.
    UNITY_BEGIN();
    unity_run_tests_by_tag("[hw]", true);
    int failures = UNITY_END();
    printf("DETECTION_LINK_QEMU_DONE failures=%d\n", failures);
#else
    unity_run_menu();
#endif
}
