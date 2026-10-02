#include <cstdlib>
#include "unity.h"
#include "unity_test_runner.h"

// FreeRTOS is not mocked here, so the linux target starts app_main in a POSIX-simulated task
extern "C" void app_main(void)
{
    unity_run_all_tests();
    std::exit(Unity.TestFailures > 0 ? 1 : 0);
}
