#include "unity.h"
#include "unity_test_runner.h"

// The FreeRTOS mock stops esp_system from providing main(), so the test app provides its own
int main(int argc, char** argv)
{
    unity_run_all_tests();
    return Unity.TestFailures > 0 ? 1 : 0;
}
