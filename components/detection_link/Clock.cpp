#include "detection_link/Transport.hpp"
#include <chrono>

namespace detection_link {

int64_t steady_clock_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

} // namespace detection_link
