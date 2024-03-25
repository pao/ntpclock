#ifndef _ntpclock_include_udp_h_
#define _ntpclock_include_udp_h_

#include <cstdint>

#include "lwip/sockets.h"

uint64_t micros64() {
    return esp_timer_get_time();
}

#endif