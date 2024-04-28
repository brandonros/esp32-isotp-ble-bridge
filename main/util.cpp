#include <mutex>
#include <Arduino.h>
#include "util.h"

void wait_for_lock(std::mutex &mtx) {
    for (;;) {
        if (mtx.try_lock()) {
            return;
        } else {
            delay(1);
        }
    }
}
