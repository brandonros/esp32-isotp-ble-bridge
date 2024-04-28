#ifndef __UTIL_
#define __UTIL__

#include <mutex>

void wait_for_lock(std::mutex &mtx);

#endif
