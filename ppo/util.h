//
// Created by Fanming Luo on 2019/1/12.
//

#ifndef CAR_SIMULATOR_UTIL_H
#define CAR_SIMULATOR_UTIL_H

#include <cmath>

namespace rs {
    namespace cs {
        template<typename T>
        T MyNorm(const T &t1, const T &t2) {
            return std::sqrt(t1 * t1 + t2 * t2);
        }

        template<typename T>
        T MyLimit(const T &x, const T &low, const T &high) {
            if (x < low) {
                return low;
            }
            if (x > high) {
                return high;
            }
            return x;
        }
    }
}

#endif //CAR_SIMULATOR_UTIL_H
