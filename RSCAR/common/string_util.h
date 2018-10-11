//
// Created by erdou on 18-10-11.
//

#ifndef RSCAR_STRING_UTIL_H
#define RSCAR_STRING_UTIL_H

#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
namespace rs{
    namespace common{
        template<typename T>
        std::string to_string_with_precision(const T a_value, const int n = 5) {
            std::ostringstream out;
            out << std::setprecision(n) << a_value;
            return out.str();
        }
    }
}


#endif //RSCAR_STRING_UTIL_H
