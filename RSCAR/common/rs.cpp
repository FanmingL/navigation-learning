//
// Created by Fanming Luo on 2018/10/10.
//

#include "common/rs.h"

namespace rs {
    namespace common {
        rs::rs(const std::string &_name) : name(_name) {

        }

        void rs::Run() {
            std::cout << "rs base method" << std::endl;
        }

        std::string rs::GetModuleName() {
            return name;
        }
    }
}


