//
// Created by Fanming Luo on 2018/10/10.
//

#ifndef RSCAR_RS_H
#define RSCAR_RS_H

#include <string>
#include <iostream>

namespace rs{
    namespace common {
        class rs {
        public:
            explicit rs(const std::string &_name);
            virtual ~rs() = default;
            virtual void Run();
            std::string GetModuleName();
        private:
            std::string name;
        };
    }
}


#endif //RSCAR_RS_H
