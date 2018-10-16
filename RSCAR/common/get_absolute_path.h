/*================================================================
*   Copyright (C) 2018 * Ltd. All rights reserved.
*   
*   File name   : GetAbsolutePath.h
*   Author      : FanmingL
*   Created date: 2018-10-10 19:03:31
*   Description : 
*
*===============================================================*/


#ifndef _GET_ABSOLUTE_PATH_H
#define _GET_ABSOLUTE_PATH_H

#include <string>

namespace rs {
    namespace common {
        std::string GetAbsolutePath(const std::string &relative_path_) {
            std::string absolute_path(PATH);
            if (relative_path_[0] != '/')
                absolute_path += '/';

            return (absolute_path + relative_path_);
        }
    }
}

#endif //GET_ABSOLUTE_PATH_H
