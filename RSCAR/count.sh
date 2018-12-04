#! /bin/bash

#================================================================
#   Copyright (C) 2018 * Ltd. All rights reserved.
#   
#   File name   : count.sh
#   Author      : FanmingL
#   Created date: 2018-11-30 21:38:51
#   Description : 
#
#================================================================

( find . -name "*.cpp" && find . -name "*.h" && find . -name "*.prototxt" && find . -name "*.proto" && find . -name "*.txt"  )| xargs wc -l| sort -n
