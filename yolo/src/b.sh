#! /bin/bash

#================================================================
#   Copyright (C) 2018 * Ltd. All rights reserved.
#   
#   File name   : b.sh
#   Author      : FanmingL
#   Created date: 2018-09-23 07:28:41
#   Description : 
#
#================================================================

g++ -o main *.cpp -L/root/darknet -ldarknet  `pkg-config --cflags --libs opencv` -std=c++11 -DPATH=\"/root/yolo\" -lpthread
