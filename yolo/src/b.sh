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

g++ -o main *.cpp /usr/local/lib/libdarknet.so  `pkg-config --cflags --libs opencv` -lpthread -std=c++14 -DPATH=\"/root/yolo\" -DGPU -DCUDNN -DOPENCV -I/usr/local/cuda/include

