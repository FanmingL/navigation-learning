#! /usr/local/bin/python3
# coding=utf-8
#================================================================
#   Copyright (C) 2019 * Ltd. All rights reserved.
#   
#   File name   : main.py
#   Author      : FanmingL
#   Created date: 2019-01-16 12:46:32
#   Description : 
#
#================================================================
import time
import ctypes
from model import env
load_index = 4
#load_index = 38
#from tensorboardX import SummaryWriter
#log_path = "logs"



if __name__ == '__main__':
	my_env = env('./ppopy.so')
	my_env.load_model(load_index)
	for i in range(3000):
		my_env.run_model()
		# print(i)
		# time.sleep(0.0333)
