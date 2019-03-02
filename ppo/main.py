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

import ctypes
from tensorboardX import SummaryWriter
from model import env
import time
if_need_load = False
load_index = 59
log_path = "log"


if __name__ == '__main__':
	my_env = env('./ppopy.so')
	writer = SummaryWriter(log_path)
	if if_need_load:
		my_env.load_model(load_index)
	for i in range(1000000000):
		actor_loss, critic_loss, average_return = my_env.train_model()

		print('%9d, %12.8f, %12.8f, %12.8f' % (i, actor_loss, critic_loss, average_return))
		writer.add_scalar('score', average_return, i)
		writer.add_scalar('actor_loss', actor_loss, i)
		writer.add_scalar('critic_loss', critic_loss, i)

		if i % 500 == 0:
			my_env.save_model(i // 500)



