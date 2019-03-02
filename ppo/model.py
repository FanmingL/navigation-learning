#! /usr/local/bin/python3
# coding=utf-8
#================================================================
#   Copyright (C) 2019 * Ltd. All rights reserved.
#   
#   File name   : model.py
#   Author      : FanmingL
#   Created date: 2019-01-17 11:07:51
#   Description : 
#
#================================================================
import ctypes
class env:
	def __init__(self, path):
		self.so = ctypes.cdll.LoadLibrary(path)
		self.load = self.so.Load
		self.save = self.so.Save
		self.inference = self.so.Inference
		self.train = self.so.Run

		self.inference.argtypes = [ctypes.POINTER(ctypes.c_float)]
		self.inference.restype = None

		self.load.argtypes = [ctypes.c_int]
		self.load.restype = None

		self.save.argtypes = [ctypes.c_int]
		self.save.restype = None

		self.train.argtypes = [ctypes.POINTER(ctypes.c_float), 
		ctypes.POINTER(ctypes.c_float),
		ctypes.POINTER(ctypes.c_float)]
		self.train.restype = None

	def load_model(self, index):
		index = ctypes.c_int(int(index))
		self.load(index)

	def save_model(self, index):
		index = ctypes.c_int(int(index))
		self.save(index)

	def run_model(self):
		average_return = ctypes.c_float()
		self.inference(ctypes.pointer(average_return))
		return average_return.value

	def train_model(self):
		actor_loss = ctypes.c_float()
		critic_loss = ctypes.c_float()
		average_return = ctypes.c_float()
		self.train(ctypes.pointer(actor_loss), 
					ctypes.pointer(critic_loss),
					ctypes.pointer(average_return))
		return actor_loss.value, critic_loss.value, average_return.value

