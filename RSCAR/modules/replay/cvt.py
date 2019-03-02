#! /usr/local/bin/python3
# coding=utf-8
#================================================================
#   Copyright (C) 2018 * Ltd. All rights reserved.
#   
#   File name   : cvt.py
#   Author      : FanmingL
#   Created date: 2018-12-19 14:15:40
#   Description : 
#
#================================================================

import torch
file_index = 4
class Net(torch.nn.Module):  # 继承 torch 的 Module
    def __init__(self):
        super(Net, self).__init__()     # 继承 __init__ 功能

        self.main_net = torch.nn.Sequential(
            (torch.nn.Linear(980, 100)),
            # (torch.nn.Dropout(0.4)),
            (torch.nn.ReLU()),
            (torch.nn.Linear(100, 50)),
            # (torch.nn.Dropout(0.4)),
            (torch.nn.ReLU()),
            (torch.nn.Linear(50, 10))
            )  

        self.sub_net = torch.nn.Sequential(
            (torch.nn.Linear(3, 20)),
            # (torch.nn.Dropout(0.4)),
            (torch.nn.ReLU()),
            (torch.nn.Linear(20, 10))
            )

        self.subsub_net = torch.nn.Linear(20,2)


    def forward(self, x):   # 这同时也是 Module 中的 forward 功能
        # 正向传播输入值, 神经网络分析出输出值
        x1 = self.main_net(x[3:])
        
        x2 = self.sub_net(x[0:3])
    
        x3 = torch.cat((x1, x2), dim=0)

        x = self.subsub_net(x3)
        
        
        
        return x



if __name__ == '__main__':
    a = torch.load('/Users/erdou/Downloads/model'+str(file_index)+'.net')
    # a = Net() 
    exa = torch.rand(1, 983)
    exa = torch.squeeze(exa)
    tsm = torch.jit.trace(a, exa)
    print(tsm)
    tsm.save('/Users/erdou/Documents/GitHub/navigation-learning/RSCAR/data/model.pt')
