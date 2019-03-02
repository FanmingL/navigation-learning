CC = g++
GPU = 0
DEBUG = 0
PROTO = protoc
TORCH_PATH = /Users/erdou/Documents/libtorch
TORCH_INCLUDE = $(TORCH_PATH)/include/torch/csrc/api/include 
RM = rm -rf
INSTALL_PATH = /usr/local/lib/
CPPFLAGS = -I. -I$(OBJ_PATH) -I/usr/include
CPPFLAGS += `pkg-config --cflags opencv`
CPPFLAGS += `pkg-config --cflags protobuf`
CPPFLAGS += -std=c++11
CPPFLAGS += -D_GLIBCXX_USE_CXX11_ABI=0
CPPFLAGS += -I$(TORCH_INCLUDE) -I$(TORCH_PATH)/include
CPPFLAGS += -O2
CPPFLAGS += -fPIC
ifeq ($(DEBUG), 1)
CPPFLAGS += -g
endif
LDFLAGS = `pkg-config --libs opencv`
LDFLAGS += `pkg-config --libs protobuf`

TORCH_LDFLAGS = $(LDFLAGS)
TORCH_LDFLAGS += -L$(TORCH_PATH)/lib
ifeq ($(GPU), 1)
TORCH_LDFLAGS += -ltorch -lc10 -lnnpack -lonnx -lpthreadpool  -lbenchmark -lcaffe2 -lclog -lcpuinfo -lmkldnn -lc10_cuda -lgloo_cuda -lnnpack -lnvrtc-builtins -lnvrtc -lc10d -lc10d_cuda_test -lcaffe2_gpu -lTHD -lnvrtc-builtins -lcaffe2_module_test_dynamic -lcaffe2_protos -lclog -lgloo -lgloo_builder -lgloo_cuda -lgtest -lgtest_main -lnnpack -lnnpack_reference_layers -lonnx -lonnxifi_dummy -lonnxifi_loader -lonnxifi -lonnx_proto -lbenchmark -lbenchmark_main -lc10_cuda -lnvToolsExt
TORCH_LDFLAGS += -L/usr/local/cuda/lib64 -lcuda -lcudart -lcublas -lcurand -lcudnn
TORCH_LDFLAGS += -ltorch -lc10 -lcaffe2 -lmkldnn -lgomp -lcaffe2_gpu -lnvToolsExt
TORCH_LDFLAGS +=-L/usr/local/cuda/lib64 -lcudnn -lcudart -lcublas -lcurand -lcusolver -lnvrtc
TORCH_LDFLAGS +=-L/usr/local/cuda/lib64/stubs/ -lcuda
else
TORCH_LDFLAGS += -ltorch -lc10 -lnnpack -lonnx -lpthreadpool  -lbenchmark -lcaffe2 -lclog -lcpuinfo -lmkldnn 
TORCH_LDFLAGS += -ltorch -lc10 -lcaffe2 -lmkldnn 
endif
COMMON_OBJS = car_config.o car_simulator.o 

PPO_OBJS = ppo.o $(COMMON_OBJS)
SIMULATOR_OBJS = main.o $(COMMON_OBJS)
PPO_EXEC_OBJS = ppo_main.o $(PPO_OBJS) 
SIMULATOR_EXEC = simulator
PPO_EXEC = ppo

PPO_LIB = ppo.so
PYTHON_LIB = ppopy.so
PYTHON_OBJS = $(PPO_OBJS) py_interface.o
CP = cp

all: $(SIMULATOR_EXEC) $(PPO_EXEC) $(PPO_LIB) $(PYTHON_LIB)


$(SIMULATOR_EXEC) : $(SIMULATOR_OBJS) 
	$(CC) $^ -o $@ $(LDFLAGS)

$(PPO_EXEC) : $(PPO_EXEC_OBJS)
	$(CC) $^ -o $@ $(TORCH_LDFLAGS)

$(PPO_LIB) : $(PPO_OBJS) 
	$(CC) -fPIC -shared $^ -o $@ $(TORCH_LDFLAGS)

$(PYTHON_LIB) : $(PYTHON_OBJS) 
	$(CC) -fPIC -shared $^ -o $@ $(TORCH_LDFLAGS)

%.o : %.cpp
	$(CC) -c $< -o $@ $(CPPFLAGS)

%.po : %.pb.cc
	$(CC) -c $< -o $@ $(CPPFLAGS)
.PHONY:clean
clean:
	$(RM) $(PPO_OBJS) $(PYTHON_OBJS) $(SIMULATOR_EXEC) $(PPO_EXEC) $(PPO_LIB) $(PYTHON_LIB) $(SIMULATOR_OBJS) $(PPO_EXEC_OBJS)
.PHONY:install
install:
	cp $(PPO_LIB) $(INSTALL_PATH)



