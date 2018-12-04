# RSCAR
## Get Started 
### Software Requirements

- CMake 3.1
- OpenCV 3.0.0 or higher
- Eigen3
- Google Protocol Buffers 2.6
- darknet

### Install OpenCV
If You had not installed OpenCV, please follow [the installing and configuring OpenCV tutorial](https://blog.csdn.net/u013066730/article/details/79411767).

### Install Dependencies

**Ubuntu**

```bash
sudo apt-get install cmake protobuf-compiler libprotobuf-dev libeigen3-dev 
```

**MacOS**

```bash
brew install cmake protobuf eigen
```

### Download RSCAR
Download the RSCAR project to `/path/to/RSCAR`.

```bash
cd /path/to/RSCAR
git clone https://....git
```

### Download darknet

```bash
cd third_party
bash darknet.sh
cd darknet
```
change the some configure component in `Makefile`, like `GPU=1`,`CUDNN=1`,`OPENCV=1`.

```bash
make
sudo cp libdarknet.* /usr/local/lib
sudo cp darknet.h /usr/local/include
sudo ldconfig
```

### Install RSCAR
```bash
cd /path/to/RSCAR
mkdir build
cd build
cmake ..
make -j4
```
then, there are several executable file in `bin` directory. In fact, each module in `module` directory will produce an executable file. In the next part we will introduce the function of these executable files.


## The modules in module directory
### preprocess
Pre-process the video, like resizing the video to a proper size or selecting the ROI.
### stabling
Stabling the video based on SIFT features and RANSAC algorithnm.
### detect
Detect the car or persong using YOLO or background-foreground separation.
### track
Track the objects based on KCF and Kalman Fileter algorithms.
### filter_tracker
Smooth the trajectories.
### relocate
Transfer the pixel coordinate to world coordinate based on Homography matrix.
### mannul_mark
This part, if you lose all hope of the tracking and detecting algorithm, you can mark your own data set manually.
### altest
Some algorithms in testing, like model the background based on gaussian mixture model.











