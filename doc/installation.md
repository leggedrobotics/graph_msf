# Installation

## Requirements

#### Eigen3
Make sure you have the Eigen3-headers in your include path. This is usually automatically the case if you have ROS installed.

#### GTSAM
We install GTSAM 4.1.1 from source, but also other versions and pre-built binaries might work. We recommend a local installation of GTSAM.

* Version: [GTSAM 4.1.1](https://github.com/borglab/gtsam/tree/4.1.1)
* Clone and checkout at an arbitrary location.

```bash
git clone git@github.com:borglab/gtsam.git
cd gtsam
git checkout 4.1.1
mkdir build && cd build
```

* Use CMake with the following options. Note that _-DCMAKE_INSTALL_PREFIX:PATH=$HOME/.local_ specifies the location where GTSAM is installed to and can be individually adapted. This flag can also be removed, leading to a global installation.

```bash
cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME/.local -DCMAKE_BUILD_TYPE=Release -DGTSAM_POSE3_EXPMAP=ON -DGTSAM_ROT3_EXPMAP=ON -DGTSAM_USE_QUATERNIONS=ON -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF ..
``` 

* Compile and install locally:

```bash
make install -j$(nproc)
```

* Environment variables (e.g. add to your .bashrc-file):

```
export CMAKE_PREFIX_PATH=$HOME/.local/:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$HOME/.local/lib/:$LD_LIBRARY_PATH
export LIBRARY_PATH=${LIBRARY_PATH}:${LD_LIBRARY_PATH}
```
This is usually only needed if you you choose a non-standard (local) install directory.

## graph_msf
### Workspace
GraphMsf only has two main dependencies: Eigen3 and GTSAM. These were installed in the last step.
For compiling graph_msf create a workspace and set it to _Release_-mode.
```bash
mkdir catkin_ws
mkdir src
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src && git clone https://github.com/leggedrobotics/graph_msf.git
```

### Compiling
Now the library then can be compiled using:
```bash
catkin build graph_msf
```

## graph_msf_ros
In contrast to _graph_msf_ this package also depends on standard ROS dependencies. We tested the framework with ROS Noetic on Ubuntu 20.04.
For compiling the code run:
```bash
catkin build graph_msf_ros
```
