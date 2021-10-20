# Guide: How to compile on macOS

This guide shows how to compile the *Online LiDAR Processor* on macOs with HomeBrew.  
Tested on macOS Catalina (10.15.1).

## Install the dependencies

First, download and install *HomeBrew* if you do not have it already.
```bash
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```

In order to support reading PCAP files, PCL must be compiled with the **WITH_PCAP** flag. Unfortunately the official prebuilt libraries are built without it.

### Without PCAP support

If reading PCAP files is not a required functionality (you would like to work directly with the Velodyne sensor), all dependencies can be installed from the HomeBrew package repository.
```bash
brew install cmake boost libpcl proj
```

### With PCAP support

Install the dependencies (except PCL) from package repository:
```bash
# Install build tools
brew install cmake
# Install mandatory dependencies for PCL
brew install boost eigen flann vtk proj libpcap
# Install optional dependencies for PCL
brew install qhull
```

Download and compile PCL with PCAP support (version 1.8+ required for PCAP integration):
```bash
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_PCAP=YES ..
make -j4
sudo make install
```

## Submodules

The project depends on some locally built tools:
* [LASlib](https://github.com/LAStools/LAStools/tree/master/LASlib)
* [libpointmatcher](https://github.com/ethz-asl/libpointmatcher)
* [libnabo](https://github.com/ethz-asl/libnabo)

Check out the repository, then initialize the submodules to download the locally built dependencies:
```bash
git submodule init
git submodule update
```

Compile the dependencies:
```bash
cd vendor
make
```

*Note:* rerun when tools change in the vendor directory.

## Compile the program
```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=<install_path>
make
make install
```
