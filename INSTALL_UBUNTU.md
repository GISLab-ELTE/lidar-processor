# Guide: How to compile on Ubuntu

This guide shows how to compile the *Online LiDAR Processor (OLP)* on Ubuntu with GCC.  
Tested on Ubuntu Bionic (18.04 LTS).

## Install the dependencies

In order to support reading PCAP files, PCL must be compiled with the **WITH_PCAP** flag. Unfortunately the official prebuilt libraries are built without it.

### Without PCAP support

If reading PCAP files is not a required functionality (you would like to work directly with the Velodyne sensor), all dependencies can be installed from the standard package repository.
```bash
sudo apt-get install build-essential cmake libboost-all-dev libpcl-dev libproj-dev
```

### With PCAP support

Install the dependencies (except PCL) from package repository:
```bash
# Install build tools
sudo apt-get install build-essential cmake
# Install the Boost library
sudo apt-get install libboost-all-dev
# Install mandatory dependencies for PCL
sudo apt-get install libboost-all-dev libeigen3-dev libflann-dev libvtk6-dev libvtk6-qt-dev \
 libproj-dev libpcap-dev
# Install optional dependencies for PCL
sudo apt-get install libqhull-dev libopenni-dev
```

Download and compile PCL with PCAP support (version 1.8+ required for PCAP integration):
```bash
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.9.1.tar.gz
tar -xvf pcl-1.9.1.tar.gz
cd pcl-pcl-1.9.1

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_PCAP=YES ..
make -j4
sudo make install
```

## Submodules

The project depends on some locally built tools:
* [LASlib](https://github.com/LAStools/LAStools/tree/master/LASlib)

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
