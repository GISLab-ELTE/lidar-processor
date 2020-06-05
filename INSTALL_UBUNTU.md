# Tutorial: How to compile on Ubuntu

This guide shows how to compile the *Online LiDAR Processor* on Ubuntu with GCC.  
Tested on Ubuntu Xenial (16.04 LTS) and Bionic (18.04 LTS).

## Install the dependencies

In order to support reading PCAP files, PCL must be compiled with the **WITH_PCAP** flag. Unfortunately the official prebuilt libraries are built without it.

### Without PCAP support

If reading PCAP files is not a required functionality (you would like to work directly with the Velodyne sensor), all dependencies can be installed from the standard package repository.
```bash
sudo apt-get install build-essential cmake libpcl-dev libproj-dev
```

### With PCAP support

Install the dependencies (except PCL) from package repository:
```bash
# Install build tools
sudo apt-get install build-essential cmake
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

## Compile the program
```bash
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=<install_path>
make
make install
```
