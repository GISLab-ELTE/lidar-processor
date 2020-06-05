# Online LiDAR Processor

On-the-fly LiDAR processor and visualizer for Velodyne sensors.

## How to use

### Read data from sensor
Read point data directly from a Velodyne sensor on the fly:
```bash
olp --ip 192.168.1.201 --port 2368
```
The `--ip` and `--port` program flags are optional, the default values are shown above.

### Read data from file
Read point data from a saved PCAP file:
```bash
olp.exe --file datafile.pcap
```
To be able to read from file, the *Online LiDAR Processor* must be compiled with PCAP support.

## How to compile

The project uses some third-party dependencies:
 1. Any standard C++ compiler supporting language version C++14 or newer (e.g. GCC, Clang, MSVC).
 2. [CMake](https://cmake.org/) is used as the build system of the project. Version 2.8 or newer is required.
 3. [PCL](http://pointclouds.org/) is heavily used in point cloud processing and I/O management. Version 1.8 or newer is required.
 4. [PCAP](https://en.wikipedia.org/wiki/Pcap) *(optional)*, to read and "replay" previously recorded PCAP files of a Velodyne sensor.

### Tutorials

 * [How to compile on Ubuntu](INSTALL_UBUNTU.md)
 * [How to compile on Windows](INSTALL_WINDOWS.md)
