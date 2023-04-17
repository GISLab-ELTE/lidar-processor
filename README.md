# Online LiDAR Processor (OLP)

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
olp --file datafile.pcap
```
To be able to read from file, the *PCL* dependency must be compiled with PCAP support (see [How to compile](#how-to-compile)).

### Skip a certain number of clouds
It is possible to omit the first *N* cloud frames with the `--offset <N>` option.

### Localization and mapping
The program currently supports the dynamic localization of acquired point clouds with GNSS data or through SLAM algorithms.

#### Localization with GNSS sensor (offline)
For further details and examples, see the guide on [Localization and mapping with GNSS sensor](doc/LOCALIZATION_GNSS.md).

#### Localization with GNSS sensor (online)
By connecting a GNSS receiver to the Velodyne sensor, the oroduced PCAP packages will contain GNSS data as well.
```bash
olp --file datafilewithgnss.pcap --withgps
```
By default, only GNSS packets with the pulse-per-second (PPS) signal set will be used. In case you did not manage to synchronize PPS between GNSS and the LiDAR sensor, this restriction can be lifted with the `--nopcappps` option.

If a GNSS packet is not available at the time of processing a cloud frame, one will be extrapolated using the previous ones. To replace the default extrapolation method with an experimental, Kalman filter-based one, use the `--usepcapekf` switch.

#### Localization with SLAM algorithms
An ICP (*Iterative closest point*) and a LOAM (*LiDAR Odometry and Mapping*) algorithm is supported for now, with the ICP producing a better result.
```bash
olp --file datafile.pcap --slamtype icp
```

#### Localization with ICP aided by IMU sensor data
The ICP approach has the option to be aided by the data from an IMU (*Inertial Measurement Unit*) sensor, improving the outputted results.
```bash
olp --file datafile.pcap --slamtype icp --imucsv imudata.csv
```

## How to compile

The project uses some third-party dependencies:
 1. Any standard C++ compiler supporting language version C++14 or newer (e.g. GCC, Clang, MSVC).
 2. [CMake](https://cmake.org/) is used as the build system of the project. Version 2.8 or newer is required.
 3. [Boost](https://www.boost.org/) is used as an extension to the C++ standard library. (Dependency of PCL anyway.) Version 1.60 or newer required.
 4. [PCL](http://pointclouds.org/) is heavily used in point cloud processing and I/O management. Version 1.8 or newer is required.
 5. [PCAP](https://en.wikipedia.org/wiki/Pcap) *(optional)* to read and "replay" previously recorded PCAP files of a Velodyne sensor.
 6. [LASlib](https://github.com/LAStools/LAStools/tree/master/LASlib/) *(optional)*, to export point clouds in LAS format.
 7. [libpointmatcher](https://github.com/ethz-asl/libpointmatcher) *(optional)* for the ICP SLAM algorithm.

The complete compilation process depends on the system you are using (Linux, Mac OS X or Windows). See the platform-specific compilation guides below.

### Platform-specific compilation guides

 * [Ubuntu](doc/INSTALL_UBUNTU.md)
 * [MacOS](doc/INSTALL_MACOS.md)
 * [Windows](doc/INSTALL_WINDOWS.md) *(no SLAM support!)*
 * [Windows with WSL](doc/INSTALL_WINDOWS_WSL.md)

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on coding conventions.

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE) file for details.
