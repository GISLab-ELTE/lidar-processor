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

### Localization and mapping
The program currently supports the dynamic localization of acquired point clouds with GNSS data. For further details and examples, see the guide on [Localization and mapping](LOCALIZATION.md).

## How to compile

The project uses some third-party dependencies:
 1. Any standard C++ compiler supporting language version C++14 or newer (e.g. GCC, Clang, MSVC).
 2. [CMake](https://cmake.org/) is used as the build system of the project. Version 2.8 or newer is required.
 3. [Boost](https://www.boost.org/) is used as an extension to the C++ standard library. (Dependency of PCL anyway.) Version 1.60 or newer required.
 4. [PCL](http://pointclouds.org/) is heavily used in point cloud processing and I/O management. Version 1.8 or newer is required.
 5. [LASlib](https://github.com/LAStools/LAStools/tree/master/LASlib/) *(optional)*, to export point clouds in LAS format.
 6. [PCAP](https://en.wikipedia.org/wiki/Pcap) *(optional)*, to read and "replay" previously recorded PCAP files of a Velodyne sensor.

The complete compilation process depends on the system you are using (Linux, Mac OS X or Windows). See the platform-specific compilation guides below.

### Platform-specific compilation guides

 * [Ubuntu](INSTALL_UBUNTU.md)
 * [MacOS](INSTALL_MACOS.md)
 * [Windows](INSTALL_WINDOWS.md)

## Contributing

Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on coding conventions.

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE) file for details.
