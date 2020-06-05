# Tutorial: How to compile on Windows

This guide shows how to compile the *Online LiDAR Processor* on Windows with Microsoft Visual C++.  
Tested on Windows 10 with Visual Studio 2017.

## Install the dependencies

For Windows operating systems binary releases are available for the dependencies:
 * [CMake](https://cmake.org/), see the [download page](https://cmake.org/download/).
 * [Boost Library](https://www.boost.org/), check for prebuilt Windows binaries on the [download page](https://www.boost.org/users/download/).
 * [PCL](http://pointclouds.org/), the most recent binary installers can be found the [GitHub project page](https://github.com/PointCloudLibrary/pcl/releases).
 Install all dependencies unless already present on your computer!

### PCAP support

In order to support reading PCAP files, PCL must be compiled with the **WITH_PCAP** flag. Unfortunately the official prebuilt binaries are built without it. If reading PCAP files is a required functionality, you must:
 1. Install a PCAP library (e.g. [Npcap](https://nmap.org/npcap/))
 2. Compile PCL manually with the *WITH_PCAP* flag, for which you should check the [official guide](http://pointclouds.org/documentation/tutorials/compiling_pcl_windows.php).  
 Note, that version 1.8 or higher is required for PCAP integration.

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
make -f Makefile_Windows
```

*Note:* rerun when tools change in the vendor directory.

## Compile the program
```bash
mkdir build && cd build
cmake .. -DCMAKE_GENERATOR_PLATFORM=[x86|x64]
msbuild ALL_BUILD.vcxproj /p:Configuration=Release
```
Pay attention to set the value of the `CMAKE_GENERATOR_PLATFORM` option correctly, matching the architecture of the installed PCL, otherwise compilation will fail.

### Typical issues
 * **Issue:** the Boost library is not found upon CMake configuration.  
 **Solution:** in case you use a separate Boost installation instead of PCL's default one, you might need to configure its location through the `BOOST_INCLUDEDIR` variable.

 * **Issue:** the OpenNI2 library is not found upon execution of the compiled binary.  
 **Solution:** the `<path>\<to>\<openni2>\Redist` folder must be in your `PATH` variable.
