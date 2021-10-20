# Localization and mapping with GNSS sensor

The program supports the dynamic localization of acquired point clouds with GNSS data.  
Both the point cloud and the GNSS measurements must be pre-recorded and saved to files.

## How to prepare files

For proper using the program you need the following files:

1. *PCAP* file: containing LiDAR data.
2. *CSV* file: containing GNSS data. Supported sensors:
 - Stonex S9 III Plus sensor: the measurements can be exported in a proprietary *RW5* format, which can be [converted](http://www.carlsonemea.com/cwa/report/index.php) to a *CSV* file. Use the following configuration:
    * Generate CSV file: comma separated
    * Select coordinate order: X/Y Y=EAST
    * Delete first row from generated *CSV* file
 - GNSS/GPS sensors in mobile phones: the recommended application for Android is [GPS Logger](https://play.google.com/store/apps/details?id=eu.basicairdata.graziano.gpslogger). With this application, the GNSS log can be exported in a *TXT* file. It is really a *CSV* file in fact, so:
    * replace the `.txt` extension with `.csv`
    * replace `,` with `;`
    * replace `.` with `,`

## How to run

Proper parameterization of the inputs is required to run the program.
Use the `--help` or `-h` option for a list of possible parameters.

The following parameters could be given:

|    Option    | Mandatory | Description |
| ------------ | --------- | ----------- |
| `--file <path>`     |    YES    | *PCAP* file path. |
| `--csvfile <path>`  |    YES*   | *CSV* file path for Stonex sensor data. |
| `--mcsvfile <path>` |    YES*   | *CSV* file path for mobile GNSS sensor data. |
| `--stime <time>`    |           | The UNIX epoch time of the first frame in the *PCAP* file. |
| `--filter`   |           | Apply an *origo filter*, measured LiDAR point closer to 1 meter or further than 16 meters from the sensor are cropped out. |
| `--wftype <pcd \| las>`   |           | The output file format (`pcd` and `las` are supported.) Defaults to `las`. |


\* You may either specify `--csvfile` or `--mcsvfile`, or both of them.
