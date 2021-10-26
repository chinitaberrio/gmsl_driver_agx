# sekonix_camera

Sekonix GMSL Camera Drive for NVIDIA Drive AGX Xavier Developer Kit for DRIVE OS Linux 5.2.0 and DriveWorks 3.5

This is a modified version of https://gitlab.com/leo-drive/Drivers/sekonix_camera

## Dependencies

 * https://github.com/facebook/folly (v2019.12.30.00)
 
## Function

This driver creates a worker thread for each cameras, utilizes a producer consumer queue for each,
compresses them to jpeg with hardware acceleration and publishes the images with hardware timestamps of AGX.

In DriveWorks for some reason port a = a, port b = c, port c = e and port d = g. That's the convention used in this driver. 