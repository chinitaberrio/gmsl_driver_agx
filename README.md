# sekonix_camera

Sekonix GMSL Camera Drive for NVIDIA Drive AGX Xavier Developer Kit for DRIVE Software 10.0

This is a heavily modified version of https://gitlab.com/autowarefoundation/autoware.ai/drivers/-/tree/master/autoware_driveworks_gmsl_interface

If I removed some disclaimers from the top of some source files, I'm sorry, it's not to steal credit, it's to make files look cleaner.

Whoever wrote what is not hidden from anyone with the internet history.

## Dependencies

 * https://github.com/facebook/folly (v2019.12.30.00)
 
## Function

This driver creates a worker thread for each port(4 cameras), utilizes a producer consumer queue for each,
compresses them to jpeg with hardware acceleration and publishes the images with hardware timestamps of AGX.

We tried with 8 cameras (3+3+2+2 for ports) and it does its best.

I think this driver squeezes out of AGX the most to get compressed jpeg images from it currently.

The only better way to get more from the cameras in AGX would be some sort of h264/h265 compression but that would be video then.