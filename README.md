ira_photonfocus_driver
======================
ROS device driver for PhotonFocus cameras based on Pleora’s eBUS™ Software Development Kit (SDK)

Requirements
======================
- Ubuntu 14.04 (or 12.04) x86_64
- ROS Indigo (or Hydro)

ROS Package contents
======================
This ROS package contains:
- the ROS node which acts as driver for the camera;
- a utility for scanning the ethernet interface for connected cameras
- a utility for changing the IP of a specified camera (the ip changes temporarly, camera boot sets it to default)

FAQ
======================
- Does it work on 32bit architecture?
In order to make it work on 32bit architectures it should be sufficient to update the two symlink libudev.so.0 and libexpat.so.0 inside the folder ebus_sdk/lib and make them point to the 32bit version of the respective libraries. This can be done with the following commands within the folder of the package:

for 32bit architecture:
sudo ln -s /lib/i386-linux-gnu/libudev.so.1 ebus_sdk/lib/libudev.so.0
sudo ln –s /lib/i386-linux-gnu/libexpat.so.1 ebus_sdk/lib/libexpat.so.0

for 64bit architecture [default]:
sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1 ebus_sdk/lib/libudev.so.0
sudo ln –s /lib/x86_64-linux-gnu/libexpat.so.1 ebus_sdk/lib/libexpat.so.0

More Info
======================
Supported cameras         http://www.photonfocus.com/
Pleora’s eBUS™ (SDK)      http://www.pleora.com/our-products/ebus-sdk

This ROS driver uses the Pleora’s eBUS™ SDK 4.0.6 for connecting and communicating with the camera devices.

It has been tested with Photonfocus AG MV1-D1312-40-GB-12 cameras and both ROS Indigo (Ubuntu 14.04)
and ROS Hydro (Ubuntu 13.10).
