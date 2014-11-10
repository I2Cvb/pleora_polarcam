ira_photonfocus_driver
======================
ROS device driver for PhotonFocus cameras based on Pleora’s eBUS™ Software Development Kit (SDK)

Requirements
======================
- ROS Indigo (or Hydro)
- Pleora’s eBUS™ SDK 4.0.6 (download it for free from http://www.pleora.com/support-center/documentation-downloads)

ROS Package contents
======================
This ROS package contains:
- the ROS node which acts as driver for the camera;
- a utility for scanning the ethernet interface for connected cameras
- a utility for changing the IP of a specified camera (the ip changes temporarly, camera boot sets it to default)

More Info
======================
Supported cameras         http://www.photonfocus.com/
Pleora’s eBUS™ (SDK)      http://www.pleora.com/our-products/ebus-sdk

This ROS driver has been tested with Photonfocus AG MV1-D1312-40-GB-12 cameras and both ROS Indigo (Ubuntu 14.04)
and ROS Hydro (Ubuntu 13.10).
