# pleora_polarcam
======================
ROS device driver and vision and image processing toolbox for imprex polar-cameras based on Pleora’s eBUS™ Software Development Kit (SDK)

The driver was originally cloned from IRALab [ira_photonfocus_driver](
https://github.com/iralabdisco/ira_photonfocus_driver.git) repository. 

## Management of the repo 
=========================
You are on the branch ** 10_bts_process **

Each user should fork the project or clone (if they have the admin permision) and in their user space create extra branches if they like.
In case of merging the new created branch with any of the existing branches, the users can request a PR (pull request). 

## Running the program
========================
Assuming the pleora SDK and ROS requirements are installed, and the package is compiled with `catkin_make` to run the package use: 

### Using `rosrun`

#### Initialize a roscore
In a terminal 
```
	roscore 
```


#### Finding the ip of the camera and strating the driver
In another terminal
```
rosrun pleora_polarcam find_camera

``` 
This will return the ip of the camera to be used 

```
rosrun pleora_polarcam pleora_poalrcam_driver _ip:=THE_IP_FROM_FIND_CAMERA

```

#### Showing the 4 angle images acquired with the camera 

```
rosrun pleora_polarcam pix2image_node 'mat'

```
or 

```
rosrun pleora_polarcam process_angles_node 

```

#### Showing the 3 stokes images

``` 
rosrun pleora_polarcam pix2image_node 'stokes'

``` 
or

``` 
rosrun pleora_polarcam process_stokes_node

```

#### Showing the polarized parameters (DoP, AoP and S0) 

```
rosrun pleora_polarcam pix2image_node 'polar'

```
or 

```
rosrun pleora_polarcam process_polar_node 

```


### Using `roslaunch`

First the ip address of the camera in the `pleora_polarcam_driver.launch` file should be adjusted to the found ip of the camera
Second remeber ther is no need for roscore 

#### Starting the driver 
```
roslaunch pleora_polarcam pleora_polarcam_driver.launch

```

### Showing angle images
```
roslaunch pleora_polarcam pix2image_process_mat.launch

``` 
### Showing stokes images
```
roslaunch pleora_polarcam pix2image_process_stokes.launch

``` 
### Showing polarized parameters
```
roslaunch pleora_polarcam pix2image_process_polar.launch

``` 