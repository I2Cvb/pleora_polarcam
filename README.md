# pleora_polarcam

ROS device driver and vision and image processing toolbox for imprex polar-cameras based on Pleora’s eBUS™ Software Development Kit (SDK)

The driver was originally cloned from IRALab [ira_photonfocus_driver](
https://github.com/iralabdisco/ira_photonfocus_driver.git) repository. 

## Management of the repo 
The current version of the repo contains three branches: 

* master 
* 8_bts_process
* 10_bts_process

You are on the branch ** 8_bts_process **

Each user should fork the project or clone (if they have the admin permision) and in their user space create extra branches if they like.
In case of merging the new created branch with any of the existing branches, the users can request a PR (pull request). 

## Running the program

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

#### Showing images 
* ##### angle  
	
```
	rosrun pleora_polarcam pix2image_node 'angle'
```
or 

```
	rosrun pleora_polarcam process_angles_node 

```

* ##### stokes

``` 
	rosrun pleora_polarcam pix2image_node 'stokes'

``` 
or

``` 
	rosrun pleora_polarcam process_stokes_node

```

* ##### polarized parameters (DoP, AoP and S0) 

```
	rosrun pleora_polarcam pix2image_node 'polar'

```
or poa

```
	rosrun pleora_polarcam process_polar_node 

```
#### Saving images
```
	rosrun pleora_polarcam pix2image_node  'angles/polar/stokes' save

```

### Using `roslaunch`

First the ip address of the camera in the `pleora_polarcam_driver.launch` file should be adjusted to the found ip of the camera
Second remeber ther is no need for roscore 

#### Starting the driver 
```
	roslaunch pleora_polarcam pleora_polarcam_driver.launch

```

#### Showing only 

* ##### angle images
```
	roslaunch pleora_polarcam show_angle_images.launch

``` 
* ##### stokes images
```
	roslaunch pleora_polarcam show_stokes_images.launch

``` 
* ##### polarized parameters
```
	roslaunch pleora_polarcam show_polar_images.launch

``` 


#### Showing and Saving the images

* ##### angles 
```
	roslaunch pleora_polarcam save_angle_images.launch

``` 
* ##### stokes 
```
	roslaunch pleora_polarcam save_stokes_images.launch

``` 
* ##### polarized parameters
```
	roslaunch pleora_polarcam save_polar_images.launch

``` 