<div align="center">
<img width="200px"  src="images/image5.png"/>
<img width="200px"  src="images/image1.jpg"/>

# Morgan State University’s School of Engineering
## MSU RACECAR V1

<img width="200px"  src="images/image4.png"/>
<img width="200px"  src="images/image7.png"/>
<img width="200px"  src="images/image6.png"/>

# Table of contents
</div>

1. [Introduction](#intro)
2. [The Build Construction](#paragraph1)
   1. Bill of Materials
   2. Computing Modules (?)
   3. Camera(s)
   4. Wi-Fi
   5. Assembly Hardware
3. Hardware Setup
   1. 3D Printed Parts
   2. Wiring
4. The Software Setup
   1. ROS
   2. Arduino
   3. Bluetooth
5. Startup
   1. Manual
   2. Autonomous

## Introduction <a name="intro"></a>
Morgan State University is attempting to delve into the world of Artificial Intelligence. This project is meant to act as a jumping off point for a future endeavors and build a platform that students can learn from and build upon.

<img width="200px"  src="images/image1.jpg"/>

## The Build Construction <a name="intro"></a>

### Bill of Materials <a name="intro"></a>

This section lists the parts needed to build the MSU RACECAR. After you have received the parts and confirmed them, proceed to the hardware setup. Some of the parts are 3D printed. In these cases, the STL files have been made available along with recommended print settings.

```Disclaimer: Most of the URLs are from the OEM. You can probably find it cheaper elsewhere, e.g. Amazon.```

### Computing Modules (?) <a name="intro"></a>
||||||
|---|---|---|---|---|
|Part | Quantity | Cost | URL | Note |
|Jetson Nano|1|$99.00||The 2GB version is also |
|Arduino Micro|1|$32.00||Generic Version will work also|
|HC-06 Bluetooth Module|1|$32.00||Generic Version will work also|

### Camera(s) <a name="intro"></a>
||||||
|---|---|---|---|---|
|Part | Quantity | Cost | URL | Note |
|INTEL D435|1|$179.00|https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d435.html?cid=sem&source=sa360&campid=2021_q1_egi_us_ntgrs_nach_revs_text-link_brand_bmm_desk_realsense-shopping-ad_o-2fj5v_google&ad_group=RealSense+Shopping+Ads&intel_term=PRODUCT_GROUP&sa360id=92700050119513696&gclid=CjwKCAiAjeSABhAPEiwAqfxURflhmEFdvpKcQLn3MmvM4kGXbQuXrnXNFoWgEWSZqakCRxl8dq7sDRoCzKAQAvD_BwE&gclsrc=aw.ds||
|INTEL T265|1|$199.00|https://store.intelrealsense.com/buy-intel-realsense-tracking-camera-t265.html?cid=sem&source=sa360&campid=2021_q1_egi_us_ntgrs_nach_revs_text-link_brand_bmm_desk_realsense-shopping-ad_o-2fj5v_google&ad_group=RealSense+Shopping+Ads&intel_term=PRODUCT_GROUP&sa360id=92700050119513705&gclid=CjwKCAiAjeSABhAPEiwAqfxURTS6hqR8Fg53Ss-XBovh-1NOrRv66u4cPTb46CTJMK7m4en5JObQYhoCp7oQAvD_BwE&gclsrc=aw.ds||

### Wi-Fi <a name="intro"></a>
||||||
|---|---|---|---|---|
|Part | Quantity | Cost | URL | Note |
|TL-WN722N|1|$16.99|||
|USB Hub|1||||
### Assembly Hardware + Misc <a name="intro"></a>
||||||
|---|---|---|---|---|
|Part | Quantity | Cost | URL | Note |
|Chassis|1|$275|| Purchasing a different chassis may make some of the 3-D printed parts unusable|
|M3 x 6 Machine Screws|4||||
|20 x 40 Perfboard|1|$3|||
|LM2596 DC-DC Buck Converter Step Down Module|1|14.95|https://www.amazon.com/LM2596-Converter-Module-Supply-1-23V-30V/dp/B008BHBEE0|The actual individual part is much cheaper but is often sold only in packs of 5-10|
## Hardware Setup
### 3D Printed Parts 
```All Parts where 3D printed on an Ender 3```
||||||
|---|---|---|---|---|
| Image(s) | Purpose | Supports Needed | Layer Height | Printspeed |
|<img width="200px"  src="images/image3.jpg"/><img width="200px"  src="images/image11.jpg"/>|Acts as a mount for the 2 3D Sensing Cameras|No|.28mm| 150mm/s|
|<img width="200px"  src="images/image8.jpg"/>|Screws into the rear underside of the Jetson nano, while the front side rests on a piece of foam on top of the ESC|Yes|.28mm| 150mm/s|
|<img width="200px"  src="images/image9.jpg"/><img width="200px"  src="images/image10.jpg"/>|Screws into the body of the ESC and attaches to the sides of the arduinos perf board|Yes|.28mm| 150mm/s|
|<img width="200px"  src="images/image3.jpg"/>|Acts to extend the head of a Nmm bolt ```A small hole must be drilled in the frame for the bolt to pass through```|Yes|.28mm| 150mm/s|
### Wiring

<img width="200px"  src="images/image12.jpg" alt="Wiring Diagram"/>

In order to simplify powering the nano I chose to pull power from the RC battery to a LM2596 as shown above. 

<img width="200px"  src="images/image12.jpg" alt="Wiring Diagram"/>

## Software Setup
### ROS
Follow the basic instructions to install Ubuntu 16.04. This requires an at least 16 GB SD card so have one on hand before starting. 
Once that process is complete, finish by installing ROS kinetic with the provided  instructions.

``` Only the base version is necessary but if you would like to use the GUI tools in my experience they will run fine.```

### Arduino
After which you should clone the git repo into the root of your device. Running the ```./setup.sh``` installation script in the GitHub will install additional dependencies for ROS. Run ```ls /dev/tty*``` to find which port your arduino is attached on. cd into [Moonbear/src/roboguide/arduino/]() and run ```ino build; ino upload -p ${port}```

### Bluetooth
Just install the Joy Bluetooth Commander from the play store and pair to the HC-06 device within the app

## Startup
For either mode it's necessary to power the computer since there is no on or off switch manually plugging and unplugging the RC battery cable is the best way to go about turning the system on and off.

```Though this didn't raise issues during development you should shut down the nano separately via SSH first```
### Manual 
Turning on the setup without connecting *** will startup the car in bluetooth mode 
### Autonomous 
In a new terminal run ```roslaunch rc.launch``` to run the ROS control 
