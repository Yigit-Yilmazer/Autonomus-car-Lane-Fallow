# Autonomus-car-Lane-Fallow

This code aim is lane track lanes using an Raspberry pi autonomus car

  
## Technologies:

Language:python

librarys:numpy,opencv,GPIO,time,Picamera

hardware:Raspberry pi5,arducam autofocus camera,servo,dc motors,motor driver,breadbord,powerbank and 5 AA batterys

## İnstallation and dependcies:

First,the opencv and numpy librarys need to be downloaded.
you can download with this command:

``` python
pip install opencv-python
pip install numpy
```
and you can install code with this link:
```
https://github.com/Yigit-Yilmazer/Autonomus-car-Lane-Fallow.git
```

## Usage:

This code was created to detect white lanes.If the lane being used is black,you can find your ideal code by experimenting with values in the code below.And you can change the parts that are written in white to black:

``` python
low_white = np.array([0, 0, 150]) #low limit
highest_white = np.array([179, 70, 255])#highest limit
```

You can adjust your robot stability by chancing the values in the KP section of the code,
and you can also adjust the KD section to suit your robot,allowing it to make cleaner turns
``` python
KP = 0.08  #vibration and stability setting
KD = 0.04  # reaction and clear turns
```



