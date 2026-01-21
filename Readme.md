This is a simple project that I've created just to solve a problem, and I used whatever I had at hand. 
It will only send Yaw Roll and Pitch, since there's no way to "know" the translation!

I'm no programmer so the code is as it is, but it works well for me. For orientation calculation I used sensor fusion algorithm developed by jremington. Big thanks for sharing the implementation of the Mahony filters for MPU-6050. For comunication with the MPU6050 I use FastIMU. Inside the box I have Seeed studio Xiao esp32-c3, mpu6050, small battery form old e-cig (suprised it still works), and 21x15mm Rocker Switch. If i wanted to make MK II of the box i would fix the switch cutout, and make the lid a litte longer, so it will acually snap together.

To configure it, download FastIMU lubrary from Arduino, set network name, network password, and target PC ip address. Make sure correct pins for MPU6050 are set and switch on device, let it sit flat for about 5 seconds, and open Opentrack with Input set to FreePIE UDP reciver. I've created drift compensation brackets that assume that most of the time you are facing the monitor, it works good for DCS. In my design i used pins 9 and 10 for sda and scl, since in ESP you can use i2c on any pins.

For soldering this device together you can create matrix by just printing bottom few milimiters of the box. I did this unintentionally as my print failed, but it created neat cable management.

There is no license to this, feel free to use it however you want.
