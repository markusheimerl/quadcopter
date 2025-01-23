# quad
A quadcopter printed circuit board

![image](https://raw.githubusercontent.com/markusheimerl/quad/3f04bb0489eefd0a6364144f415133f1339fff0e/output_image/drone-motors.png)
![image](https://raw.githubusercontent.com/markusheimerl/quad/3f04bb0489eefd0a6364144f415133f1339fff0e/output_image/drone-imu.png)
![image](https://raw.githubusercontent.com/markusheimerl/quad/3f04bb0489eefd0a6364144f415133f1339fff0e/output_image/drone-power.png)


``` sh
sudo add-apt-repository ppa:kicad/kicad-8.0-releases
sudo apt update
sudo apt install kicad imagemagick
kicad-cli sch export svg drone.kicad_sch -o output_image
convert output_image/drone-imu.svg output_image/drone-imu.jpg
```
