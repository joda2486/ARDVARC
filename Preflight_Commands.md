## Communication Checks
- [ ] Connect Micro-HDMI to Ground Station and Monitor
- [ ] Attach USB keyboard to Ground Station
- [ ] Attach WiFi Card to Ground Station (Use ALFA cable)
- [ ] Plug Ground Pi into ethernet wall
- [ ] Ground Station Power - On (Use USB-C PD power brick)
- [ ] Go OUT SIDE
- [ ] Mission Computer and Flight Controller Power- On
- [ ] Login to Ground Station (Username: ```pi```, Password: ```ardvarc420```)
- [ ] Run command on Ground Station: (give it some time)
 ```
 wfb-cli gs
``` 
- [ ] Verify Data Link tunnel is active
- [ ] Connect Flight Controller to QGroundControl through USB
- [ ] Start Mavlink through NuttShell:
```
mavlink start -o 14540 -t 192.168.0.10
```
- [ ] Disconnect Flight Controller USB
- [ ] Verify Data Link mavlink is active
- [ ] Connect Ground Station to Laptop (MacOS or Linux) with Ethernet
- [ ] On laptop run ifconfig and note ethernet interface name (usually starts with e)
- [ ] Disconnect the Personal Computer from any Wifi
- [ ] Set Laptop static IP:
```
sudo ifconfig [ethernet interface name] 169.254.251.115
```
- [ ] Set Ground Station static IP:
```
sudo ifconfig eth0 169.254.251.113
```
- [ ] Verify QGroundControl on Laptop
