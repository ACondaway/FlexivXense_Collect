# Flexiv Robot Device
First connect ethernet via `General` port with workstation.
Download [integration package](https://fcs.flexiv.com/downloadService/downloadIntegrationPackage).
Install Flexiv Elements by `setup.sh`, and run by `run.sh`.
Update firmware via Flexiv Elements, password as `flexiv`. You need to install `sudo apt install pip3` and `pip3 install lxml,pathlib`.
Install RDK license.
Set robot `User 1` port as `192.168.2.100` and `255.255.255.0`, and workstation net as `192.168.2.101` and `255.255.255.0`.
Connect ethernet via `User 1` port with workstation.
Open Flexiv Elements, enable remote mode in `Settings -> Remote Mode -> Ethernet`.
Install RDK by `pip install flexivrdk`. Check [version compatibility](https://www.flexiv.com/software/rdk/manual/robot_software_compatibility.html).

## Coordinate
* base: (x=out, y=right, z=up)
* tcp: (x=right, y=out, z=down)
