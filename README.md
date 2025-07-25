# Keelson Connector SBG SDK 

Tools for logging SGB units using the sbgECom [GITHUB LINK](https://github.com/SBG-Systems/sbgECom)

OBS! This connector is for reading realtime data (6 decimals), if need accurate data the sbgBasicLogger (9 decimals) should be used directly.

```bash
sbgBasicLogger -s /dev/ttyUSB0 -r 115200 -p --time-mode=utcIso8601 --status-format=decimal -w 
```

## Quick start

```bash

# UDP
socat -u UDP4-RECV:2033,reuseaddr STDOUT | bin/main --log-level 10 -r rise -e storakrabban --publish raw --publish imu --publish pos 

sbgBasicLogger -s /dev/ttyUSB0 -r 115200 -p 
```

## Record Data with Keelson MCAP

```bash

sudo docker run --rm --network host  --name mcap-logger --volume ~/rec:/rec ghcr.io/rise-maritime/keelson:0.3.7-pre.51 "keelson_processor_ais --output_path rec -k rise/v0/landkrabban/**"

```



## SBG Ellipse N

SBG device and streaming all the sensor data:

✅ euler - Euler angles (roll/pitch/yaw)
✅ quat - Quaternion orientation
✅ nav - Navigation data with GPS coordinates
✅ airData - Air pressure data
✅ imuData - IMU sensor data (accelerometer/gyroscope)
✅ mag - Magnetometer data
✅ shipMotion - Ship motion data
✅ gnss1Vel - GNSS velocity data
✅ gnss1Pos - GNSS position data
✅ status - System status
✅ utcTime - UTC timestamps







Setup for development environment on your own computer: ´

1) Install [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/)
   - Docker desktop will provide you with an UI ´´for monitoring and controlling docker containers and images along debugging 
   - If you want to learn more about docker and its building blocks of images and containers checkout [Docker quick hands-on in guide](https://docs.docker.com/guides/get-started/)
2) Start up of **Zenoh router** either in your computer or any other computer within your local network 

   ```bash
    # Navigate to folder containing docker-compose.zenoh-router.yml
  
    # Start router with log output 
    docker-compose -f containing docker-compose.zenoh-router.yml up 

    # If no obvious errors, stop container "ctrl-c"

    # Start container and let it run in the background/detached (append -d) 
    docker-compose -f containing docker-compose.zenoh-router.yml up -d
   ```

    [Link to --> docker-compose.zenoh-router.yml](docker-compose.zenoh-router.yml)

1) Now the Zenoh router is hopefully running in the background and should be available on localhost:8000. This can be example tested with [Zenoh Rest API ](https://zenoh.io/docs/apis/rest/) or continue to next step running Python API
2) Set up python virtual environment  `python >= 3.11`
   1) Install package `pip install -r requirements.txt`
3)  Now you are ready to explore some example scripts in the [exploration folder](./exploration/) 
    1)  Sample are coming from:
         -   [Zenoh Python API ](https://zenoh-python.readthedocs.io/en/0.10.1-rc/#quick-start-examples)


[Zenoh CLI for debugging and problem solving](https://github.com/RISE-Maritime/zenoh-cli)




UDP 1830 



## Start UDP AIS stream on port 1830

Step 1: Set up configuration file

- Make a folder for Keelson connectors and processors:
  - Copy a docker compose file [docker-compose.ais-processor.yml](./docker-compose.ais-processor.yml) OR create a file with name "docker-compose.ais-processor.yml" and add content bellow 

```yml
services:

  keelson-processor-ais:
    image: ghcr.io/rise-maritime/keelson-processor-ais:latest
    container_name: keelson-processor-ais
    restart: unless-stopped
    network_mode: "host"
    command: "--log-level 10 --publish udp_sjv --subscribe sjofartsverket"
```

Step 2: Set up docker service configuration

- Open a terminal and navigate to location where you created the file
- Run following start up command in a terminal to start the service
  
  ```bash
  docker compose -f docker-compose.ais-processor.yml up -d
  ```

- Docker images will be downloaded and service started 

Step 3: Open Docker Desktop

- Check if you can find the new service in docker desktop
- From now on you can start and stop the service from docker desktop (You do not need to do the step 2 again)  
