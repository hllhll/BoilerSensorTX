# docker:
# docker run --rm --privileged --network host -v /dev/ttyS0:/dev/ttyS0 boilsersensorrx:latest

# docker-compose:
#   boilersensorrx:
#     build:
#       context: ./BoilserSensorRX/  # Or folder name of this solution relative to your docker-compose
#       dockerfile: ./Dockerfile
#     network_mode: host
#     depends_on:
#       - mqtt
#     volumes:
#       - /dev/ttyS0:/dev/ttyS0
#     privileged: true


FROM python:latest

RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir pyyaml paho-mqtt pyserial crc8 RPi.GPIO

WORKDIR /usr/app/src
COPY ./ha_mqtt_device.py ./
COPY ./*.py ./
CMD [ "python", "./reciver.py"]
