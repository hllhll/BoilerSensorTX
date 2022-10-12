# docker run --rm --privileged --network host -v /dev/ttyS0:/dev/ttyS0 boilsersensorrx:latest
FROM python:latest

RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir pyyaml paho-mqtt pyserial

WORKDIR /usr/app/src
COPY ./ha_mqtt_device.py ./
COPY ./reciver.py ./
CMD [ "python", "./reciver.py"]
