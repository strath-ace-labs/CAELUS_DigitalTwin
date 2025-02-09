FROM python:3.7

RUN apt-get update -y
RUN apt-get update && \
apt-get install -y --no-install-recommends \
        openjdk-11-jre

ENV ANT_VERSION=1.10.3
ENV ANT_HOME=/opt/ant
ENV IN_DOCKER=1
COPY . /CAELUS_DigitalTwin
# change to tmp folder
WORKDIR /tmp

# Download and extract apache ant to opt folder
RUN wget --no-check-certificate --no-cookies http://archive.apache.org/dist/ant/binaries/apache-ant-${ANT_VERSION}-bin.tar.gz \
    && wget --no-check-certificate --no-cookies http://archive.apache.org/dist/ant/binaries/apache-ant-${ANT_VERSION}-bin.tar.gz.sha512 \
    && echo "$(cat apache-ant-${ANT_VERSION}-bin.tar.gz.sha512) apache-ant-${ANT_VERSION}-bin.tar.gz" | sha512sum -c \
    && tar -zvxf apache-ant-${ANT_VERSION}-bin.tar.gz -C /opt/ \
    && ln -s /opt/apache-ant-${ANT_VERSION} /opt/ant \
    && rm -f apache-ant-${ANT_VERSION}-bin.tar.gz \
    && rm -f apache-ant-${ANT_VERSION}-bin.tar.gz.sha512

# add executables to path
RUN update-alternatives --install "/usr/bin/ant" "ant" "/opt/ant/bin/ant" 1 && \
    update-alternatives --set "ant" "/opt/ant/bin/ant" 

RUN apt-get update && apt-get install -y bash
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN apt-get update && apt-get -y install cmake protobuf-compiler
RUN apt-get install -y bc
  
WORKDIR /CAELUS_DigitalTwin
ENV PATH="/CAELUS_DigitalTwin/venv/bin:$PATH"
ENV PX4_ROOT_FOLDER="/CAELUS_DigitalTwin/Dependencies/PX4-Autopilot"
ENV PX4_SIM_SPEED_FACTOR=10
RUN bash install_all.sh

ENTRYPOINT ["python3", "-u", "start.py"]