FROM ubuntu:20.04

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python3 python3-pip

ADD . /home/dasts
