#!/bin/bash

set -e

echo "[1/7]"
sudo apt-get update
sudo apt-get install -y \
    python3-pip python3-setuptools \
    cmake g++ libpython3-dev python3-numpy swig python3-matplotlib \
    libxml2 libxml2-dev bison flex libcdk5-dev \
    libusb-1.0-0-dev libaio-dev pkg-config \
    libavahi-common-dev libavahi-client-dev \
    libsdl2-dev libgl1-mesa-dev libglew-dev libfftw3-dev

echo "[2/7]"
git clone --branch soapy-sdr-0.8.1 https://github.com/TelecomDep/SoapySDR.git
cd SoapySDR
mkdir -p build && cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
cd ../..

echo "[3/7]"
git clone --branch v0.24 https://github.com/TelecomDep/libiio.git
cd libiio
mkdir -p build && cd build
cmake ../
make -j$(nproc)
sudo make install
cd ../..

echo "[4/7]"
git clone --branch v0.3 https://github.com/TelecomDep/libad9361-iio.git
cd libad9361-iio
mkdir -p build && cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
cd ../..

echo "[5/7]"
git clone --branch sdr_gadget_timestamping https://github.com/TelecomDep/SoapyPlutoSDR.git
cd SoapyPlutoSDR
mkdir -p build && cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
cd ../..

echo "[6/7]"
sudo ldconfig

echo "[7/7]"