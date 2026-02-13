## Установка зависимостей

**Скрипт**
```bash
chmod +x
sudo ./script.sh
```
---

**SoapySDR**
```bash
sudo apt-get install python3-pip python3-setuptools
sudo apt-get install cmake g++ libpython3-dev python3-numpy swig python3-matplotlib

git clone --branch soapy-sdr-0.8.1 https://github.com/TelecomDep/SoapySDR.git

cd SoapySDR
mkdir build && cd build

cmake ../

make -j`nproc` # nproc - количество потоков, например make -j16 
sudo make install
sudo ldconfig
```

**Libiio**

```bash
sudo apt-get install libxml2 libxml2-dev bison flex libcdk5-dev cmake
sudo apt-get install libusb-1.0-0-dev libaio-dev pkg-config 
sudo apt install libavahi-common-dev libavahi-client-dev


git clone --branch v0.24 https://github.com/TelecomDep/libiio.git

cd libiio
mkdir build && cd build
cmake ../
make -j`nproc` # nproc - количество потоков, например make -j16 
sudo make install
```

**LibAD9361**

```bash
git clone --branch v0.3 https://github.com/TelecomDep/libad9361-iio.git
cd libad9361-iio

mkdir build && cd build

cmake ../

make -j`nproc` # nproc - количество потоков, например make -j16 
sudo make install
sudo ldconfig
```

**SoapyPlutoSDR**

```bash
git clone --branch sdr_gadget_timestamping https://github.com/TelecomDep/SoapyPlutoSDR.git
cd SoapyPlutoSDR

mkdir build && cd build

cmake ../

make -j`nproc` # nproc - количество потоков, например make -j16 
sudo make install
sudo ldconfig
```

**SDL, glew, fftw**
```bash
sudo apt install libsdl2-dev libgl1-mesa-dev libglew-dev libfftw3-dev
```

## Запуск

В папке dev
```bash
mkdir build && cd build
cmake ..
make -j`nproc`

./main.elf # RX или Loppback rx tx

./tx_main.elf # TX
```