#!/bin/bash

DIR=$PWD
echo "Installing Vulcan from"$DIR
echo "To install drivers for using the Vulcan wheelchair, set the DRIVERS environment variable:  DRIVERS=1 \
 $DIR/install.sh"

sudo apt-get install -y git scons doxygen texlive-latex-extra libboost-all-dev libarmadillo-dev libwxgtk3.0-gtk3-dev libwxgtk-media3.0-gtk3-dev libf2c2-dev openjdk-8-jdk libusb-dev libusb-1.0-0-dev libpopt-dev libsdl1.2-dev libsdl-net1.2-dev libopencv-dev cmake autoconf;

cd $DIR/external

# Install cereal for message passing serialization
wget -nc https://github.com/USCiLab/cereal/archive/v1.2.2.tar.gz -O cereal.tar.gz
tar -xzf cereal.tar.gz
rsync -a cereal-1.2.2/ cereal
rm -rf cereal-1.2.2/

# Install LCM
wget -nc https://github.com/lcm-proj/lcm/archive/v1.3.1.tar.gz -O lcm.tar.gz
tar -xzf lcm.tar.gz
cd lcm-1.3.1
./bootstrap.sh
./configure
make -j5
sudo make install
cd ..

# Install NLopt for running MPEPC
wget -nc https://github.com/stevengj/nlopt/archive/v2.5.0.tar.gz -O nlopt-2.5.0.tar.gz
tar -xzf nlopt-2.5.0.tar.gz
cd nlopt-2.5.0
./configure
make -j5
sudo make install
cd ..

# Install levmar for use with topological SLAM
# NOTE: levmar is GPL which makes binaries using it GPL
tar -xzf levmar-2.6.tgz
cd levmar-2.6
make
sudo cp liblevmar.a /usr/local/lib/
chmod a+r levmar.h
sudo cp levmar.h /usr/local/include/
cd ../

# Install libraries for place labeling
wget -nc https://github.com/cjlin1/liblinear/archive/v220.tar.gz -O liblinear.tar.gz
tar -xzf liblinear.tar.gz
rsync -a liblinear-220/ liblinear
rm -rf liblinear-220/

wget -nc https://github.com/cjlin1/libsvm/archive/v323.tar.gz -O libsvm.tar.gz
tar -xzf libsvm.tar.gz
rsync -a libsvm-323/ libsvm
rm -rf libsvm-323/

# Install gnuplot-iostream used internally for debugging
git clone https://github.com/dstahlke/gnuplot-iostream.git gpis_tmp
rsync -a gpis_tmp/ gnuplot-iostream
rm -rf gpis_tmp

######   Drivers for Vulcan wheelchair ######

echo "Drivers="$DRIVERS

if [ "$DRIVERS" = "1" ]; then
    # Install PEAK linux driver for talking to USB-CAN
    wget -nc https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.5.1.tar.gz
    tar -xzf peak-linux-driver-8.5.1.tar.gz
    cd peak-linux-driver-8.5.1
    make NET=NO_NETDEV_SUPPORT
    sudo make install
    sudo cp driver/pcan.h /usr/local/include/
    sudo cp lib/libpcan.h /usr/local/include/
    cd ..

    # Install libphidget for talking with the encoders
    # NOTE: libphidget is GPL so binaries including it will be GPL licensed
    wget -nc https://www.phidgets.com/downloads/phidget21/libraries/linux/libphidget/libphidget_2.1.8.20151217.tar.gz
    tar -xzf libphidget_2.1.8.20151217.tar.gz
    cd libphidget-2.1.8.20151217
    ./configure
    make -j5
    sudo make install
    cd ..

    # Install URG library for talking to Hokuyos
    tar -xzf urg-0.8.18.tar.gz
    cd urg-0.8.18
    ./configure LIBS=-lm
    make
    sudo make install
    sudo cp src/cpp/urg/ScipHandler.h /usr/local/include/urg/
    cd ..
fi

# Go back to root Vulcan directory and run the bootstrap for generating LCM messages
cd $DIR
./bootstrap.sh

echo "All dependencies are installed. You can build the software with 'scons -jN' where N is the number of parallel commands to execute"

