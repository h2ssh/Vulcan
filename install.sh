#!/bin/bash

DIR=$PWD
echo "Installing Vulcan from"$DIR
echo "To install drivers for using the Vulcan wheelchair, set the DRIVERS environment variable:  DRIVERS=1 \
 $DIR/install.sh"

sudo apt install -y build-essential meson ninja-build ccache wget default-jdk liblcm-dev liblcm-java libjchart2d-java googletest doxygen graphviz texlive-latex-extra libboost-all-dev libarmadillo-dev libwxgtk3.0-gtk3-dev libwxgtk-media3.0-gtk3-dev libusb-dev libusb-1.0-0-dev libpopt-dev libsdl1.2-dev libsdl-net1.2-dev libopencv-dev libf2c2-dev cmake;

EXTERNAL_DIR=$DIR/external
LIB_DIR=$EXTERNAL_DIR/lib
INCLUDE_DIR=$EXTERNAL_DIR/include
PKG_CONFIG_DIR=$EXTERNAL_DIR/pkgconfig

cd $EXTERNAL_DIR

mkdir include
mkdir lib
mkdir pkgconfig

# Get JVM if not already there and clean up lcm java bits to run on current JVM
java --version || sudo apt install default-jdk
sudo sed -i 's/ -Xincgc / /g' `which lcm-spy` `which lcm-logplayer-gui`

# FIX for libarmadillo-dev not including a pkg-config (fixed upstream)
cp armadillo.pc $PKG_CONFIG_DIR

# Install cereal for message passing serialization
wget -nc https://github.com/USCiLab/cereal/archive/v1.2.1.tar.gz -O cereal-1.2.1.tar.gz
tar -xzf cereal-1.2.1.tar.gz
rsync -a cereal-1.2.1/ cereal
rm -rf cereal-1.2.1/
rsync -a cereal/include/ $INCLUDE_DIR

# Install NLopt for running MPEPC
wget -nc https://github.com/stevengj/nlopt/archive/v2.5.0.tar.gz -O nlopt-2.5.0.tar.gz
tar -xzf nlopt-2.5.0.tar.gz
cd nlopt-2.5.0
mkdir build
cd build
cmake ../ -DBUILD_SHARED_LIBS=0 -DCMAKE_INSTALL_PREFIX=$EXTERNAL_DIR
make -j5
make install
cd $EXTERNAL_DIR

# Install levmar for use with topological SLAM
# NOTE: levmar is GPL which makes binaries using it GPL
tar -xzf levmar-2.6.tgz
cd levmar-2.6
make
cp liblevmar.a $LIB_DIR
chmod a+r levmar.h
cp levmar.h $INCLUDE_DIR
cd $EXTERNAL_DIR/
# Prepend the prefix with the absolute path to the install directory
echo "prefix=$PKG_CONFIG_DIR" > $PKG_CONFIG_DIR/levmar.pc
cat levmar.pc >> $PKG_CONFIG_DIR/levmar.pc

# Install libraries for place labeling
wget -nc https://github.com/cjlin1/liblinear/archive/v220.tar.gz -O liblinear.tar.gz
tar -xzf liblinear.tar.gz
rsync -a liblinear-220/ liblinear
rm -rf liblinear-220/
mkdir $INCLUDE_DIR/liblinear
cp liblinear/linear.h $INCLUDE_DIR/liblinear

wget -nc https://github.com/cjlin1/libsvm/archive/v323.tar.gz -O libsvm.tar.gz
tar -xzf libsvm.tar.gz
rsync -a libsvm-323/ libsvm
rm -rf libsvm-323/
mkdir $INCLUDE_DIR/libsvm
cp libsvm/svm.h $INCLUDE_DIR/libsvm

# Install gnuplot-iostream used internally for debugging
git clone https://github.com/dstahlke/gnuplot-iostream.git gpis_tmp
rsync -a gpis_tmp/ gnuplot-iostream
rm -rf gpis_tmp
cp gnuplot-iostream/gnuplot-iostream.h $INCLUDE_DIR

######   Drivers for Vulcan wheelchair ######

echo "Drivers="$DRIVERS

if [ "$DRIVERS" = "1" ]; then
    # Install PEAK linux driver for talking to USB-CAN
    wget -nc https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.5.1.tar.gz
    tar -xzf peak-linux-driver-8.5.1.tar.gz
    cd peak-linux-driver-8.5.1
    make NET=NO_NETDEV_SUPPORT
    sudo make install
    cp driver/pcan.h $INCLUDE_DIR
    cp lib/libpcan.h $INCLUDE_DIR
    cd $EXTERNAL_DIR

    # Install libphidget for talking with the encoders
    # NOTE: libphidget is GPL so binaries including it will be GPL licensed
    wget -nc https://www.phidgets.com/downloads/phidget21/libraries/linux/libphidget/libphidget_2.1.8.20151217.tar.gz
    tar -xzf libphidget_2.1.8.20151217.tar.gz
    cd libphidget-2.1.8.20151217
    ./configure --prefix=$EXTERNAL_DIR
    make -j5
    make install
    cd $EXTERNAL_DIR

    # Install URG library for talking to Hokuyos
    tar -xzf urg-0.8.18.tar.gz
    cd urg-0.8.18
    ./configure LIBS=-lm --prefix=$EXTERNAL_DIR
    make
    make install
    cp src/cpp/urg/ScipHandler.h $INCLUDE_DIR/urg
    cd $EXTERNAL_DIR
    # Prepend the prefix with the absolute path to the install directory
    echo "prefix=$PKG_CONFIG_DIR" > $PKG_CONFIG_DIR/liburg.pc
    cat liburg.pc >> $PKG_CONFIG_DIR/liburg.pc
fi

# Go back to root Vulcan directory and run the bootstrap for generating LCM messages
cd $DIR
./bootstrap.sh

echo "All dependencies are installed. You can build the software with 'ninja -C build install' Binaries will be installed in the bin directory."
