sudo apt-get install -y python-catkin-tools \
                    python-future \
                    ros-kinetic-image-geometry \
                    ros-kinetic-pcl-ros \
                    ros-kinetic-rqt-gui \
                    ros-kinetic-rqt-gui-py \
                    ros-kinetic-mav-msgs \
                    ros-kinetic-nmea-msgs \
                    ros-kinetic-image-transport \
                    ros-kinetic-cv-bridge \
                    ros-kinetic-mavros-msgs \
                    libyaml-cpp-dev \
                    libusb-1.0-0-dev \
                    libavcodec-dev \
                    libswresample-dev \
                    libopencv-dev \
                    liblapack-dev \
                    libblas-dev \
                    libboost-dev \
                    libarmadillo-dev \
                    libserial-dev \
                    ros-kinetic-pcl-ros \
                    ros-kinetic-vision-msgs \
                    ros-kinetic-cv-bridge \
                    ros-kinetic-image-transport \
                    ros-kinetic-tf2-geometry-msgs \
                    build-essential \
                    pkg-config \
                    python-dev \
                    python-opencv \
                    libopencv-dev \
                    libav-tools  \
                    libjpeg-dev \
                    libpng-dev \
                    libtiff-dev \
                    libjasper-dev \
                    libgtk2.0-dev \
                    python-numpy \
                    python-pycurl \
                    libatlas-base-dev \
                    gfortran \
                    python-pip \
                    python-scipy \
                    usbutils \
                    lsb-release \
                    libtool \
                    m4 \
                    automake \
                    ros-kinetic-rosserial \
                    ros-kinetic-rosserial-python

sudo pip install --upgrade pip
sudo pip install numpy imutils scipy

# install opencv 3.4.8 and contrib
cd ~ && git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout 3.4.8 && \
    cd ~ && git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && \
    git checkout 3.4.8 && \
    cd ~/opencv && mkdir -p build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D BUILD_EXAMPLES=OFF .. && \
    make -j4 && \
    sudo make install && \
    sudo ldconfig

# glogging for rotors
cd ~
wget https://github.com/google/glog/archive/v0.4.0.tar.gz
tar zxvf v0.4.0.tar.gz
cd glog-0.4.0
libtoolize
./autogen.sh
./configure
make
sudo make install

# install eigen
cd ~
git clone https://github.com/eigenteam/eigen-git-mirror.git
cd eigen-git-mirror && git checkout 3.3.7
mkdir build && cd build
cmake ..
make
sudo make install

# install librealsense
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u

sudo apt-get install -y librealsense2-dkms
sudo apt-get install -y librealsense2-utils
sudo apt-get install -y librealsense2-dev
sudo apt-get install -y librealsense2-dbg

cd ~/catkin_ws/

# pull all the things
git submodule update --init --recursive

# clean and build
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src -r -y
catkin clean --yes
catkin build -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=False

# source setup.bash file
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc;
