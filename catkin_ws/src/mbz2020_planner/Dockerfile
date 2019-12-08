FROM ros:kinetic-robot

ENV IS_DOCKER_BUILD true

RUN apt-get update 
RUN apt-get install -y python-catkin-tools \
                       wget \
                       libtool \
                       m4 \
                       automake \
                       pkg-config \
                       libyaml-cpp-dev \
                       ros-kinetic-mavros-msgs \
                       ros-kinetic-vision-msgs \
                       libeigen3-dev \ 
                       ros-kinetic-cv-bridge \ 
                       ros-kinetic-orocos-kdl \ 
                       ros-kinetic-tf2-geometry-msgs 

COPY . /mbz2020_planner

# install glogging
WORKDIR ~/ 
RUN bash -c "wget https://github.com/google/glog/archive/v0.4.0.tar.gz && \
             tar zxvf v0.4.0.tar.gz && \
             cd glog-0.4.0 && \
             libtoolize && \
             ./autogen.sh && \
             ./configure && \
             make && \
             make install"

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    wget \
    cpio \
    sudo \
    curl \
    git \
    vim \
    cmake \
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
    lsb-release && \
    rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip

RUN pip install \
    numpy \
    imutils

RUN cd ~ && git clone https://github.com/Itseez/opencv.git && \
    cd opencv && \
    git checkout 2.4.13.5 && \
    cd ~ && git clone https://github.com/Itseez/opencv_contrib.git && \
    cd opencv_contrib && \
    git checkout 2.4.13.5 && \
    cd ~/opencv && mkdir -p build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D BUILD_EXAMPLES=OFF .. && \
    make -j4 && \
    make install && \
    ldconfig


WORKDIR /mbz2020_planner

RUN bash -c "source /opt/ros/kinetic/setup.bash && \
             catkin build -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release && \
             echo \"source /opt/ros/kinetic/setup.bash\" >> ~/.bashrc && \
             echo \"source /mbz2020_planner/devel/setup.bash\" >> ~/.bashrc"
