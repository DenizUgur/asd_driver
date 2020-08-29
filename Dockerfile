FROM osrf/ros:melodic-desktop-bionic
LABEL version="1.0"
LABEL description="ASD Driver Image"

# Initialization
RUN apt-get update && \
    apt-get install -y --no-install-recommends openssh-server \ 
    apt-transport-https software-properties-common && \
    rm -rf /var/lib/apt/lists/*

# Add R
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys E298A3A825C0D65DFD57CBB651716619E084DAB9 && \
    add-apt-repository -y 'deb https://cloud.r-project.org/bin/linux/ubuntu bionic-cran35/' && \
    apt-get update && apt-get install -y --no-install-recommends r-base libgeos-dev && rm -rf /var/lib/apt/lists/* && \
    echo "install.packages(\"gdistance\", repos=\"https://cran.rstudio.com\")" | R --no-save && \
    echo "install.packages(\"rgeos\", repos=\"https://cran.rstudio.com\")" | R --no-save

# Add ROS related packages
RUN apt-get update && apt-get install -y --no-install-recommends ros-melodic-grid-map ros-melodic-rtabmap-ros \
    python3 python3-pip python3-yaml python3-setuptools libpcl-dev python-catkin-tools && \
    pip3 install matplotlib numpy rpy2 rospkg catkin_pkg && \
    apt-get -qy autoremove && rm -rf /var/lib/apt/lists/*

# Downlaod necessary repositories
RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src \ 
    && git clone https://github.com/ANYbotics/kindr.git \
    && git clone https://github.com/ANYbotics/kindr_ros.git \
    && git clone https://github.com/anybotics/elevation_mapping.git \
    && git clone https://github.com/ros/catkin.git \
    && mkdir asd_driver

RUN apt-get update && rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update && \
    rosdep install --from-paths /catkin_ws/src/ --ignore-src --rosdistro melodic -r -y && \
    rm -rf /var/lib/apt/lists/*

ADD . /catkin_ws/src/asd_driver

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin build -DCMAKE_BUILD_TYPE=Release'

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc