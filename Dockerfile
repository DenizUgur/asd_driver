FROM osrf/ros:melodic-desktop-full-bionic
LABEL version="1.0"
LABEL description="ASD Driver Image"

RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y lsb-core g++ openssh-server gedit vim \ 
    apt-transport-https software-properties-common build-essential

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys E298A3A825C0D65DFD57CBB651716619E084DAB9

RUN add-apt-repository -y 'deb https://cloud.r-project.org/bin/linux/ubuntu bionic-cran35/'

RUN apt-get update && apt-get install -y r-base libgeos-dev

RUN echo "install.packages(\"gdistance\", repos=\"https://cran.rstudio.com\")" | R --no-save

RUN echo "install.packages(\"rgeos\", repos=\"https://cran.rstudio.com\")" | R --no-save

RUN apt-get install -y ros-melodic-grid-map ros-melodic-rtabmap-ros \
    python3 python3-pip python3-yaml libpcl-dev python-catkin-tools

RUN pip3 install matplotlib numpy rpy2 rospkg catkin_pkg

RUN mkdir -p /catkin_ws/src && cd /catkin_ws/src \ 
    && git clone https://github.com/ANYbotics/kindr.git \
    && git clone https://github.com/ANYbotics/kindr_ros.git \
    && git clone https://github.com/anybotics/elevation_mapping.git \
    && git clone https://github.com/ros/catkin.git \
    && mkdir asd_driver

RUN apt-get -qy autoremove

RUN rosdep install --from-paths /catkin_ws/src/ --ignore-src --rosdistro melodic -r -y

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin build -DCMAKE_BUILD_TYPE=Release'

ADD . /catkin_ws/src/asd_driver

RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd /catkin_ws; catkin build -DCMAKE_BUILD_TYPE=Release'

RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc