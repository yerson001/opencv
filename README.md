# opencv_cpp_basics
OpenCV tutorials with C++ for beginners

To build using CMake, in ~/opencv_cpp_basics/build:

$ cmake ..

$ make


# instala dependencias 

~~~
 sudo apt-get install build-essential
 sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
 sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev  libdc1394-22-dev
 sudo apt-get install libxvidcore-dev libx264-dev libgtk-3-dev
 sudo apt-get install libtbb2 libtbb-dev libdc1394-22-dev
 sudo apt-get install libv4l-dev v4l-utils
 sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
 sudo apt-get install libavresample-dev libvorbis-dev libxine2-dev
 sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
 sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
 sudo apt-get install libopenblas-dev libatlas-base-dev libblas-dev
 sudo apt-get install liblapack-dev libeigen3-dev gfortran
 sudo apt-get install libhdf5-dev protobuf-compiler
 sudo apt-get install libprotobuf-dev libgoogle-glog-dev libgflags-dev
 sudo apt install default-jre
 
# a symlink to videodev.h
 cd /usr/include/linux
 sudo ln -s -f ../libv4l1-videodev.h videodev.h
 cd ~
~~~
## Descargar
~~~
$ sudo -s

$ cd /opt
$ opt/ git clone https://github.com/opencv/opencv.git
$ opt/ git clone https://github.com/opencv/opencv_contrib.git
~~~
### EN OPENCV --> elegi versión
~~~
$/opt/ cd opencv
$ git chekcout 4.4.0  # cualquier version que quieras de opencv
$/opt/opencv$ mkdir release

$/opt/opencv$ cd release

$/opt/opencv/release$ cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules /opt/opencv
~~~
### Error linea 11 -> solo si lo tiene
~~~
"FATAL: In-source builds are not allowed.
You should create separate directory for build files."
~~~
~~~
mkdir my_build_dir
cd my_build_dir
rm ../CMakeCache.txt
~~~
## Compilar
~~~
/opt/opencv/release$ make -j4

/opt/opencv/release$ make install

/opt/opencv/release$ ldconfig

/opt/opencv/release$ exit

/opt/opencv/release$ cd ~
~~~
## opencv VIZ

~~~
–D WITH_VTK=ON ../
~~~

~~~
check yerson001/myslam/README.md

~~~


