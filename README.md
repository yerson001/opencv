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
### ver ramas
git branch
### listar ramas
git branch -a

### descartar cambios locales
git checkout -- .
~~~
~~~
$/opt/ cd opencv
$ git checkout 4.4.0  # cualquier version que quieras de opencv
$/opt/opencv$ mkdir release

$/opt/opencv$ cd release

$/opt/opencv/release$ cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. -DOPENCV_GENERATE_PKGCONFIG=YES
~~~

~~~
cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules /opt/opencv
~~~


~~~
cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=OFF -D WITH_OPENCL=OFF -D OPENCV_PC_FILE_NAME=opencv4.pc -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=OFF -D WITH_V4L=OFF -D WITH_VTK=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D OPENCV_GENERATE_PKGCONFIG=ON -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules /opt/opencv/
~~~


### CUDA OPENCV
~~~
cmake -D CMAKE_BUILD_TYPE=RELEASE \
         -D CMAKE_INSTALL_PREFIX=/usr/local \
         -D INSTALL_C_EXAMPLES=OFF \
         -D INSTALL_PYTHON_EXAMPLES=OFF \
         -D WITH_TBB=ON \
	     -D WITH_CUDA=ON \
         -D ENABLE_FAST_MATH=ON \
         -D NVCUVID_FAST_MATH=ON \
         -D CUDA_FAST_MATH=ON \
         -D WITH_CUBLAS=ON \
         -D BUILD_opencv_java=OFF \
         -D BUILD_ZLIB=ON \
         -D BUILD_TIFF=ON \
         -D WITH_GTK=ON \
         -D WITH_NVCUVID=ON \
         -D WITH_FFMPEG=ON \
         -D WITH_1394=ON \
         -D BUILD_PROTOBUF=ON \
         -D OPENCV_GENERATE_PKGCONFIG=ON \
         -D OPENCV_PC_FILE_NAME=opencv4.pc \
         -D OPENCV_ENABLE_NONFREE=OFF \
         -D WITH_GSTREAMER=ON \
         -D WITH_V4L=ON \
         -D WITH_QT=ON \
         -D WITH_CUDNN=ON \
         -DBUILD_opencv_dnn=OFF \
         -D WITH_OPENGL=ON \
         -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
         -D BUILD_EXAMPLES=ON ..
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

## get version
~~~
pkg-config --modversion opencv4
~~~

## opencv VIZ

~~~
–D WITH_VTK=ON ../
~~~

~~~
check yerson001/myslam/README.md

~~~


~~~
sudo rm -rf /usr/local/include/opencv4 /usr/local/include/opencv2 /usr/local/share/opencv4 /usr/local/lib/libopencv*
sudo rm /usr/local/lib/pkgconfig/opencv4.pc

cd opencv/build
sudo make uninstall


cd opencv/build
sudo make clean
sudo make cleanall


sudo rm /usr/local/lib/cmake/opencv4/OpenCVConfig.cmake
sudo rm /usr/local/lib/cmake/opencv4/OpenCVConfig-version.cmake
sudo rm /usr/local/lib/cmake/opencv4/OpenCVModules.cmake
sudo rm /usr/local/lib/cmake/opencv4/OpenCVModules-release.cmake

sudo nano /etc/environment
sudo ldconfig


~~~
