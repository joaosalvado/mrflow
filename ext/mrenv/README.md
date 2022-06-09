# mr_env

Pre-install
===========
    
    Eigen3
    opencv

Setup/Install IRIS-Distro
=========================

    sudo cp mr_env/external/iris-distro/lib/lib* /usr/local/lib
    sudo cp -r * mr_env/external/iris-distro/include/* /usr/local/include
    

Installation
============

This project is configured as a standard CMake project, so the general build process is:

	mkdir build
	cd build
	cmake ..
	make

