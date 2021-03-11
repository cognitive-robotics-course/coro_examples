# Example code for the Cognitive Robotics course

## Table of contents
1. [External dependencies](#external-dependencies)
2. [Installation](#installation)


## External dependencies

### Build dependencies

To build successfully, the `module5` package depends on the following library:

* ncurses

To get this, do the following.

```
sudo apt-get install libncurses-dev
```

### Run-time dependencies

To run successfully, package `module4`  and node `imageAcquisitionFromSimulator` in package `module5`require the installation of the following packages:

* [Lynxmotion AL5D robot description in URDF](https://github.com/cognitive-robotics-course/lynxmotion_al5d_description)

## Installation
The `coro_examples` meta-package can be installed for the first time using the following commands:
```
roscd
cd ../src
git clone https://github.com/cognitive-robotics-course/coro_examples.git
cd ..
catkin_make
```

If you have previously installed  modules from the coro_examples meta-package or just want to install any updates, run the following commands.
```
roscd
cd ../src/coro_examples
git pull origin main
cd ../..
catkin_make
```


