# Example code for the Cognitive Robotics course

## Table of contents
1. [External dependencies](#external-dependencies)
2. [Installation](#installation)


### External dependencies
The package currently depends on the successful installation of the
following packages: 
```
sudo apt-get install libncurses-dev
```
* ncurses

### Installation
If this is your first time installing anything from the coro_examples repository, run the following commands.
```
roscd; cd ../src
git clone https://github.com/cognitive-robotics-course/coro_examples.git
cd ..
catkin_make
```

If you have previously installed other modules from the coro_examples repository, run the following commands.
```
roscd; cd ../src/coro_examples
git pull origin main
cd ../..
catkin_make
```


