# MRSFramework

## Description

c++ Framework for MRS

* `Initiator`: requests a service;
* `Participant`: offers a service;
* `Watcher`: stops simulation when all negotiations are completed.


## Installation

### Web Application

In order to execute the Web Application the [Webtoolkit](https://www.webtoolkit.eu/wt) must be installed. 

Please follow the [Webtoolkit Installation](https://redmine.webtoolkit.eu/projects/wt/wiki/Wt_Installation) for your Operational System.

```
git clone https://github.com/emweb/wt.git

cd wt-x.x.x
mkdir build
cd build
cmake ../

make
make install

```

### ROS Simulation

In case of Gazebo simulation, ROS also needs to be installed. 

```
pip3 install -r requirements.txt

cd jade
make

cd ..
python3 benchmark.py
```
