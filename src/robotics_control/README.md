# Install this

Install [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/installation/installation.html#linux-ubuntu) 

```
sudo apt-get install libboost-all-dev
pip install --user pybind11
pip3 install --user pybind11

git clone https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
mkdir build
cd build
cmake ..
make
sudo make install

```
