Steps:

Install ompl:
* sudo apt-get install libompl-dev ompl-demos

Install boost:
* sudo apt-get install libboost-all-dev

Building Code with G++ (not recomended):
* g++ ompl_example.cpp -lboost_system

Building with cmake:
- goto build folder
* cmake ..
* make

How to use OMPL from Source
1. Download the source code: http://ompl.kavrakilab.org/installation.html
2. Run the script inside the directory: ./install-ompl-ubuntu.sh --python
3. In the OMPL folder with the CMake files:
	cmake -DCMAKE_INSTALL_PREFIX=/usr/lib
	make install 