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