# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/cole/Dropbox/sidd_work/cpp_src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cole/Dropbox/sidd_work/cpp_src

# Include any dependencies generated for this target.
include CMakeFiles/samplerLib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/samplerLib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/samplerLib.dir/flags.make

CMakeFiles/samplerLib.dir/src/Sampler.cpp.o: CMakeFiles/samplerLib.dir/flags.make
CMakeFiles/samplerLib.dir/src/Sampler.cpp.o: src/Sampler.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/cole/Dropbox/sidd_work/cpp_src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/samplerLib.dir/src/Sampler.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/samplerLib.dir/src/Sampler.cpp.o -c /home/cole/Dropbox/sidd_work/cpp_src/src/Sampler.cpp

CMakeFiles/samplerLib.dir/src/Sampler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/samplerLib.dir/src/Sampler.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/cole/Dropbox/sidd_work/cpp_src/src/Sampler.cpp > CMakeFiles/samplerLib.dir/src/Sampler.cpp.i

CMakeFiles/samplerLib.dir/src/Sampler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/samplerLib.dir/src/Sampler.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/cole/Dropbox/sidd_work/cpp_src/src/Sampler.cpp -o CMakeFiles/samplerLib.dir/src/Sampler.cpp.s

CMakeFiles/samplerLib.dir/src/Sampler.cpp.o.requires:
.PHONY : CMakeFiles/samplerLib.dir/src/Sampler.cpp.o.requires

CMakeFiles/samplerLib.dir/src/Sampler.cpp.o.provides: CMakeFiles/samplerLib.dir/src/Sampler.cpp.o.requires
	$(MAKE) -f CMakeFiles/samplerLib.dir/build.make CMakeFiles/samplerLib.dir/src/Sampler.cpp.o.provides.build
.PHONY : CMakeFiles/samplerLib.dir/src/Sampler.cpp.o.provides

CMakeFiles/samplerLib.dir/src/Sampler.cpp.o.provides.build: CMakeFiles/samplerLib.dir/src/Sampler.cpp.o

# Object files for target samplerLib
samplerLib_OBJECTS = \
"CMakeFiles/samplerLib.dir/src/Sampler.cpp.o"

# External object files for target samplerLib
samplerLib_EXTERNAL_OBJECTS =

lib/libsamplerLib.so: CMakeFiles/samplerLib.dir/src/Sampler.cpp.o
lib/libsamplerLib.so: CMakeFiles/samplerLib.dir/build.make
lib/libsamplerLib.so: lib/libproblemDefinitionLib.so
lib/libsamplerLib.so: CMakeFiles/samplerLib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library lib/libsamplerLib.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/samplerLib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/samplerLib.dir/build: lib/libsamplerLib.so
.PHONY : CMakeFiles/samplerLib.dir/build

CMakeFiles/samplerLib.dir/requires: CMakeFiles/samplerLib.dir/src/Sampler.cpp.o.requires
.PHONY : CMakeFiles/samplerLib.dir/requires

CMakeFiles/samplerLib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/samplerLib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/samplerLib.dir/clean

CMakeFiles/samplerLib.dir/depend:
	cd /home/cole/Dropbox/sidd_work/cpp_src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cole/Dropbox/sidd_work/cpp_src /home/cole/Dropbox/sidd_work/cpp_src /home/cole/Dropbox/sidd_work/cpp_src /home/cole/Dropbox/sidd_work/cpp_src /home/cole/Dropbox/sidd_work/cpp_src/CMakeFiles/samplerLib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/samplerLib.dir/depend

