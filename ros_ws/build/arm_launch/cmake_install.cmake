# Install script for directory: /home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/src/arm_launch

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/install/arm_launch")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch/" TYPE DIRECTORY FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/src/arm_launch/launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/arm_launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/arm_launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch/environment" TYPE FILE FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch/environment" TYPE FILE FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch" TYPE FILE FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch" TYPE FILE FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch" TYPE FILE FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch" TYPE FILE FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch" TYPE FILE FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_index/share/ament_index/resource_index/packages/arm_launch")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch/cmake" TYPE FILE FILES
    "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_core/arm_launchConfig.cmake"
    "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/ament_cmake_core/arm_launchConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/arm_launch" TYPE FILE FILES "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/src/arm_launch/package.xml")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/sp/Desktop/5th_Semester/RnD/repo/ArmBench-Control_TestBed_for_Robotic_Arms/ros_ws/build/arm_launch/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
