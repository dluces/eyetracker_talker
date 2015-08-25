Eye Tracker (talker)
====================

This is a ROS (winros) package that publishes Gaze data from a Tobii EyeX eye tracker into an /eyetracker topic.


## Installing win_ros

To be updated

  - Install win_ros as described in: http://wiki.ros.org/win_ros/hydro/Msvc%20Overlays (see the next point for prereqs)
  - Install the compiled SDK as described in: http://wiki.ros.org/win_ros/hydro/Msvc%20Compiled%20SDK  (see inner points for prereqs)
    - Install the prerequisites for the compiled SDK.
      - Install win_python_build_tools 0.2.5 as described in: http://wiki.ros.org/win_python_build_tools/hydro
      - As per Windows SDK 7.1, I recommend that you install Microsoft Visual Studio 10 and its SP1. After that, you can install Windows SDK 7.1 as in the instructions. This is the only setup that worked for me, I don't know why.
    - (optional) Do point 4 to use a nicer replacement for cmd.exe
    - By now, you should have a working winros workspace.
  - (optional) We might need to make some python scripts executable from the command line (they have no extension so they won't be grabbed by the windows command line utility):
    - Create a .bat script named the same as the extension-less python script:
      - In C:/Python27/Scripts
        - all ros and catkin scripts
      - In C:\opt\ros\hydro\x86\bin
        - all winros and catkin executables/scripts
    - This is the content:
      ```
      REM Start of proxy batch script
      @echo off
      python "%~dp0%~n0" %*
      REM End of proxy batch script
      ```

### More Information

Read on how to develop using Catkin in Windows: http://wiki.ros.org/win_ros/hydro/Msvc%20Overlays

*Note:*
If the setup.bat file referred to in the instructions (the one for the workspace) had in comments "Could not find windows sdk or visual studio, please ...", it means that the tool could not find the dev environment automatically (see point 2.2 in the webpage). Make sure you have MS Visual Studio 2010 installed along with its SP1, and only then install Windows SDK 7.1. I did not try this on a completely clean Windows install, so it might have to do with the fact that I have multiple VS versions installed.


## Install the eytracker_talker package

Clone this repo in the src directory of your workspace.


## Building the Project

  - Do a regular package build, using cmake.
    - See: http://wiki.ros.org/win_ros/hydro/Msvc%20Overlays
    - Execute the following commands:
      ```
      > {root}/setup.bat
      > winros_init_build --underlays="C:/opt/ros/hydro/x86"
      > winros_make
      ```
  - You'll now see a devel directory and inside a bin directory.
  - In the {root} directory, there will be a config.cmake created when running winros_init_buil. Edit it so that line 7 (or similar, use logic) looks like this instead:
    ```
    set(INSTALL_ROOT "C:/opt/ros/hydro/x86" CACHE PATH "Install root.")
    ```
  - To install the node, execute:
    ```
    > winros_make -i
    ```

## Running the Project

### Windows

  - Open one terminal (cmd.exe) and execute the following commands:
    ```
    > call C:\opt\ros\hydro\x86\setup.bat
    > roslaunch eyetracker_talker talker.launch
    ```

### Ubuntu

  - Open one terminal and execute the following commands:
    ```
    > rostopic echo /eyetracker
    ```

## Libraries and Requirements

This project needs, of course, a machine with a Tobii EyeX controller. It also needs the Tobii Engine running in the same machine.

This project uses and includes the Tobii EyeX C++ SDK header files. The compiled libraries should be placed in a lib directory. The lib directory should have an x86 and an x64 folders with Tobii.EyeX.Client.dll and Tobii.EyeX.Client.lib files in both. These files are required to compile, install and run the node.