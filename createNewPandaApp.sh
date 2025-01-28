#/bin/bash
# createNewPandaApp.sh
# Author: Jonathan Sprinkle
# Copyright (c) 2023 Vanderbilt University
# License: BSD 3-clause

# This file will help generate the basis for a Panda application for running apps on the car.
# It should be run with no command line parameters, and the prompts will request information.

# RAW FILES that form the basis for codegen are below

# To read variables into a bash string, the
# << reads in from the stdin, which
# - means to skip leading spaces (or maybe tabs?)
# 'EOF' rather than EOF means to avoid expansions ${INBRACKETS} `or execs`

# this file should be run as
# cd catkin_ws/src/repository
# ./path/to/createNewPandaApp.sh

read -r -d '' DESCRIPTION <<-'EOF'
{APP_DESCRIPTION}
EOF


read -r -d '' INSTALL <<-'EOF'
#!/bin/bash

echo "=========================="
echo "Installing App {APPNAMEFOLDER}"

# Here is where we perform installation of scripts, services, etc.
echo " - Installing ROS packages for {APPNAMEFOLDER}..."

LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)
LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)
LAUNCH_FILE={LAUNCHFILE}

source /home/$LIBPANDA_USER/.bashrc

# if [ -d /home/$LIBPANDA_USER/strym ]; then
#     pushd /home/$LIBPANDA_USER/strym
#     git pull
# else
#     pushd /home/$LIBPANDA_USER/
#     git clone https://github.com/jmscslgroup/strym
# fi
# popd

runuser -l $LIBPANDA_USER -c /etc/libpanda.d/apps/{APPNAMEFOLDER}/installRosPackages.sh

echo "Installing {APP_PRETTY_NAME}..."
# runuser -l $LIBPANDA_USER -c /etc/libpanda.d/apps/vsl/installMidVslController.sh
pushd /home/$LIBPANDA_USER/catkin_ws
runuser -l $LIBPANDA_USER -c 'source /opt/ros/noetic/setup.bash && cd catkin_ws && catkin_make'
source devel/setup.sh
rosrun robot_upstart install {REPONAME}/launch/${LAUNCH_FILE} --user root

echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable {REPONAMECLEAN}
popd
echo "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
EOF


read -r -d '' INSTALLCONTROLLER <<-'EOF'
#!/bin/bash
# Author: Matt Nice,Matt Bunting

LAUNCH_FILE={LAUNCHFILE}

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

source ~/.bashrc

pushd ~/catkin_ws
source devel/setup.sh

# rosrun robot_upstart install can_to_ros/launch/${LAUNCH_FILE} --user root
rosrun robot_upstart install ${LAUNCH_FILE} --user root

echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable {REPONAMECLEAN}
popd

echo "----------------------------"
EOF


# It is not clear whether this is needed or not...
read -r -d '' INSTALLROSPACKAGES <<-'EOF'
#!/bin/bash
# Author: Matt Bunting, Matt Nice

echo "----------------------------"
if [[ $EUID == 0 ]];
  then echo "Do NOT run this script as root"
  exit
fi

echo "Installing/Updating {APP_NAME_PRETTY} packages"

source ~/.bashrc

LIBPANDA_SRC=$(cat /etc/libpanda.d/libpanda_src_dir)


cd ~
if [ ! -d catkin_ws/src ]; then
    mkdir -p catkin_ws/src
fi

cd ~/catkin_ws
source /opt/ros/noetic/setup.bash

echo "Regenerating CanToRos"
cd ~/catkin_ws/src/can_to_ros/scripts
echo y | ./regenerateCanToRos.sh


# Build:
source ~/catkin_ws/devel/setup.sh
cd ~/catkin_ws
catkin_make


echo "Enabling can_to_ros startup script"
sudo systemctl daemon-reload
sudo systemctl enable {REPONAMECLEAN}



echo "----------------------------"
EOF


read -r -d '' SERVICE <<-'EOF'
{REPONAME}
EOF


read -r -d '' START <<-'EOF'
#!/bin/bash

echo "=========================="
echo "Starting App {APPNAME} for {APP_PRETTY_NAME}"


systemctl start {REPONAMECLEAN}
systemctl start rosnodeChecker
EOF


read -r -d '' STOP <<-'EOF'
#!/bin/bash

echo "=========================="
echo "Stopping App {APPNAME}"

systemctl stop rosnodeChecker
systemctl stop {REPONAMECLEAN}
EOF

read -r -d '' UNINSTALL <<-'EOF'
#!/bin/bash

echo "=========================="
echo "Removing App {APPNAME}"


LIBPANDA_USER=$(cat /etc/libpanda.d/libpanda_usr)

# Disable the installed services:
echo " - Disabling startup scripts..."
systemctl disable {REPONAMECLEAN}


# Here is where we remove scripts, services, etc.
echo " - Removing scripts..."
cd
if [ "x"`systemctl list-units | grep -c {REPONAMECLEAN}.service` = "x1" ]; then
    echo "Uninstalling {REPONAMECLEAN}.service"

    source /home/$LIBPANDA_USER/catkin_ws/devel/setup.bash
    rosrun robot_upstart uninstall {REPONAMECLEAN}
fi

systemctl daemon-reload # if needed
EOF

read -r -d '' LIBPANDAPPS <<-'EOF'
apps:
  - name: {APPNAMEFOLDER}
    properties:
      - owns_panda: true
    dependencies:
      - apt: python3-pandas
      - pip3: cantools
      - pip3: shapely
    ros_repositories:
        - owner: jmscslgroup
          repo: can_to_ros
        - owner: jmscslgroup
          repo: cbf
          branch_or_hash: original_CBF_implementation_2021
        - owner: jmscslgroup
          repo: velocity_controller
        - owner: jmscslgroup
          repo: setpointreader
        - owner: {GITHUB_OWNER}
          repo: {REPONAME}
EOF


APPNAME=
REPONAME=
APP_PRETTY_NAME=""
APP_DESCRIPTION=""
GITHUB_OWNER=

# now we read variables

read -p 'Enter the app pretty name (spaces ok): ' APP_PRETTY_NAME
echo "${APP_PRETTY_NAME}" is the name of the app.

read -p 'What is the github owner/org for the repository for this app? ' GITHUB_OWNER
read -p 'What is the github repo for this app?: ' REPONAME
read -p 'Enter a 1-line sentence that describes this app: ' APP_DESCRIPTION
read -p 'What is the app shortname (no spaces or special chars)?: ' APPNAME
read -p 'What is the path-free name of the launchfile to load on startup in the ${REPONAME}/launch folder? [file.launch]: ' LAUNCHFILE



# get the version before the underscore
APPNAMEFOLDER=$(echo ${APPNAME})
APPNAMECLEAN=$(echo ${APPNAMEFOLDER} | cut -d'_' -f1)
REPONAMECLEAN=$(echo ${REPONAME} | cut -d'_' -f1)

DESCRIPTIONFILE=description.txt
INSTALLFILE=install.sh
INSTALLCONTROLLERFILE=installController.sh
INSTALLROSPACKAGESFILE=installRosPackages.sh
SERVICEFILE=service
STARTFILE=start.sh
STOPFILE=stop.sh
UNINSTALLFILE=uninstall.sh
YAMLFILE=libpanda-apps.yaml


# now generate all the files in the output directory
echo "Making folder ${APPNAMEFOLDER} (if it does not exist)"
mkdir -p ${APPNAMEFOLDER}

# to get the output to work, we have to use single quotes, then double quotes
# next is description file
echo "Generating ${DESCRIPTIONFILE}"
echo "${DESCRIPTION}" | sed -e 's/{APP_DESCRIPTION}/'"${APP_DESCRIPTION}/" \
| sed -e 's/{GITHUB_OWNER}/'"${GITHUB_OWNER}/" \
| sed -e 's/{REPONAME}/'"${REPONAME}/" \
| sed -e 's/{REPONAMECLEAN}/'"${REPONAMECLEAN}/" \
| sed -e 's/{APPNAME}/'"${APPNAME}/" \
| sed -e 's/{APPNAMECLEAN}/'"${APPNAMECLEAN}/" \
| sed -e 's/{APPNAMEFOLDER}/'"${APPNAMEFOLDER}/" \
| sed -e 's/{APPNAMEPRETTY}/'"${APPNAMEPRETTY}/" \
| sed -e 's/{LAUNCHFILE}/'"${LAUNCHFILE}/" \
> ${APPNAMEFOLDER}/${DESCRIPTIONFILE}

# next is install file
echo "Generating ${INSTALLFILE}"
echo "${INSTALL}" | sed -e 's/{APP_DESCRIPTION}/'"${APP_DESCRIPTION}/" \
| sed -e 's/{GITHUB_OWNER}/'"${GITHUB_OWNER}/" \
| sed -e 's/{REPONAME}/'"${REPONAME}/" \
| sed -e 's/{REPONAMECLEAN}/'"${REPONAMECLEAN}/" \
| sed -e 's/{APPNAME}/'"${APPNAME}/" \
| sed -e 's/{APPNAMECLEAN}/'"${APPNAMECLEAN}/" \
| sed -e 's/{APPNAMEFOLDER}/'"${APPNAMEFOLDER}/" \
| sed -e 's/{APPNAMEPRETTY}/'"${APPNAMEPRETTY}/" \
| sed -e 's/{LAUNCHFILE}/'"${LAUNCHFILE}/" \
> ${APPNAMEFOLDER}/${INSTALLFILE}

# next is INSTALLCONTROLLERFILE
echo "Generating ${INSTALLCONTROLLERFILE}"
echo "${INSTALLCONTROLLER}" | sed -e 's/{APP_DESCRIPTION}/'"${APP_DESCRIPTION}/" \
| sed -e 's/{GITHUB_OWNER}/'"${GITHUB_OWNER}/" \
| sed -e 's/{REPONAME}/'"${REPONAME}/" \
| sed -e 's/{REPONAMECLEAN}/'"${REPONAMECLEAN}/" \
| sed -e 's/{APPNAME}/'"${APPNAME}/" \
| sed -e 's/{APPNAMECLEAN}/'"${APPNAMECLEAN}/" \
| sed -e 's/{APPNAMEFOLDER}/'"${APPNAMEFOLDER}/" \
| sed -e 's/{APPNAMEPRETTY}/'"${APPNAMEPRETTY}/" \
| sed -e 's/{LAUNCHFILE}/'"${LAUNCHFILE}/" \
> ${APPNAMEFOLDER}/${INSTALLCONTROLLERFILE}

# next is INSTALLROSPACKAGESFILE
echo "Generating ${INSTALLROSPACKAGESFILE}"
echo "${INSTALLROSPACKAGES}" | sed -e 's/{APP_DESCRIPTION}/'"${APP_DESCRIPTION}/" \
| sed -e 's/{GITHUB_OWNER}/'"${GITHUB_OWNER}/" \
| sed -e 's/{REPONAME}/'"${REPONAME}/" \
| sed -e 's/{REPONAMECLEAN}/'"${REPONAMECLEAN}/" \
| sed -e 's/{APPNAME}/'"${APPNAME}/" \
| sed -e 's/{APPNAMECLEAN}/'"${APPNAMECLEAN}/" \
| sed -e 's/{APPNAMEFOLDER}/'"${APPNAMEFOLDER}/" \
| sed -e 's/{APPNAMEPRETTY}/'"${APPNAMEPRETTY}/" \
| sed -e 's/{LAUNCHFILE}/'"${LAUNCHFILE}/" \
> ${APPNAMEFOLDER}/${INSTALLROSPACKAGESFILE}

# next is SERVICEFILE
echo "Generating ${SERVICEFILE}"
echo "${SERVICE}" | sed -e 's/{APP_DESCRIPTION}/'"${APP_DESCRIPTION}/" \
| sed -e 's/{GITHUB_OWNER}/'"${GITHUB_OWNER}/" \
| sed -e 's/{REPONAME}/'"${REPONAME}/" \
| sed -e 's/{REPONAMECLEAN}/'"${REPONAMECLEAN}/" \
| sed -e 's/{APPNAME}/'"${APPNAME}/" \
| sed -e 's/{APPNAMECLEAN}/'"${APPNAMECLEAN}/" \
| sed -e 's/{APPNAMEFOLDER}/'"${APPNAMEFOLDER}/" \
| sed -e 's/{APPNAMEPRETTY}/'"${APPNAMEPRETTY}/" \
| sed -e 's/{LAUNCHFILE}/'"${LAUNCHFILE}/" \
> ${APPNAMEFOLDER}/${SERVICEFILE}

# next is STARTFILE
echo "Generating ${STARTFILE}"
echo "${START}" | sed -e 's/{APP_DESCRIPTION}/'"${APP_DESCRIPTION}/" \
| sed -e 's/{GITHUB_OWNER}/'"${GITHUB_OWNER}/" \
| sed -e 's/{REPONAME}/'"${REPONAME}/" \
| sed -e 's/{REPONAMECLEAN}/'"${REPONAMECLEAN}/" \
| sed -e 's/{APPNAME}/'"${APPNAME}/" \
| sed -e 's/{APPNAMECLEAN}/'"${APPNAMECLEAN}/" \
| sed -e 's/{APPNAMEFOLDER}/'"${APPNAMEFOLDER}/" \
| sed -e 's/{APPNAMEPRETTY}/'"${APPNAMEPRETTY}/" \
| sed -e 's/{LAUNCHFILE}/'"${LAUNCHFILE}/" \
> ${APPNAMEFOLDER}/${STARTFILE}

# next is STOPFILE
echo "Generating ${STOPFILE}"
echo "${STOP}" | sed -e 's/{APP_DESCRIPTION}/'"${APP_DESCRIPTION}/" \
| sed -e 's/{GITHUB_OWNER}/'"${GITHUB_OWNER}/" \
| sed -e 's/{REPONAME}/'"${REPONAME}/" \
| sed -e 's/{REPONAMECLEAN}/'"${REPONAMECLEAN}/" \
| sed -e 's/{APPNAME}/'"${APPNAME}/" \
| sed -e 's/{APPNAMECLEAN}/'"${APPNAMECLEAN}/" \
| sed -e 's/{APPNAMEFOLDER}/'"${APPNAMEFOLDER}/" \
| sed -e 's/{APPNAMEPRETTY}/'"${APPNAMEPRETTY}/" \
| sed -e 's/{LAUNCHFILE}/'"${LAUNCHFILE}/" \
> ${APPNAMEFOLDER}/${STOPFILE}

# next is UNINSTALLFILE
echo "Generating ${UNINSTALLFILE}"
echo "${UNINSTALL}" | sed -e 's/{APP_DESCRIPTION}/'"${APP_DESCRIPTION}/" \
| sed -e 's/{GITHUB_OWNER}/'"${GITHUB_OWNER}/" \
| sed -e 's/{REPONAME}/'"${REPONAME}/" \
| sed -e 's/{REPONAMECLEAN}/'"${REPONAMECLEAN}/" \
| sed -e 's/{APPNAME}/'"${APPNAME}/" \
| sed -e 's/{APPNAMECLEAN}/'"${APPNAMECLEAN}/" \
| sed -e 's/{APPNAMEFOLDER}/'"${APPNAMEFOLDER}/" \
| sed -e 's/{APPNAMEPRETTY}/'"${APPNAMEPRETTY}/" \
| sed -e 's/{LAUNCHFILE}/'"${LAUNCHFILE}/" \
> ${APPNAMEFOLDER}/${UNINSTALLFILE}

# generate the libpanda-apps.yaml file
# HACK: (if it already exists, this will go at the bottom)
# final is YAML file
echo "Generating ${YAMLFILE}"
echo "${LIBPANDAPPS}" | sed -e 's/{APP_DESCRIPTION}/'"${APP_DESCRIPTION}/" \
| sed -e 's/{GITHUB_OWNER}/'"${GITHUB_OWNER}/" \
| sed -e 's/{REPONAME}/'"${REPONAME}/" \
| sed -e 's/{REPONAMECLEAN}/'"${REPONAMECLEAN}/" \
| sed -e 's/{APPNAME}/'"${APPNAME}/" \
| sed -e 's/{APPNAMECLEAN}/'"${APPNAMECLEAN}/" \
| sed -e 's/{APPNAMEFOLDER}/'"${APPNAMEFOLDER}/" \
| sed -e 's/{APPNAMEPRETTY}/'"${APPNAMEPRETTY}/" \
| sed -e 's/{LAUNCHFILE}/'"${LAUNCHFILE}/" \
>> ${YAMLFILE}

echo "Done."
