#!/bin/bash

yellow='\E[0;33m'
red='\E[0;31m'
wipe="\033[1m\033[0m"

echo_yellow "Welcome to the Node Creator!"

echo -e "What is the name of the ${red}class${wipe} you're wrapping a Node around?  ${red}[camelCase format]${wipe}"
read CLASS 

echo -e "What is the name of the ${red}class${wipe} you're wrapping a node around in? ${red}[lower_case format]${wipe}"
read CLASS_LC

echo -e "What ${red}topic${wipe} would you like to subscribe to? (no quotes)"
read SUB_TOPIC

echo -e "What is the ${red}namespace${wipe} of the message type you're subcribing to? [${yellow}relative_nav_msgs${wipe}::FilterState ]"
read SUB_NAMESPACE

echo -e "What is the message ${red}type${wipe} you're subcribing to? [relative_nav_msgs::${yellow}FilterState${wipe}]"
read SUB_TYPE

echo -e "What ${red}topic${wipe} would you like to publish to? (no quotes)"
read PUB_TOPIC

echo -e "What is the ${red}namespace${wipe} of the message type you're publishing to? [${yellow}relative_nav_msgs${wipe}::FilterState ]"
read PUB_NAMESPACE

echo -e "What is the message ${red}type${wipe} you're publishing to? [relative_nav_msgs::${yellow}FilterState${wipe}]"
read PUB_TYPE

echo -e "What is your ${red}email${wipe}?"
read EMAIL

echo -e "What is your ${red}name${wipe}?"
read PROGRAMMER

FILES=(include/CLASS_LC/CLASS_LC.h src/CLASS_LC_node.cpp src/CLASS_LC.cpp package.xml CMakeLists.txt)

echo -en "${red}Modifying Files "
for i in ${FILES[@]}; do
	echo -en "."
	sed -i -e "s/CLASS_LC/${CLASS_LC}/g" ${i}
	sed -i -e "s/CLASS_UC/${CLASS_LC^^}/g" ${i}
	sed -i -e "s/CLASS/${CLASS}/g" ${i}
	sed -i -e "s/PARAM/param/g" ${i}

	sed -i -e "s/SUB_NAMESPACE/${SUB_NAMESPACE}/g" ${i}

	sed -i -e "s/SUB_TOPIC/${SUB_TOPIC}/g" ${i}
	sed -i -e "s/SUB_TYPE/${SUB_TYPE}/g" ${i}

	sed -i -e "s/PUB_NAMESPACE/${PUB_NAMESPACE}/g" ${i}
	sed -i -e "s/PUB_TOPIC/${PUB_TOPIC}/g" ${i}
	sed -i -e "s/PUB_TYPE/${PUB_TYPE}/g" ${i}

	sed -i -e "s/NAMESPACE/${CLASS_LC}/g" ${i}
	sed -i -e "s/EMAIL@gmail.com/${EMAIL}/g" ${i}
	sed -i -e "s/PROGRAMMER/${PROGRAMMER}/g" ${i}
	read DONE
done
echo -en "."
mv include/CLASS_LC/CLASS_LC.h include/CLASS_LC/${CLASS_LC}.h
mv include/CLASS_LC include/${CLASS_LC}

mv src/CLASS_LC.cpp src/${CLASS_LC}.cpp
mv src/CLASS_LC_node.cpp src/${CLASS_LC}_node.cpp
echo ""

cd ../..

echo "Done moving files.  Perform catkin_make? [Y/n]"
read BUILD

if [ $BUILD == 'n' ]; then
	echo "skipping build"
else
	catkin_make
fi

echo "Finished.  Remember to rename the folder and restart the .git directory"


