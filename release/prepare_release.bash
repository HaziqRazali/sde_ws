##### NOTE REMOVE THIS LINE WHEN READY TO USE, CURRENTLY NOT USEABLE FOR SDE_WS

#!/bin/bash

# set foreground and background together
# echo -e "\033[0;FG;BGm"
K_COLOR_FG_BLUE="34"
K_COLOR_FG_BLACK="30"
K_COLOR_FG_CYAN="36"
K_COLOR_FG_DEFAULT="39"
K_COLOR_FG_GREEN="32"
K_COLOR_FG_MAGENTA="35"
K_COLOR_FG_RED="31"
K_COLOR_FG_WHITE="97"
K_COLOR_FG_YELLOW="33"
K_COLOR_BG_BLUE="44"
K_COLOR_BG_BLACK="40"
K_COLOR_BG_CYAN="46"
K_COLOR_BG_DEFAULT="49"
K_COLOR_BG_GREEN="42"
K_COLOR_BG_MAGENTA="45"
K_COLOR_BG_RED="41"
K_COLOR_BG_WHITE="107"
K_COLOR_BG_YELLOW="43"

K_COLOR_BLUE="\033[0;${K_COLOR_FG_BLUE};${K_COLOR_BG_DEFAULT}m"
K_COLOR_CYAN="\033[0;${K_COLOR_FG_CYAN};${K_COLOR_BG_DEFAULT}m"
K_COLOR_GREEN="\033[0;${K_COLOR_FG_GREEN};${K_COLOR_BG_DEFAULT}m"
K_COLOR_MAGENTA="\033[0;${K_COLOR_FG_MAGENTA};${K_COLOR_BG_DEFAULT}m"
K_COLOR_RED="\033[0;${K_COLOR_FG_RED};${K_COLOR_BG_DEFAULT}m"
K_COLOR_WHITE="\033[0;${K_COLOR_FG_WHITE};${K_COLOR_BG_DEFAULT}m"
K_COLOR_YELLOW="\033[0;${K_COLOR_FG_YELLOW};${K_COLOR_BG_DEFAULT}m"
K_COLOR_NONE="\033[0m"

# ---------------------------------------
# make sure ros is present
rosversion=${ROS_DISTRO}
if [ "${#rosversion}" -eq 0 ]
then
    echo -e "${K_COLOR_RED}ROS is not found.${K_COLOR_NONE}"
    exit
fi
echo -e "${K_COLOR_YELLOW}found ROS version ${rosversion}${K_COLOR_NONE}"

# ---------------------------------------
# make sure we are running the script in the releases directory
cwd=$(pwd)
if [[ "$cwd" != *releases ]]
then
    echo -e "${K_COLOR_RED}Please run this tool in the releases directory.${K_COLOR_NONE}"
    exit
fi

# ---------------------------------------
# make sure on right branch
curr_git_branch=`git branch | grep \* | cut -d ' ' -f2-`
if [ "${curr_git_branch}" == "master" ] || [ ]"${curr_git_branch}" == "main" ]
then
    echo -e "${K_COLOR_RED}Cannot create release on master or main branch. Create proper release branch (e.g. i2r_ros2_0.0.1-1) and retry.${K_COLOR_NONE}"
    exit
else
    echo -e "You are on branch '$curr_git_branch'. Is this correct [y/n]? \c"
    read check_response
    if [ "$check_response" != "y" ]
    then
        echo -e "${K_COLOR_RED}Please check and run this tool again.${K_COLOR_NONE}"
        exit
    fi
fi

# ---------------------------------------
# get name of release directory
dirname_release=""
looping=1
while [ "$looping" -eq 1 ]
do
    echo -e "Enter name of release directory to create (e.g. i2r_ros2_0.0.1-1): \c"
    read dirname_release
    if [ -z "$check_response" ]
    then
        echo -e "${K_COLOR_RED}Release directory name cannot be empty. Try again.${K_COLOR_NONE}"
    else
        dirname_release="$cwd/$dirname_release"

        # make sure directory doesn't exist currently
        if [ -d "$dirname_release" ]
        then
            echo -e "${K_COLOR_RED}Directory '$dirname_release' already exist. Try again.${K_COLOR_NONE}"
        else
            # we will create the directories below
            looping=0
        fi
    fi
done

# ---------------------------------------
# get path of mrccc main directory
echo -e "Enter full path of mrccc command centre src directory: \c"
read dirname_cmdcentre
if [ ! -d "$dirname_cmdcentre" ]
then
    echo -e "${K_COLOR_RED}Directory '$dirname_cmdcentre' doesn't exist.${K_COLOR_NONE}"
    exit
fi

# ---------------------------------------
# source the common variables and files only after dirname_release and dirname_cmdcentre have been read
source ./prepare_release_files.bash

# ---------------------------------------
# find all robots
i2r_robot_config_dirs='../src/i2r_robot_config/config'
echo "List of existing robots:"
robot_dirs=(`find $i2r_robot_config_dirs -mindepth 1 -maxdepth 1 -type d | sort`)
declare -a robot_names=()
for robot_dir in "${robot_dirs[@]}"
do
    robot_name="${robot_dir/$i2r_robot_config_dirs\//}"
    robot_names+=($robot_name)
    echo "$robot_name"
done

# allow choosing of multiple robots to release
robots_to_release=()
is_getting_robot_names=1
while [[ $is_getting_robot_names == 1 ]]
do
    echo -e "Enter robot to release (blank to stop): \c"
    read robot_to_release
    if [ -z "$robot_to_release" ]
    then 
        is_getting_robot_names=0
    else 
        robots_to_release+=($robot_to_release)
    fi 
done 

# make sure we got at least one robot name
if [[ ${#robots_to_release[@]} == 0 ]] 
then 
    echo -e "${K_COLOR_RED}No robot names selected. Run this tool again.${K_COLOR_NONE}"
    exit
fi 

# make sure robot names are in our list
for robot_to_release in "${robots_to_release[@]}"
do
    found=0
    for robot_name in "${robot_names[@]}"
    do
        if [[ "$robot_name" = "$robot_to_release" ]]
        then
            found=1
            break
        fi
    done

    if [[ $found == 0 ]]
    then
        echo -e "${K_COLOR_RED}$robot_to_release not found in list. Run this tool again.${K_COLOR_NONE}"
        exit
    fi
done 

# find all directories of robots to remove
declare -a robots_to_remove=()
for robot_name in "${robot_names[@]}"
do
    is_robot_to_release=0
    for robot_to_release in "${robots_to_release[@]}"
    do
        if [[ "$robot_name" == "$robot_to_release" ]]
        then 
            is_robot_to_release=1
            break 
        fi 
    done 

    if [[ $is_robot_to_release == 1 ]] 
    then
        #echo "--> MAINTAINING robot $robot_name"
        continue
    else
        curr_robot_dir="${i2r_robot_config_dirs}/${robot_name}"
        robots_to_remove+=($curr_robot_dir)
    fi
done

# ---------------------------------------
echo -e "\nYour selection as follows:-"
echo -e "curr_git_branch   : $curr_git_branch"
echo "dirname_release   : $dirname_release"
echo "dirname_cmdcentre : $dirname_cmdcentre"

# we are releasing Qt source moving forward, no more binaries
#echo "tool_linuxdeployqt: $tool_linuxdeployqt"
#echo "qt_bin_dir        : $qt_bin_dir"

echo -e "Robot to release  : "
for robot_to_release in "${robots_to_release[@]}"
do
    echo -e "$robot_to_release "
done 
echo ""

# echo "register_robot    : $register_robot_tool"

echo "robots to remove:"
for robot_name_to_remove in "${robots_to_remove[@]}"
do
    echo ${robot_name_to_remove}
done 

echo -e "\nFiles/directories to be removed: "
for filetoremove in "${filestoremove[@]}"
do
    echo $filetoremove
done

echo -e "\n${K_COLOR_YELLOW}Is this correct [y/n]? ${K_COLOR_NONE}\c"
read check_response
if [ "$check_response" != "y" ]
then
    echo -e "${K_COLOR_RED}Please check and run this tool again.${K_COLOR_NONE}"
    exit
fi
echo ""

# ---------------------------------------
function has_path
{
    local path_found=0
    re='/'
    if [[ "${1}" =~ "${re}" ]]
    then
        path_found=1
    fi
    echo $path_found
}

path_valid=1
for curr_retain_mapfname in "${retain_mapfnames[@]}"
do
    fname="$cwd/../src/i2r_robot_config/maps/$curr_retain_mapfname"
    if [ -e "$fname" ]
    then
        # check path of PGM in YAML contents
        if [[ $fname == *".yaml" ]]
        then
            yamlstr=`cat $fname | grep "image:" | grep -v "#image:"`
            ispathfound=$(has_path "$yamlstr")
            if [ "$ispathfound" -eq 1 ]
            then
                expected_path="/opt/ros/$rosversion"
                if [[ ! $yamlstr == *"$expected_path"* ]]
                then
                    echo "Path does not contain '$expected_path' -> $yamlstr"
                    path_valid=0
                fi
            fi
        fi
    else
        echo -e "${K_COLOR_RED}${fname} does not exist${K_COLOR_NONE}"
        path_valid=0
    fi
done
if [ $path_valid -ne 1 ]
then
    exit
fi

# delete other files
all_mapfnames=(`ls -1 $cwd/../src/i2r_robot_config/maps`)
for curr_all_mapfname in "${all_mapfnames[@]}"
do
    isretainfile=0
    for curr_retain_mapfname in "${retain_mapfnames[@]}"
    do
        if [ "$curr_all_mapfname" == "$curr_retain_mapfname" ]
        then
            isretainfile=1
            break
        fi
    done

    currfile="$cwd/../src/i2r_robot_config/maps/$curr_all_mapfname"
    if [ "$isretainfile" -eq 0 ]
    then
        `rm -f $currfile`
        if [ -e "$currfile" ]
        then
            echo -e "${K_COLOR_RED}Cannot delete $currfile${K_COLOR_NONE}"
            exit
        else
            echo "Deleted $currfile"
        fi
    else
        echo "Retaining $currfile"
    fi
done
echo ""

# ---------------------------------------
# create the necessary sub directories.
for subdir in "${subdirs[@]}"
do
    `mkdir -p $subdir`
    if [ -d "$subdir" ]
    then
        echo "Created directory $subdir"
    else
        echo -e "${K_COLOR_RED}Failed to create directory '$subdir'${K_COLOR_NONE}"
        exit
    fi
done
echo ""

# ---------------------------------------
# files to copy
# copy the ubuntu/ROS installation script too 
if [[ "${rosversion}" == "jazzy" ]]
then
    filestocopy+=("$dirname_release/../../tools/install_ros2_u24.sh"
                  "$dirname_release/robot/install_ros2_u24.sh")

    filestocopy+=("$dirname_release/../../tools/cyclone_profiles/cyclone_profile.xml"
                  "$dirname_release/robot/cyclone_profile.xml")
fi

idx=0
while [ "$idx" -lt "${#filestocopy[@]}" ]
do
    file1=${filestocopy[$idx]}
    file2=${filestocopy[$((idx+1))]}
    `cp -r $file1 $file2`
    if [ -e "$file2" ]
    then
        echo "Copied $file1"
    else
        echo -e "${K_COLOR_RED}Failed to copy '$file1'${K_COLOR_NONE}"
        exit
    fi

    idx=$((idx + 2))
done
echo ""

# ---------------------------------------
# files to move
idx=0
while [ "$idx" -lt "${#filestomove[@]}" ]
do
    file1=${filestomove[$idx]}
    file2=${filestomove[$((idx+1))]}
    `mv $file1 $file2`
    if [ -e "$file2" ]
    then
        echo "Moved $file1"
    else
        echo -e "${K_COLOR_RED}Failed to move '$file1'${K_COLOR_NONE}"
        exit
    fi

    idx=$((idx + 2))
done
echo ""

# ---------------------------------------
for filetoremove in "${filestoremove[@]}"
do
    if [ -e "$filetoremove" ]
    then
        `rm -rf $filetoremove`
        if [ ! -e "$filetoremove" ]
        then
            echo "Removed $filetoremove"
        fi
    else
        echo -e "${K_COLOR_RED}Failed to remove '$filetoremove' as it doesn't exist${K_COLOR_NONE}"
    fi
done
echo ""

# ---------------------------------------
# robot directories to remove
for robot_to_remove in "${robots_to_remove[@]}"
do
    if [ -e "$robot_to_remove" ]
    then
        `rm -rf $robot_to_remove`
        if [ ! -e "$robot_to_remove" ]
        then
            echo "Removed $robot_to_remove"
        fi
    else
        echo -e "${K_COLOR_YELLOW}Failed to remove robot directory '$robot_to_remove' as it doesn't exist${K_COLOR_NONE}"
    fi
done
echo ""

# ---------------------------------------
function updateContentsRobotname()
{
    updated_contents=""
    while IFS= read -r line; do
        line="${line//$2/$3}"
        updated_contents+="${line}"$'\n'
    done < "$1"
    echo "$updated_contents"
}

# robot files to copy and rename 
for robot_to_release in "${robots_to_release[@]}"
do
    idx=0
    while [ "$idx" -lt "${#robotfilestocopy[@]}" ]
    do
        file1=${robotfilestocopy[$idx]}
        file2=${robotfilestocopy[$((idx+1))]}
        file2="${file2//robotname/$robot_to_release}"
        echo "copying $file1 to $file2"
        `cp -r $file1 $file2`

        # now go into 2nd folder and edit/change package name to robotname correctly
        pkgname1=$(basename $file1)
        pkgname2=$(basename $file2)

        fname_package_xml="${file2}/package.xml"
        echo "updating package name for $fname_package_xml"
        updated_contents=$(updateContentsRobotname "$fname_package_xml" "$pkgname1" "$pkgname2")
        echo "$updated_contents" > $fname_package_xml

        fname_package_cmake="${file2}/CMakeLists.txt"
        echo "updating package name for $fname_package_cmake"
        updated_contents=$(updateContentsRobotname "$fname_package_cmake" "$pkgname1" "$pkgname2")
        echo "$updated_contents" > $fname_package_cmake

        idx=$((idx + 2))
    done
done

# ---------------------------------------
# final manual steps
echo -e "\n\n==============Manual Steps=============="
echo "i2r_robot_config/maps : make sure all map YAML files are pointing to the correct PGM files"
