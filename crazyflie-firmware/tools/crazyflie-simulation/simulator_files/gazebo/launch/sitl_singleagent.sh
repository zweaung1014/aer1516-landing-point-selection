#!/bin/bash
function cleanup() {
	pkill -x cf2
	pkill -9 ruby
}

function spawn_model() {
	MODEL=$1
	N=$2 # Cf ID
	X=$3 # spawn x position
	Y=$4 # spawn y position
	Z=$5 # spawn z position
	X=${X:=$X}
	Y=${Y:=$Y}
	SUPPORTED_MODELS=("crazyflie", "crazyflie_thrust_upgrade")
	if [[ " ${SUPPORTED_MODELS[*]} " != *"$MODEL"* ]];
	then
		echo "ERROR: Currently only vehicle model $MODEL is not supported!"
		echo "       Supported Models: [${SUPPORTED_MODELS[@]}]"
		trap "cleanup" SIGINT SIGTERM EXIT
		exit 1
	fi

	working_dir="$build_path/$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null

	set --
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/launch/jinja_gen.py
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/models/${MODEL}/model.sdf.jinja
	set -- ${@} ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo
	set -- ${@} --cffirm_udp_port $((19950+${N}))
	set -- ${@} --cflib_udp_port $((19850+${N}))
	set -- ${@} --cf_id $((${N}))
	set -- ${@} --cf_name cf
	set -- ${@} --output-file /tmp/${MODEL}_${N}.sdf

	python3 ${@}

	echo "Spawning ${MODEL}_${N} at ${X} ${Y} ${Z}"

    gz service -s /world/${world}/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 300 --req 'sdf_filename: "/tmp/'${MODEL}_${N}'.sdf", pose: {position: {x:'${X}', y:'${Y}', z: '${Z}'}}, name: "'${MODEL}_${N}'", allow_renaming: 1'

	if [ "$HOLD" == "1" ]; then
		# -H: hold the robot where it spawns until something publishes on
		# /cf_N/release (hopcopter_perception's lio_anchor, once FAST-LIO is up).
		# A hidden static model is the DetachableJoint's parent and the robot its
		# child: the joint keeps the robot still with motors idle, and the robot
		# drops when it detaches, as it does at Play without -H.
		cat > /tmp/${MODEL}_${N}_holder.sdf <<EOF_HOLDER
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="${MODEL}_${N}_holder">
    <static>true</static>
    <link name="link"/>
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>link</parent_link>
      <child_model>${MODEL}_${N}</child_model>
      <child_link>base_link</child_link>
      <detach_topic>/cf_${N}/release</detach_topic>
      <suppress_child_warning>true</suppress_child_warning>
    </plugin>
  </model>
</sdf>
EOF_HOLDER
		echo "Holding ${MODEL}_${N} until /cf_${N}/release"
		gz service -s /world/${world}/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 300 --req 'sdf_filename: "/tmp/'${MODEL}_${N}'_holder.sdf", pose: {position: {x:'${X}', y:'${Y}', z: '${Z}'}}, name: "'${MODEL}_${N}'_holder", allow_renaming: 0'
	fi
	
	echo "starting instance $N in $(pwd)"
	$build_path/cf2 $((19950+${N})) > out.log 2> error.log &

	popd &>/dev/null
}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
    echo "Description: This script is used to spawn a single vehicle in gazebo."
	echo "Usage: $0 [-m <vehicle_model>] [-n <x_coordinate>] [-y <y_coordinate>] [-z <z_coordinate>] [-w <world>] [-H]"
	echo "  -H: hold the robot at its spawn pose until /cf_0/release (the LiDAR pipeline's lio_anchor)"
	exit 1
fi

HOLD=0
while getopts m:x:y:z:w:H option
do
	case "${option}"
	in
		m) VEHICLE_MODEL=${OPTARG};;
        x) X_CORD=${OPTARG};;
        y) Y_CORD=${OPTARG};;
        z) Z_CORD=${OPTARG};;
		w) WORLD=${OPTARG};;
		H) HOLD=1;;
	esac
done

world=${WORLD:=crazysim_default}
x_cord=${X_CORD:=0}
y_cord=${Y_CORD:=0}
z_cord=${Z_CORD:=1.34128}  # spawn height; raise it for worlds that start on a platform
vehicle_model=${VEHICLE_MODEL:="crazyflie"}
export CF2_SIM_MODEL=gz_${vehicle_model}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../../../.."

build_path=${src_path}/sitl_make/build

echo "killing running crazyflie firmware instances"
pkill -x cf2 || true

sleep 1

source ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/launch/setup_gz.bash ${src_path} ${build_path}

echo "Starting gazebo"
gz sim -s ${src_path}/tools/crazyflie-simulation/simulator_files/gazebo/worlds/${world}.sdf -v 3 &
sleep 3

n=0
spawn_model ${vehicle_model} $(($n)) ${x_cord} ${y_cord} ${z_cord}

trap "cleanup" SIGINT SIGTERM EXIT

echo "Starting gazebo gui"
#gdb ruby
gz sim -g