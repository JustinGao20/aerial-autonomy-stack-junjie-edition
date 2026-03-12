#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Set up the simulation
AUTOPILOT="${AUTOPILOT:-px4}" # Options: px4 (default), ardupilot
HEADLESS="${HEADLESS:-false}" # Options: true, false (default)
CAMERA="${CAMERA:-true}" # Options: true (default), false
LIDAR="${LIDAR:-true}" # Options: true (default), false 
#
SIM_SUBNET="${SIM_SUBNET:-10.42}" # Simulation subnet (default = 10.42) Note: this is overridden if INSTANCE != 0
AIR_SUBNET="${AIR_SUBNET:-10.22}" # Inter-vehicle subnet (default = 10.22) Note: this is overridden if INSTANCE != 0
SIM_ID="${SIM_ID:-100}" # Last byte of the simulation container IP (default = 100)
GROUND_ID="${GROUND_ID:-101}" # Last byte of the simulation container IP (default = 101)
#
NUM_QUADS="${NUM_QUADS:-1}" # Number of quadcopters (default = 1)
NUM_VTOLS="${NUM_VTOLS:-0}" # Number of VTOLs (default = 0)
WORLD="${WORLD:-impalpable_greyness}" # Options: impalpable_greyness (default), apple_orchard, shibuya_crossing, swiss_town
#
DEV="${DEV:-false}" # Options: true, false (default)
DEV_SHELL_ENTRYPOINT="${DEV_SHELL_ENTRYPOINT:-false}" # Options: true, false (default). true => override entrypoint with /bin/bash
HITL="${HITL:-false}" # Options: true, false (default)
GND_CONTAINER="${GND_CONTAINER:-true}" # Options: true (default), false
# HuggingFace model cache options for VILA in aircraft container.
HF_CACHE_DIR="${HF_CACHE_DIR:-$HOME/.cache/huggingface-aas}"
VILA_MODEL_ID="${VILA_MODEL_ID:-Efficient-Large-Model/VILA1.5-3b}"
VILA_PREFETCH="${VILA_PREFETCH:-false}" # true => pre-download model before launching aircraft containers
RTF="${RTF:-1.0}" # Real-time factor (default = 1.0), set to <=0.0 for as fast as possible execution
START_AS_PAUSED="${START_AS_PAUSED:-false}" # Options: true, false (default)
INSTANCE="${INSTANCE:-0}" # Integer ID to make docker network/container names unique as well as offsetting the second byte of the subnets (default = 0)
# Set unique subnets and container/network names based on INSTANCE
SIM_BYTE_1=$(echo "$SIM_SUBNET" | cut -d'.' -f1)
SIM_BYTE_2=$(echo "$SIM_SUBNET" | cut -d'.' -f2)
SIM_SUBNET="${SIM_BYTE_1}.$((SIM_BYTE_2 + INSTANCE))"
AIR_BYTE_1=$(echo "$AIR_SUBNET" | cut -d'.' -f1)
AIR_BYTE_2=$(echo "$AIR_SUBNET" | cut -d'.' -f2)
AIR_SUBNET="${AIR_BYTE_1}.$((AIR_BYTE_2 + INSTANCE))"
SIM_NET_NAME="aas-sim-network-inst${INSTANCE}"
AIR_NET_NAME="aas-air-network-inst${INSTANCE}"
SIM_CONT_NAME="simulation-container-inst${INSTANCE}"
GND_CONT_NAME="ground-container-inst${INSTANCE}"

# Detect the environment (Ubuntu/GNOME, WSL, etc.)
if grep -qEi "(Microsoft|WSL)" /proc/version &> /dev/null; then
  DESK_ENV="wsl"
elif command -v gnome-terminal >/dev/null 2>&1 && [ -n "$XDG_CURRENT_DESKTOP" ]; then
  DESK_ENV="gnome"
elif command -v xterm >/dev/null 2>&1; then
  DESK_ENV="linux"
else
  DESK_ENV="linux-headless"
fi
echo "Desktop environment: $DESK_ENV"

if [[ -z "$DISPLAY" ]]; then
  DISPLAY=":0"
fi

USE_X11="false"
if command -v xhost >/dev/null 2>&1 && [[ -n "$DISPLAY" ]]; then
  if xhost +local:docker >/dev/null 2>&1; then
    USE_X11="true"
  fi
fi

if [[ "$USE_X11" == "false" ]]; then
  HEADLESS="true"
fi

if [[ "$USE_X11" == "true" ]] && command -v xterm >/dev/null 2>&1 && [[ -t 0 ]] && [[ -t 1 ]]; then
  LAUNCH_MODE="xterm"
  DOCKER_RUN_FLAGS="-it --rm"
else
  LAUNCH_MODE="detached"
  DOCKER_RUN_FLAGS="-d"
fi

# In dev mode, resources and workspaces are mounted from the host
if [[ "$DEV" == "true" ]]; then
  SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
  DEV_SIM_OPTS=""
  DEV_GND_OPTS=""
  DEV_AIR_OPTS=""

  if [[ "$DEV_SHELL_ENTRYPOINT" == "true" ]]; then
    DEV_SIM_OPTS+=" --entrypoint /bin/bash"
    DEV_GND_OPTS+=" --entrypoint /bin/bash"
    DEV_AIR_OPTS+=" --entrypoint /bin/bash"
  fi

  #
  DEV_SIM_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_resources/:/aas/simulation_resources:cached"
  DEV_SIM_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation.yml.erb:/aas/simulation.yml.erb:cached"
  #
  DEV_GND_OPTS+=" -v ${SCRIPT_DIR}/../ground/ground_resources/:/aas/ground_resources:cached"
  DEV_GND_OPTS+=" -v ${SCRIPT_DIR}/../ground/ground_ws/src:/aas/ground_ws/src:cached"
  #
  DEV_AIR_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_resources/:/aas/aircraft_resources:cached"
  DEV_AIR_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_ws/src:/aas/aircraft_ws/src:cached"
  DEV_AIR_OPTS+=" -v ${SCRIPT_DIR}/../ground/ground_ws/src/ground_system_msgs:/aas/aircraft_ws/src/ground_system_msgs:cached"

  SELECTED_SOFTWAREARCH_PATH=""
  if [[ -n "$SOFTWAREARCH_HOST_PATH" ]]; then
    if [[ -d "$SOFTWAREARCH_HOST_PATH/src" ]]; then
      SELECTED_SOFTWAREARCH_PATH="$SOFTWAREARCH_HOST_PATH"
    else
      echo "Warning: SOFTWAREARCH_HOST_PATH set but no src/ found (${SOFTWAREARCH_HOST_PATH})"
    fi
  else
    REPO_ROOT=$(cd "${SCRIPT_DIR}/../.." && pwd)
    CANDIDATE_SOFTWAREARCH_PATHS=(
      "${REPO_ROOT}/SoftwareArch"
      "${REPO_ROOT}-clone/SoftwareArch"
      "/home/sim/Scene-Aware-UAV-Nevigation/SoftwareArch"
      "/home/sim/Scene-Aware-UAV-Nevigation-clone/SoftwareArch"
    )
    for candidate in "${CANDIDATE_SOFTWAREARCH_PATHS[@]}"; do
      if [[ -d "$candidate/src" ]]; then
        SELECTED_SOFTWAREARCH_PATH="$candidate"
        break
      fi
    done
  fi

  if [[ -n "$SELECTED_SOFTWAREARCH_PATH" ]]; then
    DEV_AIR_OPTS+=" -v ${SELECTED_SOFTWAREARCH_PATH}:/aas/SoftwareArch:cached"
    echo "DEV mount: ${SELECTED_SOFTWAREARCH_PATH} -> /aas/SoftwareArch"
    if [[ -d "${SELECTED_SOFTWAREARCH_PATH}/src/offboard_eval_tools" ]]; then
      DEV_AIR_OPTS+=" -v ${SELECTED_SOFTWAREARCH_PATH}/src/offboard_eval_tools:/aas/aircraft_ws/src/offboard_eval_tools:cached"
      echo "DEV mount: ${SELECTED_SOFTWAREARCH_PATH}/src/offboard_eval_tools -> /aas/aircraft_ws/src/offboard_eval_tools"
    fi
  else
    echo "Warning: no valid SoftwareArch path with src/ found; skipping /aas/SoftwareArch mount"
  fi

  SOFTWARE_ARCH_FINAL_HOST_DIR="/home/sim/Scene-Aware-UAV-Nevigation/SoftwareArchFinal"
  if [[ -d "$SOFTWARE_ARCH_FINAL_HOST_DIR" ]]; then
    DEV_AIR_OPTS+=" -v ${SOFTWARE_ARCH_FINAL_HOST_DIR}:/aas/SoftwareArchFinal:cached"
    echo "DEV mount: ${SOFTWARE_ARCH_FINAL_HOST_DIR} -> /aas/SoftwareArchFinal"
  else
    echo "Warning: ${SOFTWARE_ARCH_FINAL_HOST_DIR} not found; skipping /aas/SoftwareArchFinal mount"
  fi
fi

# Prepare persistent HuggingFace cache mount used by aircraft containers.
mkdir -p "$HF_CACHE_DIR"
HF_CACHE_OPTS="-v ${HF_CACHE_DIR}:/root/.cache/huggingface:rw --env HF_HOME=/root/.cache/huggingface --env TRANSFORMERS_CACHE=/root/.cache/huggingface/hub"

# Optional model prefetch before aircraft containers start.
if [[ "$VILA_PREFETCH" == "true" ]]; then
  echo "Prefetch enabled: downloading ${VILA_MODEL_ID} into ${HF_CACHE_DIR}"
  docker run --rm \
    ${HF_CACHE_OPTS} \
    aircraft-image \
    python3 - <<PY
from huggingface_hub import snapshot_download
snapshot_download(repo_id="${VILA_MODEL_ID}")
print("Prefetch complete: ${VILA_MODEL_ID}")
PY
fi

# Create docker networks for SITL
if [[ "$HITL" == "false" ]]; then
  docker network inspect $SIM_NET_NAME >/dev/null 2>&1 || docker network create --subnet=${SIM_SUBNET}.0.0/16 $SIM_NET_NAME
  docker network inspect $AIR_NET_NAME >/dev/null 2>&1 || docker network create --subnet=${AIR_SUBNET}.0.0/16 $AIR_NET_NAME
fi

# WSL-specific options
WSL_OPTS="--env WAYLAND_DISPLAY=$WAYLAND_DISPLAY --env PULSE_SERVER=$PULSE_SERVER --volume /usr/lib/wsl:/usr/lib/wsl \
--env MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA --env LD_LIBRARY_PATH=/usr/lib/wsl/lib --env LIBGL_ALWAYS_SOFTWARE=0"

# Get display dimensions
resolution=$(xrandr 2>/dev/null | grep " connected primary" | grep -oE '[0-9]+x[0-9]+' | head -1)
if [[ ! "$resolution" =~ ^[0-9]+x[0-9]+$ ]]; then
  resolution=$(xrandr 2>/dev/null | grep " connected" | grep -oE '[0-9]+x[0-9]+' | head -1) # Fallback
fi
if [[ "$resolution" =~ ^[0-9]+x[0-9]+$ ]]; then
  SCREEN_WIDTH=$(echo "$resolution" | cut -d'x' -f1)
  SCREEN_HEIGHT=$(echo "$resolution" | cut -d'x' -f2)
  echo "Detected display: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"
else
  SCREEN_WIDTH=1920
  SCREEN_HEIGHT=1080
  echo "Fallback resolution to ${SCREEN_WIDTH}x${SCREEN_HEIGHT} default"
fi

# Function to calculate terminal position based on ID
calculate_terminal_position() {
  local xterm_id=$1
  SCREEN_SCALE=$((SCREEN_HEIGHT * 100 / 1080)) # Full HD = 100%
  X_POS=$(( (50 + xterm_id * 50) * SCREEN_SCALE / 100 ))
  Y_POS=$(( (xterm_id * 125) * SCREEN_SCALE / 100 ))
}

# Setup terminal dimensions and enable Shift+Ctrl+c, Shift+Ctrl+v copy-paste in Xterm
TERM_COLS=100
TERM_ROWS=32
FONT_SIZE=10
XTERM_CONFIG_ARGS=(
  -xrm 'XTerm*selectToClipboard: true'
  -xrm 'XTerm*VT100.Translations: #override \
    Ctrl Shift <Key>C: copy-selection(CLIPBOARD) \n\
    Ctrl Shift <Key>V: insert-selection(CLIPBOARD)'
)

X11_DOCKER_OPTS=""
if [[ "$USE_X11" == "true" ]]; then
  X11_DOCKER_OPTS="--volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"
fi

run_container_command() {
  local title="$1"
  local cmd="$2"
  local xterm_id="$3"
  if [[ "$LAUNCH_MODE" == "xterm" ]]; then
    calculate_terminal_position "$xterm_id"
    xterm "${XTERM_CONFIG_ARGS[@]}" -title "$title" -fa Monospace -fs $FONT_SIZE -bg black -fg white \
      -geometry "${TERM_COLS}x${TERM_ROWS}+${X_POS}+${Y_POS}" -hold -e bash -c "$cmd" &
  else
    bash -lc "$cmd" >/dev/null
  fi
}

# Launch the simulation container
docker rm -f "$SIM_CONT_NAME" >/dev/null 2>&1 || true
DOCKER_CMD="docker run $DOCKER_RUN_FLAGS \
  $X11_DOCKER_OPTS --gpus all \
  --env NVIDIA_DRIVER_CAPABILITIES=all --env GST_DEBUG=3 \
  --env __NV_PRIME_RENDER_OFFLOAD=1 --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
  --env AUTOPILOT=$AUTOPILOT --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
  --env NUM_QUADS=$NUM_QUADS --env NUM_VTOLS=$NUM_VTOLS --env WORLD=$WORLD \
  --env SIMULATED_TIME=true --env RTF=$RTF --env START_AS_PAUSED=$START_AS_PAUSED \
  --env SIM_SUBNET=$SIM_SUBNET --env GROUND_ID=$GROUND_ID \
  --env GND_CONTAINER=$GND_CONTAINER \
  --env ROS_DOMAIN_ID=$SIM_ID \
  --env HOST_INPUT_GID=$(getent group input | cut -d: -f3) \
  --privileged \
  --name $SIM_CONT_NAME"
# Configure network for HITL or SITL
if [[ "$HITL" == "true" ]]; then
  DOCKER_CMD="$DOCKER_CMD --net=host"
else
  DOCKER_CMD="$DOCKER_CMD --net=$SIM_NET_NAME --ip=${SIM_SUBNET}.90.${SIM_ID}"
fi
# Add WSL-specific options and complete the command
if [[ "$DESK_ENV" == "wsl" ]]; then
  DOCKER_CMD="$DOCKER_CMD $WSL_OPTS"
fi
DOCKER_CMD="$DOCKER_CMD ${DEV_SIM_OPTS} simulation-image"
run_container_command "Simulation" "$DOCKER_CMD" 0

if [[ "$HITL" == "false" ]]; then

  if [[ "$GND_CONTAINER" == "true" ]] && ! docker image inspect ground-image:latest >/dev/null 2>&1; then
    echo "ground-image:latest not found; disabling ground container"
    GND_CONTAINER="false"
  fi

  if [[ "$GND_CONTAINER" == "true" ]]; then
    sleep 1.0 # Limit resource usage
    # Launch the ground container
    docker rm -f "$GND_CONT_NAME" >/dev/null 2>&1 || true
    DOCKER_CMD="docker run $DOCKER_RUN_FLAGS \
      $X11_DOCKER_OPTS --gpus all \
      --env NVIDIA_DRIVER_CAPABILITIES=all --env GST_DEBUG=3 \
      --env __NV_PRIME_RENDER_OFFLOAD=1 --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
      --env HEADLESS=$HEADLESS \
      --env NUM_QUADS=$NUM_QUADS --env NUM_VTOLS=$NUM_VTOLS \
      --env SIMULATED_TIME=true \
      --env ROS_DOMAIN_ID=$GROUND_ID \
      --env HOST_INPUT_GID=$(getent group input | cut -d: -f3) \
      --net=$SIM_NET_NAME --ip=${SIM_SUBNET}.90.${GROUND_ID} \
      --privileged \
      --name $GND_CONT_NAME"
    # Add WSL-specific options and complete the command
    if [[ "$DESK_ENV" == "wsl" ]]; then
      DOCKER_CMD="$DOCKER_CMD $WSL_OPTS"
    fi
    DOCKER_CMD="$DOCKER_CMD ${DEV_GND_OPTS} ground-image"
    run_container_command "Ground" "$DOCKER_CMD" 1
  fi

  # Initialize a counter for the drone IDs
  DRONE_ID=1 # 1, 2, .., N drones

  # Function to launch the aircraft containers
  launch_aircraft_containers() {
    local drone_type=$1
    local num_drones=$2
    
    for i in $(seq 1 $num_drones); do
      sleep 1.0 # Limit resource usage
      local NAME_AIRCRAFT_CNT="aircraft-container-inst${INSTANCE}_${DRONE_ID}"
      docker rm -f "$NAME_AIRCRAFT_CNT" >/dev/null 2>&1 || true
      DOCKER_CMD="docker run $DOCKER_RUN_FLAGS \
        $X11_DOCKER_OPTS --gpus all \
        $HF_CACHE_OPTS \
        --env NVIDIA_DRIVER_CAPABILITIES=all --env GST_DEBUG=3 \
        --env __NV_PRIME_RENDER_OFFLOAD=1 --env __GLX_VENDOR_LIBRARY_NAME=nvidia \
        --env AUTOPILOT=$AUTOPILOT --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
        --env DRONE_TYPE=$drone_type --env DRONE_ID=$DRONE_ID \
        --env SIMULATED_TIME=true \
        --env SIM_SUBNET=$SIM_SUBNET --env AIR_SUBNET=$AIR_SUBNET --env SIM_ID=$SIM_ID --env GROUND_ID=$GROUND_ID \
        --env GND_CONTAINER=$GND_CONTAINER \
        --env ROS_DOMAIN_ID=$DRONE_ID \
        --net=$SIM_NET_NAME --ip=${SIM_SUBNET}.90.$DRONE_ID \
        --privileged \
        --name $NAME_AIRCRAFT_CNT"
      # Add WSL-specific options and complete the command
      if [[ "$DESK_ENV" == "wsl" ]]; then
        DOCKER_CMD="$DOCKER_CMD $WSL_OPTS"
      fi
      DOCKER_CMD="$DOCKER_CMD ${DEV_AIR_OPTS} aircraft-image"
      run_container_command "${drone_type^^} $DRONE_ID" "$DOCKER_CMD" $(($DRONE_ID + 1))
      DRONE_ID=$((DRONE_ID + 1))
    done
  }
  # Launch the Quad containers
  launch_aircraft_containers "quad" $NUM_QUADS
  # Launch the VTOL containers
  launch_aircraft_containers "vtol" $NUM_VTOLS

  if [[ "$GND_CONTAINER" == "true" ]]; then
    sleep 2.0 # Once all containers are up, connect ground and aircraft containers to the air network
    docker network connect --ip=${AIR_SUBNET}.90.$GROUND_ID $AIR_NET_NAME $GND_CONT_NAME
    for i in $(seq 1 $((NUM_QUADS + NUM_VTOLS))); do
      docker network connect --ip=${AIR_SUBNET}.90.$i $AIR_NET_NAME "aircraft-container-inst${INSTANCE}_${i}"
    done
  fi
fi

echo "Fly, my pretties, fly!"
if [[ -r /dev/tty ]]; then
  echo "Press any key to stop all containers and close the terminals..."
  read -n 1 -s < /dev/tty # Wait for user input from terminal device
else
  echo "No interactive TTY detected; waiting for ${SIM_CONT_NAME} to exit."
  docker wait "$SIM_CONT_NAME" >/dev/null 2>&1 || true
fi

# Cleanup function
cleanup() {
  DOCKER_PIDS=$(pgrep -f "docker run.*inst${INSTANCE}" 2>/dev/null || true)
  CONTAINER_NAMES=("${SIM_CONT_NAME}" "${GND_CONT_NAME}" "aircraft-container-inst${INSTANCE}")
  CONTAINERS_TO_STOP=""
  for name in "${CONTAINER_NAMES[@]}"; do
      CONTAINERS_TO_STOP+=$(docker ps -a -q --filter name="${name}" 2>/dev/null || true)
      CONTAINERS_TO_STOP+=" "
  done
  echo "Stopping Docker containers (this will take a few seconds)..."
  if [ -n "$CONTAINERS_TO_STOP" ]; then
      echo "$CONTAINERS_TO_STOP" | xargs docker stop
  fi
  docker network rm $SIM_NET_NAME 2>/dev/null && echo "Removed $SIM_NET_NAME" || echo "Network $SIM_NET_NAME not found or already removed"
  docker network rm $AIR_NET_NAME 2>/dev/null && echo "Removed $AIR_NET_NAME" || echo "Network $AIR_NET_NAME not found or already removed"
  if [ -n "$DOCKER_PIDS" ]; then
    for dpid in $DOCKER_PIDS; do
      PARENT_PID=$(ps -o ppid= -p $dpid 2>/dev/null | tr -d ' ') # Determine process pids with a parent pid
      if [ -n "$PARENT_PID" ]; then
        echo "Killing terminal process $dpid"
        kill $dpid
      fi
    done
  fi
  echo "All-clear"
}
# Set trap to cleanup on script interruption (Ctrl+C, etc.)
trap cleanup EXIT INT TERM
