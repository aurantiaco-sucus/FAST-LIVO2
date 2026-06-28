#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

CONTAINER_NAME="fastlivo2"
IMAGE_NAME="fast-livo2:latest"

setup_x11() {
    if [[ -z "${DISPLAY:-}" ]]; then
        echo "WARNING: DISPLAY is not set. GUI applications (rviz) will not work."
        return
    fi
    xhost +local: 2>/dev/null || true
}

run_with_roscore() {
    local cmd="$1"
    local extra_args=("${@:2}")
    local script
    script="$(mktemp)"
    cat > "$script" << 'HEREDOC_EOF'
#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash 2>/dev/null || true
roscore &
ROSCORE_PID=$!
for i in $(seq 1 30); do
    if rostopic list >/dev/null 2>&1; then
        break
    fi
    sleep 1
done
exec "$@"
HEREDOC_EOF
    chmod +x "$script"
    podman-compose run --rm \
        -v "${script}:/tmp/run_cmd.sh:ro" \
        "${extra_args[@]}" \
        fastlivo /tmp/run_cmd.sh $cmd
    rm -f "$script"
}

cmd_build() {
    echo "Building container image..."
    podman-compose build
    echo "Done. Run './run.sh shell' to enter the container."
}

cmd_shell() {
    setup_x11
    mkdir -p bags
    podman-compose run --rm fastlivo bash
}

cmd_launch() {
    local launch_file="${1:-mapping_avia.launch}"
    launch_file="${launch_file#launch/}"
    launch_file="${launch_file#./launch/}"
    setup_x11
    mkdir -p bags
    echo "Launching roslaunch fast_livo ${launch_file} ..."
    podman-compose run --rm fastlivo \
        env -u ROS_MASTER_URI roslaunch fast_livo "${launch_file}"
}

cmd_rviz() {
    setup_x11
    run_with_roscore "rviz -d /catkin_ws/src/fast_livo/rviz_cfg/fast_livo2.rviz"
}

cmd_bag() {
    if [[ $# -lt 1 ]]; then
        echo "Usage: ./run.sh bag <path-to-bag-file>"
        exit 1
    fi
    local bag_path bag_dir bag_file
    bag_path="$(realpath "$1")"
    bag_dir="$(dirname "$bag_path")"
    bag_file="$(basename "$bag_path")"
    setup_x11
    mkdir -p bags
    run_with_roscore "rosbag play /bag_data/${bag_file}" \
        -v "${bag_dir}:/bag_data:ro"
}

cmd_rebuild_ws() {
    setup_x11
    echo "Rebuilding catkin workspace inside container..."
    podman-compose run --rm fastlivo bash -c \
        "source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin_make -j\$(nproc) && source devel/setup.bash && echo 'Build complete.'"
}

cmd_clean() {
    echo "Stopping and removing container..."
    podman-compose down --rmi local 2>/dev/null || true
    echo "Done."
}

usage() {
    cat <<EOF
FAST-LIVO2 Container Runner

Usage: ./run.sh <command> [args]

Commands:
  build           Build the container image
  shell           Open an interactive bash shell in the container
  launch [file]   Run a roslaunch file (default: mapping_avia.launch)
  rviz            Launch rviz with the project config
  bag <file>      Play a rosbag file (absolute or relative path)
  rebuild-ws      Rebuild the catkin workspace inside the container
  clean           Remove the container image

Examples:
  ./run.sh build
  ./run.sh shell
  ./run.sh launch mapping_avia.launch
  ./run.sh bag /data/dataset.bag

Inside the container shell, you can:
  roslaunch fast_livo mapping_avia.launch
  rosbag play /catkin_ws/bags/your_file.bag
  catkin_make
EOF
}

case "${1:-help}" in
    build)      cmd_build ;;
    shell)      cmd_shell ;;
    launch)     shift; cmd_launch "$@" ;;
    rviz)       cmd_rviz ;;
    bag)        shift; cmd_bag "$@" ;;
    rebuild-ws) cmd_rebuild_ws ;;
    clean)      cmd_clean ;;
    help|*)     usage ;;
esac
