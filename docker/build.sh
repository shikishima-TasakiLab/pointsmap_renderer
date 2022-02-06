#!/bin/bash
RUN_DIR=$(dirname $(readlink -f $0))
DISTRO_DEFAULT='melodic'

DISTRO=${DISTRO_DEFAULT}

function usage_exit {
  cat <<_EOS_ 1>&2
  Usage: build.sh [OPTIONS...]
  OPTIONS:
    -h, --help          Show this help
    -d, --distro        ROS distro. (default: ${DISTRO_DEFAULT})
_EOS_
  exit 1
}

while (( $# > 0 )); do
  if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
    usage_exit
  elif [[ $1 == "-d" ]] || [[ $1 == "--distro" ]]; then
    if [[ $2 == -* ]]; then
      echo "Invalid parameter"
      usage_exit
    fi
    DISTRO=$2
    shift 2
  else
    echo "Invalid parameter: $1"
    usage_exit
  fi
done

docker build \
    -t shikishimatasakilab/pointsmap_renderer:${DISTRO} \
    --build-arg DISTRO=${DISTRO} \
    -f ${RUN_DIR}/Dockerfile.ros1 \
    ${RUN_DIR}
