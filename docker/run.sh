#!/bin/bash
RUN_DIR=$(dirname $(readlink -f $0))
PKG_DIR=$(dirname ${RUN_DIR})

DISTRO_DEFAULT='melodic'

DISTRO=${DISTRO_DEFAULT}

function usage_exit {
  cat <<_EOS_ 1>&2
  Usage: run.sh [OPTIONS...]
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

ip_list=($(hostname -I))
if [[ ${#ip_list[@]} -eq 1 ]]; then
    ROS_IP="${ip_list[0]}"
else
    echo -e "番号\tIPアドレス"
    cnt=0
    for ip in "${ip_list[@]}"; do
        echo -e "${cnt}:\t${ip}"
        cnt=$((${cnt}+1))
    done
    isnum=3
    ip_num=-1
    while [[ ${isnum} -ge 2 ]] || [[ ${ip_num} -ge ${cnt} ]] || [[ ${ip_num} -lt 0 ]]; do
        read -p "使用するIPアドレスの番号を入力してください: " ip_num
        expr ${ip_num} + 1 > /dev/null 2>&1
        isnum=$?
    done
    ROS_IP="${ip_list[${ip_num}]}"
fi

DOCKER_ENV="${DOCKER_ENV} -e ROS_IP=${ROS_IP}"
DOCKER_VOLUME="${DOCKER_VOLUME} -v ${PKG_DIR}:/workspace/src/pointsmap_renderer:rw"

docker run \
    -it \
    --rm \
    --net host \
    ${DOCKER_VOLUME} \
    ${DOCKER_ENV} \
    --name ros-pointsmap_renderer \
    shikishimatasakilab/pointsmap_renderer:${DISTRO}
