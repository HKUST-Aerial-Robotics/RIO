#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)

function build() {
    docker build \
    -t rio \
    -f $SCRIPT_DIR/Dockerfile \
    $SCRIPT_DIR/..
}

function run() {
    docker run -itd --rm \
    --network host \
    --privileged \
    -v /dev:/dev \
    -v $SCRIPT_DIR/../:/ws/src \
    rio \
    /bin/bash
}


function help() {
    echo "Usage: $0 [build|run]"
    exit 0
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -b|--build)
            build
            shift
            ;;
        -r|--run)
            run
            shift
            ;;
        -h|--help)
            help
            shift
            ;;
        *)
            echo "Unknown option: $1"
            help
            exit 1
            ;;
    esac
done