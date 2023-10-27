#!/usr/bin/env bash
TAB="    " # 4 Spaces
DOCKERFILE=$1

function print_usage() {
    local prog="$(basename $0)"
    echo "Usage:"
    echo "${TAB}${prog} -f <Dockerfile> [Options]"
    echo "Available options:"
    echo "${TAB}-h,--help       Show this message and exit"
    echo "E.g.,"
    echo "${TAB}${prog} -f ros2.galactic.x86_64.dockerfile"
}

function check_opt_arg() {
    local opt="$1"
    local arg="$2"
    if [[ -z "${arg}" || "${arg}" =~ ^-.* ]]; then
        echo "Argument missing for option ${opt}. Exiting..."
        exit 1
    fi
}

function parse_arguments() {
    if [[ $# -eq 0 ]] || [[ "$1" == "--help" ]]; then
        print_usage
        exit 0
    fi
    while [[ $# -gt 0 ]]; do
        local opt="$1"
        shift
        case $opt in
            -f|--dockerfile)
                check_opt_arg "${opt}" "$1"
                DOCKERFILE="$1"
                shift
                ;;
            -h|--help)
                print_usage
                exit 0
                ;;
            *)
                echo "Unknown option: ${opt}"
                print_usage
                exit 1
                ;;
        esac
    done
}

TAG=ros2_galactic_open_robotics

function docker_build_run() {
    local extra_args=""
    if [[ "${USE_CACHE}" -eq 0 ]]; then
        extra_args="${extra_args} --no-cache=true"
    fi

    local context="$(dirname "${BASH_SOURCE[0]}")"
    
    set -x
    docker build --network=host ${extra_args} -t "${TAG}" \
            ${build_args} \
            -f "${DOCKERFILE}" \
            "${context}"
    set +x
}

function main() {
    parse_arguments "$@"
    docker_build_run
    echo "Built new image ${TAG}"
}

main "$@"
