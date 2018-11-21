#!/bin/bash

source $(dirname $0)/_env.sh
export RUSTC_WRAPPER=$(dirname $0)/_rustc_wrapper.sh
set -x

xargo doc --target $BUILD_TARGET --all "$@"