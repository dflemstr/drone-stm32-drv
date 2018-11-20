#!/bin/bash

source $(dirname $0)/_env.sh
export RUSTC_WRAPPER=$(dirname $0)/_clippy_wrapper.sh
set -x

xargo check --target $BUILD_TARGET --all "$@"
