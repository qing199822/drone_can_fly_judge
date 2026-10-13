#!/bin/bash
set -e

if [ "$#" -lt 2 ]; then
    echo "usage: $0 <delay_seconds> <spawn_model_args...>" >&2
    exit 2
fi

delay="$1"
shift

exec flock -x /tmp/zzu_fly_spawn_models.lock bash -c '
    sleep "$1"
    shift
    exec "$@"
' _ "$delay" "$@"
