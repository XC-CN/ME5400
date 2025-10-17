#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
exec "$SCRIPT_DIR/run_fastlio.sh" mapping "$@"
