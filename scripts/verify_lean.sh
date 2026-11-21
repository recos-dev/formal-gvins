#!/usr/bin/env bash
set -euo pipefail

REPO_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
LAKE=${LAKE:-lake}

main()
{
    if [[ $# -gt 1 || ( $# -eq 1 && $1 != --cache ) ]]; then
        printf 'Usage: %s [--cache]\n' "$0" >&2
        exit 2
    fi
    if ! command -v "$LAKE" >/dev/null 2>&1; then
        printf 'error: lake was not found; install the pinned Lean toolchain through Elan or set LAKE\n' >&2
        exit 1
    fi
    cd "$REPO_DIR/formal/lean"
    if [[ ${1:-} == --cache ]]; then
        "$LAKE" exe cache get
    fi
    "$LAKE" --wfail build
}

main "$@"
