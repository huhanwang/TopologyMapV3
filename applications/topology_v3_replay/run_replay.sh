#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
config="${1:-${script_dir}/configs/autofused_1970_2090_v3_replay.json}"

cmake -S "${repo_root}" -B "${repo_root}/build"
cmake --build "${repo_root}/build" -j"$(nproc)"
"${repo_root}/build/applications/topology_v3_replay/topology_v3_replay" "${config}"
