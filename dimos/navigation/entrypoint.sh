#!/bin/bash
# Compatibility wrapper: keep a single source of truth for startup logic.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/dimos_module_entrypoint.sh" "$@"
