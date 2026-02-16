#!/bin/bash
# System design format linting wrapper for pre-commit in autoware_universe
# Handles only system design format files (*.node.yaml, *.module.yaml, *.system.yaml, *.parameter_set.yaml)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"

# Try to find local core package first (development mode)
CORE_PACKAGE_DIR="${SCRIPT_DIR}/../../core/autoware_system_designer/autoware_system_designer"

if [ -d "${CORE_PACKAGE_DIR}" ]; then
    export PYTHONPATH="${CORE_PACKAGE_DIR}:${PYTHONPATH}"
else
    # Fallback to external repository clone
    LINTER_REPO="https://github.com/autowarefoundation/autoware_system_designer.git"
    CACHE_DIR="${HOME}/.cache/autoware/autoware_system_designer_linter"

    if [ ! -d "${CACHE_DIR}" ]; then
        echo "Cloning autoware_system_designer linter from ${LINTER_REPO}..."
        mkdir -p "$(dirname "${CACHE_DIR}")"
        git clone --depth 1 "${LINTER_REPO}" "${CACHE_DIR}"
    else
        # Optional: update the repo
        # git -C "${CACHE_DIR}" pull --rebase --autostash >/dev/null 2>&1 || true
        :
    fi

    # The package is located in the root of the repo or inside a subdir
    if [ -d "${CACHE_DIR}/autoware_system_designer" ]; then
        export PYTHONPATH="${CACHE_DIR}/autoware_system_designer:${PYTHONPATH}"
    else
        export PYTHONPATH="${CACHE_DIR}:${PYTHONPATH}"
    fi
fi

# Process each file passed by pre-commit (only system design format files)
EXIT_CODE=0
for file in "$@"; do
    # Convert to absolute path if relative
    if [[ $file != /* ]]; then
        file="${REPO_ROOT}/${file}"
    fi

    # Run the linter using the source code
    python3 -m autoware_system_designer.linter.run_lint --format github-actions "$file" || EXIT_CODE=1
done

exit $EXIT_CODE
