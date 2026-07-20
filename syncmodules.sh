#!/usr/bin/env bash
# Run with: . sync-dependencies.sh

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "ERROR: must be sourced, not executed. Use: . ${0}" >&2
    exit 1
fi

if ! command -v uv >/dev/null 2>&1; then
    echo "'uv' not found. Install now? (Y/n)"
    read -r answer
    if [[ "$answer" =~ ^[Yy]?$ ]]; then
        echo "Installing 'uv'..."
        curl -LsSf https://astral.sh/uv/install.sh | sh || { echo "ERROR: uv install failed" >&2; return 1; }
        source "$HOME/.local/bin/env" 2>/dev/null || export PATH="$HOME/.local/bin:$PATH"
        command -v uv >/dev/null 2>&1 || { echo "ERROR: uv not found in PATH after install" >&2; return 1; }
    else
        echo "Installation aborted by user. Please install 'uv' and rerun the script." >&2
        return 1
    fi
fi

echo "Syncing submodules..."
git submodule sync && git submodule update --init --recursive gnn dataset-generator || { echo "ERROR: submodule update failed" >&2; return 1; }

echo "Installing gnn dependencies..."
( cd gnn && source install_dependencies.sh ) || { echo "ERROR: gnn 'install_dependencies.sh' script failed" >&2; return 1; }

echo "Installing dataset-generator dependencies..."
( cd dataset-generator && uv sync ) || { echo "ERROR: dataset-generator uv sync failed" >&2; return 1; }

echo " ✅ Done. You can use the S-LDM now."