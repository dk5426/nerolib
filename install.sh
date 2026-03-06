#!/bin/bash
set -euo pipefail

# Detect active environment
CURRENT_ENV=${CONDA_DEFAULT_ENV:-"base"}
TARGET_ENV="nerolib"

# If we are in an active environment that isn't 'base', use it
if [ "$CURRENT_ENV" != "base" ]; then
    echo "Active environment '$CURRENT_ENV' detected. Installing into '$CURRENT_ENV'..."
    TARGET_ENV=$CURRENT_ENV
else
    echo "No active environment detected (base). Defaulting to '$TARGET_ENV'..."
fi

# Find mamba or conda
MAMBA_EXE=$(which mamba || which micromamba || true)
CONDA_EXE=$(which conda || true)

if [ -n "$MAMBA_EXE" ]; then
    SOLVER_EXE=$MAMBA_EXE
    echo "Using mamba/micromamba at $SOLVER_EXE"
elif [ -n "$CONDA_EXE" ]; then
    SOLVER_EXE=$CONDA_EXE
    echo "Using conda at $SOLVER_EXE"
    # Try to enable libmamba solver for conda if possible
    if $CONDA_EXE config --show-sources | grep -q "solver: libmamba"; then
        echo "libmamba solver already enabled for conda."
    else
        echo "Attempting to use libmamba solver for faster solving..."
        $CONDA_EXE install -n base conda-libmamba-solver -y || echo "Could not install libmamba solver, falling back to default."
        $CONDA_EXE config --set solver libmamba || echo "Could not set libmamba solver."
    fi
else
    echo "Neither mamba nor conda found. Please ensure they are in your PATH."
    exit 1
fi

# Install/Update dependencies
echo "Updating environment '$TARGET_ENV'..."
if $SOLVER_EXE info --envs | grep -q "$TARGET_ENV"; then
    $SOLVER_EXE env update -n "$TARGET_ENV" -f environment.yml --prune
else
    $SOLVER_EXE env create -f environment.yml -n "$TARGET_ENV"
fi

RUN_CMD="$SOLVER_EXE run -n $TARGET_ENV"

# Get environment prefix
NEROLIB_CONDA_ENV=$($RUN_CMD printenv CONDA_PREFIX)
echo "NEROLIB_CONDA_ENV=$NEROLIB_CONDA_ENV"

# Install ruckig
echo "Installing ruckig..."
if [ ! -d "ruckig" ]; then
    echo "ruckig directory not found!"
    exit 1
fi

cd ruckig
mkdir -p build && cd build
# Install to conda environment prefix to avoid sudo
$RUN_CMD cmake -DCMAKE_INSTALL_PREFIX=$NEROLIB_CONDA_ENV ..
$RUN_CMD make -j
$RUN_CMD make install
cd ../..

# Install nerolib
echo "Installing nerolib..."
# Use RUN_CMD to ensure we use the environment's pip and set the env var
$RUN_CMD env NEROLIB_CONDA_ENV=$NEROLIB_CONDA_ENV pip install -e .

echo "Installation complete!"
