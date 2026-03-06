#!/bin/bash
set -euo pipefail

# Find conda
CONDA_EXE=$(which conda)
if [ -z "$CONDA_EXE" ]; then
    echo "conda not found. Please ensure conda is in your PATH."
    exit 1
fi
echo "Using conda at $CONDA_EXE"

# Install/Update nerolib dependencies
echo "Updating nerolib environment..."
# Check if environment exists
if $CONDA_EXE info --envs | grep -q "nerolib"; then
    $CONDA_EXE env update -n nerolib -f environment.yml --prune
else
    $CONDA_EXE env create -f environment.yml
fi

# Get environment prefix
NEROLIB_CONDA_ENV=$($CONDA_EXE run -n nerolib printenv CONDA_PREFIX)
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
$CONDA_EXE run -n nerolib cmake -DCMAKE_INSTALL_PREFIX=$NEROLIB_CONDA_ENV ..
$CONDA_EXE run -n nerolib make -j
$CONDA_EXE run -n nerolib make install
cd ../..

# Install nerolib
echo "Installing nerolib..."
# Use conda run to ensure we use the environment's pip and set the env var
$CONDA_EXE run -n nerolib env NEROLIB_CONDA_ENV=$NEROLIB_CONDA_ENV pip install -e .

echo "Installation complete!"
