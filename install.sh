#!/bin/bash
# Nerolib installer.
#
# Works on a stock Ubuntu machine (x86_64 or aarch64) with nothing but curl:
# it will bootstrap Miniforge if no conda/mamba is present, create the
# `nerolib` environment, build and install the vendored Ruckig into that
# environment, and finally install the Nerolib Python package.
#
#   ./install.sh              # interactive
#   ./install.sh --yes        # never prompt (CI / provisioning)
#   ./install.sh --env myenv  # install into a differently named environment
#
set -euo pipefail

ASSUME_YES=0
TARGET_ENV=""

while [ $# -gt 0 ]; do
    case "$1" in
        -y|--yes)   ASSUME_YES=1; shift ;;
        --env)      TARGET_ENV="${2:-}"; shift 2 ;;
        -h|--help)
            sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *) echo "Unknown argument: $1" >&2; exit 1 ;;
    esac
done

confirm() {
    # confirm <prompt>  -> 0 if the user agreed (or --yes was passed)
    [ "$ASSUME_YES" -eq 1 ] && return 0
    read -r -p "$1 [Y/n] " reply
    case "$reply" in [nN]*) return 1 ;; *) return 0 ;; esac
}

if [ "$(uname -s)" != "Linux" ]; then
    echo "WARNING: Nerolib talks to the arm over SocketCAN, which is Linux-only."
    echo "         The library will build here but cannot drive hardware."
    confirm "Continue anyway?" || exit 1
fi

# ---------------------------------------------------------------------------
# 1. Locate (or install) a conda/mamba
# ---------------------------------------------------------------------------

MAMBA_EXE=$(command -v mamba || command -v micromamba || true)
CONDA_EXE=$(command -v conda || true)

if [ -z "$MAMBA_EXE" ] && [ -z "$CONDA_EXE" ]; then
    # Pick up an installation that exists but was never added to PATH.
    for candidate in "$HOME/miniforge3" "$HOME/mambaforge" "$HOME/miniconda3" "$HOME/anaconda3"; do
        if [ -x "$candidate/bin/conda" ]; then
            echo "Found an existing installation at $candidate that is not on your PATH."
            # shellcheck disable=SC1091
            source "$candidate/etc/profile.d/conda.sh"
            [ -f "$candidate/etc/profile.d/mamba.sh" ] && source "$candidate/etc/profile.d/mamba.sh"
            MAMBA_EXE=$(command -v mamba || command -v micromamba || true)
            CONDA_EXE=$(command -v conda || true)
            break
        fi
    done
fi

if [ -z "$MAMBA_EXE" ] && [ -z "$CONDA_EXE" ]; then
    echo "No conda or mamba found."
    echo "Nerolib needs Pinocchio, spdlog and a matching C++ toolchain, which come from conda-forge."
    if ! confirm "Install Miniforge into \$HOME/miniforge3 now?"; then
        echo "Aborting. Install Miniforge yourself and re-run: https://github.com/conda-forge/miniforge"
        exit 1
    fi

    command -v curl >/dev/null 2>&1 || {
        echo "curl is required to download Miniforge. Install it with: sudo apt-get install -y curl"
        exit 1
    }

    MINIFORGE_URL="https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
    echo "Downloading $MINIFORGE_URL"
    curl -fsSL "$MINIFORGE_URL" -o /tmp/miniforge.sh
    bash /tmp/miniforge.sh -b -p "$HOME/miniforge3"
    rm -f /tmp/miniforge.sh

    # shellcheck disable=SC1091
    source "$HOME/miniforge3/etc/profile.d/conda.sh"
    [ -f "$HOME/miniforge3/etc/profile.d/mamba.sh" ] && source "$HOME/miniforge3/etc/profile.d/mamba.sh"
    MAMBA_EXE=$(command -v mamba || true)
    CONDA_EXE=$(command -v conda || true)
    echo
    echo "Miniforge installed. Run '$HOME/miniforge3/bin/conda init' afterwards to"
    echo "get conda on your PATH in new shells."
    echo
fi

if [ -n "$MAMBA_EXE" ]; then
    SOLVER_EXE=$MAMBA_EXE
    echo "Using mamba/micromamba at $SOLVER_EXE"
else
    SOLVER_EXE=$CONDA_EXE
    echo "Using conda at $SOLVER_EXE"
    # Enable the libmamba solver: the default solver is painfully slow on the
    # constrained hardware these arms usually run on.
    if $CONDA_EXE config --show-sources | grep -q "solver: libmamba"; then
        echo "libmamba solver already enabled."
    else
        echo "Attempting to enable libmamba solver..."
        $CONDA_EXE install -n base conda-libmamba-solver -y || echo "Could not install libmamba solver, falling back to default."
        $CONDA_EXE config --set solver libmamba || echo "Could not set libmamba solver."
    fi
fi

# ---------------------------------------------------------------------------
# 2. Choose the target environment
# ---------------------------------------------------------------------------

if [ -z "$TARGET_ENV" ]; then
    CURRENT_ENV=${CONDA_DEFAULT_ENV:-"base"}
    if [ "$CURRENT_ENV" != "base" ]; then
        echo "Active environment '$CURRENT_ENV' detected. Installing into '$CURRENT_ENV'..."
        TARGET_ENV=$CURRENT_ENV
    else
        TARGET_ENV="nerolib"
        echo "No active environment detected (base). Defaulting to '$TARGET_ENV'..."
    fi
fi

# Install/Update dependencies into target environment.
# Use CONDA_ALWAYS_YES instead of -y because some conda versions do not
# recognise the -y flag on the env subcommands.
echo "Updating environment '$TARGET_ENV'..."
if $SOLVER_EXE info --envs | grep -qE "^$TARGET_ENV[[:space:]]"; then
    CONDA_ALWAYS_YES=true $SOLVER_EXE env update -n "$TARGET_ENV" -f environment.yml --prune
else
    CONDA_ALWAYS_YES=true $SOLVER_EXE env create -f environment.yml -n "$TARGET_ENV"
fi

# Get environment prefix: prefer CONDA_PREFIX (set by `conda run`) so we
# always target the correct env even when two envs share the same name --
# but only when it actually belongs to the environment we are installing into.
if [ -n "${CONDA_PREFIX:-}" ] && [ "${CONDA_DEFAULT_ENV:-}" = "$TARGET_ENV" ]; then
    ENV_PREFIX="$CONDA_PREFIX"
    echo "Using active CONDA_PREFIX: $ENV_PREFIX"
else
    ENV_PREFIX=$($SOLVER_EXE info --envs | grep -E "^$TARGET_ENV[[:space:]]" | awk '{print $NF}' | head -n1)
    if [ -z "$ENV_PREFIX" ]; then
        # Fallback: look for active env path
        ENV_PREFIX=$($SOLVER_EXE info --envs | grep "\*" | awk '{print $NF}')
    fi
fi

if [ -z "$ENV_PREFIX" ] || [ ! -d "$ENV_PREFIX" ]; then
    echo "ERROR: Could not resolve a prefix for environment '$TARGET_ENV'."
    exit 1
fi
echo "ENV_PREFIX=$ENV_PREFIX"

# ---------------------------------------------------------------------------
# 3. Locate the toolchain inside the environment
# ---------------------------------------------------------------------------

# conda-forge names these <arch>-conda-linux-gnu-gcc, so this works on both
# x86_64 and aarch64 without hardcoding either.
C_COMPILER=$(ls "$ENV_PREFIX/bin/"*-gcc 2>/dev/null | head -n1 || echo "")
CXX_COMPILER=$(ls "$ENV_PREFIX/bin/"*-g++ 2>/dev/null | head -n1 || echo "")

if [ -z "$C_COMPILER" ] || [ -z "$CXX_COMPILER" ]; then
    echo "ERROR: Could not find gcc/g++ in $ENV_PREFIX/bin/"
    echo "       environment.yml installs these via the 'gxx' and 'cxx-compiler'"
    echo "       packages -- check that the environment solve actually succeeded."
    exit 1
fi

echo "Using C compiler:   $C_COMPILER"
echo "Using C++ compiler: $CXX_COMPILER"

# All binaries in the environment prefix
PIP="$ENV_PREFIX/bin/pip"
CMAKE="$ENV_PREFIX/bin/cmake"

for tool in "$PIP" "$CMAKE"; do
    [ -x "$tool" ] || { echo "ERROR: $tool missing from the environment."; exit 1; }
done

# ---------------------------------------------------------------------------
# 4. Build and install the vendored Ruckig
# ---------------------------------------------------------------------------

echo "Installing ruckig..."
if [ ! -d "ruckig" ]; then
    echo "ruckig directory not found!"
    echo "It is vendored in this repo -- run install.sh from the repository root."
    exit 1
fi

NPROC=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 2)

cd ruckig
mkdir -p build && cd build
# Build using environment compilers and install into environment prefix
unset CC CXX  # clear any inherited compiler vars
"$CMAKE" -DCMAKE_INSTALL_PREFIX="$ENV_PREFIX" \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_C_COMPILER="$C_COMPILER" \
         -DCMAKE_CXX_COMPILER="$CXX_COMPILER" ..
make -j"$NPROC"
# Use cmake --install to explicitly enforce the prefix (avoids /usr/local fallback)
"$CMAKE" --install . --prefix "$ENV_PREFIX"
cd ../..

# ---------------------------------------------------------------------------
# 5. Install the Nerolib Python package
# ---------------------------------------------------------------------------

echo "Installing nerolib..."
NEROLIB_CONDA_ENV="$ENV_PREFIX" "$PIP" install -e .

# ---------------------------------------------------------------------------
# 6. Verify and report
# ---------------------------------------------------------------------------

echo "Verifying import..."
if "$ENV_PREFIX/bin/python" -c "import nerolib; print('nerolib', nerolib.__version__, 'OK')"; then
    echo
    echo "Installation complete!"
else
    echo "ERROR: nerolib built but could not be imported." >&2
    exit 1
fi

cat <<EOF

Next steps
----------
  1. Activate the environment:
         conda activate $TARGET_ENV

  2. Bring up the CAN interface the arm is on (1 Mbit/s):
         sudo ip link set can0 up type can bitrate 1000000
         ip -details link show can0

     'can-utils' is handy for checking traffic:
         sudo apt-get install -y can-utils && candump can0

  3. Set ControllerConfig.firmware_version to match your arm before
     connecting -- see the Firmware Version section of README.md.
EOF
