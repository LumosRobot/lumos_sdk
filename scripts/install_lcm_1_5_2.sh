#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${CONDA_PREFIX:-}" ]]; then
    echo "Activate the lumos-sdk Conda environment first." >&2
    exit 1
fi

version="1.5.2"
archive="${TMPDIR:-/tmp}/lcm-${version}.tar.gz"
source_dir="${TMPDIR:-/tmp}/lcm-${version}-src"
build_dir="${TMPDIR:-/tmp}/lcm-${version}-build"
url="https://codeload.github.com/lcm-proj/lcm/tar.gz/refs/tags/v${version}"

curl -L --fail --retry 3 -o "${archive}" "${url}"
rm -rf "${source_dir}" "${build_dir}"
mkdir -p "${source_dir}"
tar -xzf "${archive}" --strip-components=1 -C "${source_dir}"

cmake -S "${source_dir}" -B "${build_dir}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${CONDA_PREFIX}" \
    -DPython_EXECUTABLE="${CONDA_PREFIX}/bin/python" \
    -DLCM_ENABLE_PYTHON=ON \
    -DLCM_ENABLE_JAVA=OFF \
    -DLCM_ENABLE_LUA=OFF \
    -DLCM_ENABLE_GO=OFF \
    -DLCM_ENABLE_TESTS=OFF \
    -DLCM_ENABLE_EXAMPLES=OFF
cmake --build "${build_dir}" -j"$(nproc)"
cmake --install "${build_dir}"

mkdir -p "${CONDA_PREFIX}/etc/conda/activate.d"
cat > "${CONDA_PREFIX}/etc/conda/activate.d/lcm.sh" <<EOF
export CMAKE_PREFIX_PATH="${CONDA_PREFIX}\${CMAKE_PREFIX_PATH:+:\${CMAKE_PREFIX_PATH}}"
export PKG_CONFIG_PATH="${CONDA_PREFIX}/lib/pkgconfig\${PKG_CONFIG_PATH:+:\${PKG_CONFIG_PATH}}"
export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}"
EOF

"${CONDA_PREFIX}/bin/lcm-gen" --version
"${CONDA_PREFIX}/bin/python" -c 'import lcm; print(lcm.__file__)'