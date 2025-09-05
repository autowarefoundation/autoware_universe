#!/bin/bash
set -e

LIBTORCH_DIR="third_party"

# clone repository if folder does not exist
if [ ! -d "$LIBTORCH_DIR/libtorch" ]; then
  if [ ! -d "$LIBTORCH_DIR/pytorch" ]; then
    echo "cloning pytorch..."
    # clone pytorch source code
    # cmake version must be <= 3.22
    git clone -b v2.7.1 --recurse-submodule https://github.com/pytorch/pytorch.git $LIBTORCH_DIR/pytorch
  fi

  cd $LIBTORCH_DIR/pytorch

  if [ ! -d "build" ]; then
    mkdir build
  fi

  cd build

  echo "building pytorch..."
  cmake ../ \
    -DCMAKE_INSTALL_PREFIX=../../libtorch \
    -DBUILD_SHARED_LIBS:BOOL=ON \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_EXPORT_PACKAGE_REGISTRY=ON \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_CAFFE2=ON \
    -DUSE_CUDA=ON \
    -DUSE_MKLDNN=ON \
    -DBUILD_PYTHON=OFF \
    -DUSE_DISTRIBUTED=OFF \
    -DUSE_NCCL=OFF \
    -DUSE_CUFILE=OFF \
    -USE_FLASH_ATTENTION:BOOL=OFF
  make -j$(nproc) # this may take large RAM space
  # install to LIBTORCH_DIR/libtorch
  make install

else
  echo "liborch directory already exists, skip building"
fi

echo "libtorch setup completed."
