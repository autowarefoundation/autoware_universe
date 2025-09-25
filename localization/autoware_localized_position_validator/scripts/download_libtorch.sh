#!/bin/bash
set -e

# this might expire in the future
# libtorch cuda 11.8
LIBTORCH_DOWNLOAD_URL="https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcu118.zip"
# libtorch cuda 12.6
#LIBTORCH_DOWNLOAD_URL="https://download.pytorch.org/libtorch/cu126/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcu126.zip"
# libtorch cuda 12.8
#LIBTORCH_DOWNLOAD_URL="https://download.pytorch.org/libtorch/cu126/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcu128.zip"

ZIP_NAME="libtorch.zip"
LIBTORCH_DIR="third_party"

# download zip if it doesn't exist
if [ ! -f "$ZIP_NAME" ] && [ ! -d "$LIBTORCH_DIR" ]; then
  echo "Downloading libtorch..."
  curl -sSL "$LIBTORCH_DOWNLOAD_URL" -o "$ZIP_NAME"
else
  echo "zip file already downloaded: $ZIP_NAME"
fi

# Unzip if not already extracted
if [ ! -d "$LIBTORCH_DIR" ]; then
  echo "Extracting libtorch..."
  unzip -q "$ZIP_NAME" -d "$LIBTORCH_DIR"

  if [ -f "$ZIP_NAME" ]; then
    rm "$ZIP_NAME"
  fi
else
  echo "libtorch directory already exists, skipping extraction."
fi

echo "libtorch setup completed."
