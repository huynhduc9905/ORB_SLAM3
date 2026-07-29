#!/usr/bin/env bash
set -e

echo "Configuring and building Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j$(nproc)

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j$(nproc)

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."
rm -rf build
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_CXX_FLAGS="-Wno-error"
make -j$(nproc)

cd ../../../

echo "Uncompress vocabulary ..."
cd Vocabulary
if [ ! -f "ORBvoc.txt" ]; then
    tar -xf ORBvoc.txt.tar.gz
fi
cd ..

echo "Configuring and building ORB_SLAM3 ..."
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j4
