echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j30

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j30

cd ../../Line3Dpp

echo "Configuring and building Thirdparty/Line3Dpp ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j30

cd ../../../

# 可以将离线数据进行压缩，此时需要对txt进行解码
# echo "Uncompress yolo_txts ..."
# cd ../../../data
# tar -xf yolo_txts.tar.gz
# cd ..

# echo "Uncompress vocabulary ..."

# cd Vocabulary
# tar -xf ORBvoc.txt.tar.gz
# cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j30

# cd ..
# echo "Converting vocabulary to binary"
# ./tools/bin_vocabulary
