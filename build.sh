# echo "Configuring and building Thirdparty/DBoW2 ..."
###
 # @Author: wmx
 # @Date: 2022-04-25 09:20:32
 # @LastEditTime: 2022-04-25 09:33:14
 # @LastEditors: wmx
 # @Description: 
 # @FilePath: /ORB-SLAM2-DenseMap/build.sh
### 

# cd Thirdparty/DBoW2
# # rm -r build
# # mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

# cd ../../g2o

# echo "Configuring and building Thirdparty/g2o ..."
# # rm -r build
# # mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j

# cd ../../../

echo "Configuring and building ORB_SLAM2 ..."

rm -r build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make 

cd ..

echo "Converting vocabulary to binary"
./tools/bin_vocabulary
echo "Converting vocabulary to binary finished********"
