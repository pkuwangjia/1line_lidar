SRC_DIR=$(dirname $(readlink -f "$0"))
INSTALL_DIR="$SRC_DIR/../opt/"

mkdir build
cd build 
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR ..
make -j8 install
