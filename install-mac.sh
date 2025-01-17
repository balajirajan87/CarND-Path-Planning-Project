#! /bin/bash
brew install openssl libuv cmake
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
cd build
#!OPENSSL_VERSION=`openssl version -v | cut -d' ' -f2`
OPENSSL_VERSION="1.0.2p"
cmake -DOPENSSL_ROOT_DIR=/usr/local/Cellar/openssl@3/3.0.1 -DOPENSSL_LIBRARIES=/usr/local/Cellar/openssl@3/3.0.1/lib ..
make 
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
