#! /bin/bash

echo "-- start building..."

if [[ $1 == "a" || $1 == "-a" ]];then
    echo "-- 全量编译..."
    rm -rf cmake
    mkdir cmake && cd cmake
    cmake .. && make dbscan_bin
else
    echo "-- 增量编译..."
    cd cmake && make dbscan_bin
fi
