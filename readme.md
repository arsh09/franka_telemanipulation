
### Dependencies 

Requried are: 

1) Libfranka 0.7.0 (available in third_party folder as deb package) 
2) Boost 1.58 ( sudo apt-get install libboost1.58-all-dev )
3) Poco - Should be built (if required) as follow:

```bash
$ git clone -b master https://github.com/pocoproject/poco.git
$ cd poco
$ mkdir cmake-build
$ cd cmake-build
$ cmake ..
$ cmake --build . --config Release
$ sudo cmake --build . --target install
```

See more details on POCO github


#### How to run
```bash
$ git clone https://github.com/arsh09/franka_panda_experiments.git
$ cd franka_panda_experiements
$ mkdir build && cd build 
$ cmake .. 
$ cmkae --build .
```