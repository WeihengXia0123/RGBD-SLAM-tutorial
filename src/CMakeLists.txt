# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 添加g2o的依赖
# 因为g2o不是常用的库，所以要手动添加它的findg2o.cmake文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
SET(G2O_ROOT /usr/local/include/g2o)
FIND_PACKAGE(g2o)
# CSparse
FIND_PACKAGE(CSparse)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})


# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# 添加可执行文件
ADD_EXECUTABLE( generate_pointcloud generatePointCloud.cpp )
TARGET_LINK_LIBRARIES( generate_pointcloud ${OpenCV_LIBS}
        ${PCL_LIBRARIES} )

# 添加自己写的slamBase库,将slamBase.cpp编译成库
ADD_LIBRARY( slambase slamBase.cpp)
TARGET_LINK_LIBRARIES(
        slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

# 添加可执行文件detectFeatures.cpp
# ADD_EXECUTABLE( detectFeatures detectFeatures.cpp joinPointCloud.cpp)
# TARGET_LINK_LIBRARIES( detectFeatures
#        slambase
#        ${OpenCV_LIBS}
#        ${PCL_LIBRARIES} )

# 添加可执行文件
ADD_EXECUTABLE( joinPointCloud joinPointCloud.cpp)
TARGET_LINK_LIBRARIES(joinPointCloud
        slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES})

# 添加可执行文件
ADD_EXECUTABLE( visualOdometry visualOdometry.cpp)
TARGET_LINK_LIBRARIES( visualOdometry
        slambase
        ${OpenCV_LIBS} 
        ${PCL_LIBRARIES} )


# 添加可执行文件 
ADD_EXECUTABLE( slamEnd slamEnd.cpp )
TARGET_LINK_LIBRARIES( slamEnd
    slambase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    g2o_core g2o_types_slam3d 
    g2o_solver_csparse g2o_stuff 
    g2o_csparse_extension ${CSPARSE_LIBRARY})

# 添加可执行文件 
ADD_EXECUTABLE( slam slam.cpp )
TARGET_LINK_LIBRARIES( slam
    slambase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    g2o_core g2o_types_slam3d 
    g2o_solver_csparse g2o_stuff 
    g2o_csparse_extension ${CSPARSE_LIBRARY})

    # 增加可执行的二进制文件
ADD_EXECUTABLE(main main.cpp)
TARGET_LINK_LIBRARIES( main
        slambase
        ${OpenCV_LIBS} 
        ${PCL_LIBRARIES} )