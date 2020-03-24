# 增加可执行的二进制文件
ADD_EXECUTABLE(main main.cpp)

# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io )

# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )



# 添加可执行文件generatePointCloud.cpp
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
#ADD_EXECUTABLE( detectFeatures detectFeatures.cpp joinPointCloud.cpp)
#TARGET_LINK_LIBRARIES( detectFeatures
#        slambase
#        ${OpenCV_LIBS}
#        ${PCL_LIBRARIES} )

# 添加可执行文件joinPointCloud.cpp
ADD_EXECUTABLE( joinPointCloud joinPointCloud.cpp)
TARGET_LINK_LIBRARIES(joinPointCloud
        slambase
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES})


