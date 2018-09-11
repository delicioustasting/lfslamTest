#ADD_EXECUTABLE( rgbd2points rgbd2points.cpp)
#TARGET_LINK_LIBRARIES( rgbd2points ${OpenCV_LIBS} ${PCL_LIBRARIES})

#ADD_EXECUTABLE( colorDepthMap colorDepthMap.cpp)
#TARGET_LINK_LIBRARIES( colorDepthMap ${OpenCV_LIBS} ${PCL_LIBRARIES})

ADD_LIBRARY( slamBase slamBase.cpp)
TARGET_LINK_LIBRARIES( slamBase ${OpenCV_LIBS} ${PCL_LIBRARIES})

ADD_LIBRARY( feature_extract feature_extract.cpp)
TARGET_LINK_LIBRARIES( feature_extract ${OpenCV_LIBS} ${PCL_LIBRARIES})


ADD_EXECUTABLE( lfslamVO lfslamVO.cpp)
TARGET_LINK_LIBRARIES( lfslamVO slamBase feature_extract ${OpenCV_LIBS} ${PCL_LIBRARIES})
