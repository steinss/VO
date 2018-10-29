add_library( slam_one SHARED
        config.cpp
        carera_kitti.cpp
        frame_kitti.cpp
        bundle_adjustment.cpp
        map.cpp
        vo_kitti.cpp
        mappoint.cpp
        read_imglist.cpp)

target_link_libraries( slam_one ${THIRED_LIBS} )