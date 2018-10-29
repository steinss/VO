

add_executable( lk_test lk_test.cpp )
target_link_libraries( lk_test slam_one )

add_executable( compare_pose compare_pose.cpp )
target_link_libraries( compare_pose slam_one )