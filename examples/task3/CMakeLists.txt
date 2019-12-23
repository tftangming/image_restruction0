project(class3)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-fPIC")
#将ImageBasedModellingEduV1.0包含到路径中
include_directories("../..")

# test triangle
set(TRIANGLE_FILE
        class3_test_triangle.cc)
add_executable(task3_test_triangle ${TRIANGLE_FILE})
target_link_libraries(task3_test_triangle sfm util core)

##test p3p
#set(P3P_FILE
#        class3_test_p3p.cc)
#add_executable(task3_test_p3p ${P3P_FILE})
#target_link_libraries(task3_test_p3p sfm util core features)


#test jacobian
set(JACOBIAN_FILE class3_test_jacobian.cc)
add_executable(task3_test_jacobian ${JACOBIAN_FILE})
target_link_libraries(task3_test_jacobian sfm util core)


# test bundle adjustment
set(BA_FILES
        class3_test_bundle_adjustment.cc)
add_executable(task3_test_ba ${BA_FILES})
target_link_libraries(task3_test_ba sfm util core features)

# test load_image_exif
set(BA_FILES
        test.cc)
add_executable(test_image_exif ${BA_FILES})
target_link_libraries(test_image_exif sfm util core features)

#test lm optimize
set(LM_OPTIMIZE_FILES
        class3_test_lm_optimize.cc)
add_executable(task3_test_lm_optimize ${LM_OPTIMIZE_FILES})
target_link_libraries(task3_test_lm_optimize sfm util core features)
