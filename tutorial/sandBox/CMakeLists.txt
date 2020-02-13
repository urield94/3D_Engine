get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})

include_directories(/usr/include)
link_directories(/usr/lib)

if(TARGET igl_opengl)
    if(NOT TARGET stb_image)
        igl_download_stb()
        add_subdirectory(${LIBIGL_EXTERNAL}/stb stb_image)
    endif()
    compile_igl_module("png" "")
    target_link_libraries(igl_png ${IGL_SCOPE} igl_stb_image igl_opengl)
endif()

add_executable(${PROJECT_NAME}_bin main.cpp inputManager.h DB.cpp DB.h)
target_link_libraries(${PROJECT_NAME}_bin igl::core igl::opengl igl::opengl_glfw igl::png sqlite3)
