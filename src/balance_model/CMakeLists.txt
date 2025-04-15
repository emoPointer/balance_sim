cmake_minimum_required(VERSION 3.5)
project(balance_model)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED) 

set(RESOURCE_DIRS
    config
    launch
    meshes
    urdf
)

foreach(dir ${RESOURCE_DIRS})
    install(
        DIRECTORY ${dir}/
        DESTINATION share/${PROJECT_NAME}/${dir}
    )
endforeach()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(urdf)

ament_package()