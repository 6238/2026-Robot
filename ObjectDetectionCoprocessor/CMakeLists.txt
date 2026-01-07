cmake_minimum_required(VERSION 3.12)
project(rfdetr_inference CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# === DEBUG BUILD ===
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g -Og")

option(USE_TENSORRT "Build with TensorRT" ON)

# --- CUDA ---
find_package(CUDAToolkit REQUIRED)

# --- OpenCV (Manual Mode) ---
set(OpenCV_INCLUDE_DIRS "/usr/local/include/opencv4")
set(OPENCV_LIBS_TO_FIND core imgproc imgcodecs videoio highgui calib3d features2d flann ml objdetect photo stitching video gapi cudev cudaarithm cudafilters cudaimgproc cudawarping)
foreach(lib ${OPENCV_LIBS_TO_FIND})
    find_library(LIB_opencv_${lib} NAMES opencv_${lib} PATHS "/usr/local/lib" NO_DEFAULT_PATH REQUIRED)
    list(APPEND OpenCV_LIBRARIES ${LIB_opencv_${lib}})
endforeach()

# --- TensorRT ---
if(USE_TENSORRT)
    set(ARCH_DIR "aarch64-linux-gnu")
    set(TENSORRT_INCLUDE_DIR "/usr/include/${ARCH_DIR}")
    set(TENSORRT_LIBRARY "/usr/lib/${ARCH_DIR}/libnvinfer.so")
    set(TENSORRT_ONNX_PARSER "/usr/lib/${ARCH_DIR}/libnvonnxparser.so")
endif()

# --- Jetson Link Paths ---
set(JETSON_LINK_PATHS 
    "/usr/lib/aarch64-linux-gnu/nvidia" 
    "/usr/lib/aarch64-linux-gnu/tegra" 
    "/usr/lib/aarch64-linux-gnu" 
    "/lib/aarch64-linux-gnu"
    "/usr/local/cuda/targets/aarch64-linux/lib" 
    "/usr/local/cuda/lib64"
    "/usr/local/lib"
)

# --- Fetch Google Test ---
include(FetchContent)
FetchContent_Declare(googletest GIT_REPOSITORY https://github.com/google/googletest.git GIT_TAG release-1.12.1)
FetchContent_MakeAvailable(googletest)

# --- Source setup ---
set(SOURCE_DIR "${CMAKE_SOURCE_DIR}/src")
set(TEST_DIR "${CMAKE_SOURCE_DIR}/tests")
set(DATA_DIR "${CMAKE_SOURCE_DIR}/data")
file(MAKE_DIRECTORY "${DATA_DIR}")

set(RFDETR_SOURCES "${SOURCE_DIR}/rfdetr_inference.cpp" "${SOURCE_DIR}/backends/inference_backend.cpp")
if(USE_TENSORRT)
    list(APPEND RFDETR_SOURCES "${SOURCE_DIR}/backends/tensorrt_backend.cpp")
endif()

add_library(rfdetr_inference_lib STATIC ${RFDETR_SOURCES})
target_include_directories(rfdetr_inference_lib PUBLIC "${SOURCE_DIR}" "${OpenCV_INCLUDE_DIRS}" "${TENSORRT_INCLUDE_DIR}" "${CUDAToolkit_INCLUDE_DIRS}")

target_link_libraries(rfdetr_inference_lib PUBLIC ${OpenCV_LIBRARIES} CUDA::cudart CUDA::nppc CUDA::nppig CUDA::nppial CUDA::nppist dl rt)

if(USE_TENSORRT)
    target_link_libraries(rfdetr_inference_lib PUBLIC "${TENSORRT_LIBRARY}" "${TENSORRT_ONNX_PARSER}")
    target_compile_definitions(rfdetr_inference_lib PUBLIC USE_TENSORRT)
endif()

# --- Fetch WPILib ---
include(FetchContent)
set(WPI_VERSION "2025.1.1")
set(WPI_MAVEN_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first")
function(fetch_wpilib_lib NAME)
    FetchContent_Declare(${NAME}_headers URL "${WPI_MAVEN_URL}/${NAME}/${NAME}-cpp/${WPI_VERSION}/${NAME}-cpp-${WPI_VERSION}-headers.zip")
    FetchContent_MakeAvailable(${NAME}_headers)
    FetchContent_Declare(${NAME}_libs URL "${WPI_MAVEN_URL}/${NAME}/${NAME}-cpp/${WPI_VERSION}/${NAME}-cpp-${WPI_VERSION}-linuxarm64.zip")
    FetchContent_MakeAvailable(${NAME}_libs)
    add_library(${NAME}_inc INTERFACE)
    target_include_directories(${NAME}_inc INTERFACE "${${NAME}_headers_SOURCE_DIR}")
    add_library(${NAME} SHARED IMPORTED)
    find_library(${NAME}_LIB_PATH NAMES ${NAME} PATHS "${${NAME}_libs_SOURCE_DIR}/linuxarm64" "${${NAME}_libs_SOURCE_DIR}/linux/arm64/shared" NO_DEFAULT_PATH REQUIRED)
    set_target_properties(${NAME} PROPERTIES IMPORTED_LOCATION "${${NAME}_LIB_PATH}")
    target_link_libraries(${NAME} INTERFACE ${NAME}_inc)
endfunction()
fetch_wpilib_lib(wpiutil)
fetch_wpilib_lib(wpinet)
fetch_wpilib_lib(ntcore)
target_link_libraries(wpinet INTERFACE wpiutil)
target_link_libraries(ntcore INTERFACE wpinet wpiutil)
target_link_libraries(rfdetr_inference_lib PUBLIC ntcore)

# --- Main Application ---
# Manual override: force link the problematic libraries to satisfy DLA dependencies
find_library(LIBCUDA_REAL NAMES cuda PATHS "/usr/lib/aarch64-linux-gnu/nvidia" "/lib/aarch64-linux-gnu" NO_DEFAULT_PATH)
find_library(LIBNVCUDLA_REAL NAMES nvcudla PATHS "/usr/lib/aarch64-linux-gnu/nvidia" NO_DEFAULT_PATH)

add_executable(inference_app "${SOURCE_DIR}/main.cpp")
target_link_libraries(inference_app PRIVATE rfdetr_inference_lib ${LIBCUDA_REAL} ${LIBNVCUDLA_REAL})

# CRITICAL: Replicate search paths using colon-separated list
string(REPLACE ";" ":" JETSON_LINK_PATH_STR "${JETSON_LINK_PATHS}")
target_link_options(inference_app PRIVATE "-Wl,-rpath-link,${JETSON_LINK_PATH_STR}")

# --- Tests ---
add_executable(unit_tests "${TEST_DIR}/unit/test_rfdetr_inference.cpp")
target_link_libraries(unit_tests PRIVATE rfdetr_inference_lib gtest gtest_main gmock)
foreach(libname wpiutil wpinet ntcore)
    get_target_property(lib_loc ${libname} IMPORTED_LOCATION)
    add_custom_command(TARGET inference_app POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${lib_loc}" $<TARGET_FILE_DIR:inference_app>)
endforeach()
