idf_component_register(
    SRCS 
        hello_world_main.cpp
#        webserver.cpp
        image_processing.cpp
        camera.cpp
        usb_stream.cpp
    INCLUDE_DIRS 
        .
        ./opencv)


        # Be aware that the order of the librairies is important
add_prebuilt_library(opencv_imgcodecs "opencv/libopencv_imgcodecs.a")

# https://github.com/joachimBurket/esp32-opencv/issues/9
#add_prebuilt_library(libpng "opencv/3rdparty/liblibpng.a")
add_prebuilt_library(libpng "opencv/3rdparty/liblibpng.a" PRIV_REQUIRES esp_rom)

# https://github.com/espressif/esp-idf/issues/10496#issuecomment-1374208449
#add_prebuilt_library(libsetjmp_test ${LIB_FILE} PRIV_REQUIRES esp_rom)

add_prebuilt_library(libzlib "opencv/3rdparty/libzlib.a")
add_prebuilt_library(opencv_imgproc "opencv/libopencv_imgproc.a")
add_prebuilt_library(opencv_core "opencv/libopencv_core.a")
add_prebuilt_library(ade "opencv/libade.a")




set(IMGCODEC_DEP libpng libzlib)
target_link_libraries(opencv_imgcodecs INTERFACE ${IMGCODEC_DEP})

set(CORE_DEP libzlib)
target_link_libraries(opencv_core INTERFACE ${CORE_DEP})

set(OPENCV_DEP  opencv_imgcodecs opencv_imgproc opencv_core)
target_link_libraries(${COMPONENT_LIB} ${OPENCV_DEP})

# create spiffs partition (named 'storage') from the ../spiffs_image directory
 spiffs_create_partition_image(storage ../spiffs_images FLASH_IN_PROJECT)