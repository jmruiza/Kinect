# Libraries all Kinect's proyects
set( Proyect_LIBS
    ${PCL_LIBRARIES}     # All PCL libraries
    ${OpenCV_LIBS}          # OpenCV libraries
)

# Phase 1:
# ----------------------------------------------------
# Proyect: Capture
# Capture Point Clouds using Kinect, with OpenNi and PCL

# Define source files
set( Capture_SRC
    capture.cpp             # Main file
    openni_viewer.h         # OpenNi Viewer Class
)

add_executable(Capture ${Capture_SRC})
target_link_libraries(Capture ${Proyect_LIBS})

# Phase 2:
# ----------------------------------------------------
# Proyect: Filter Cloud
# Apply diferent filters to capture Point Cloud

# Define source files
set( Filter_Cloud_SRC
    filter_cloud.cpp            # Main file
    filters_viewer.h            # Filters Class
)

add_executable(Filter_Cloud ${Filter_Cloud_SRC})
target_link_libraries(Filter_Cloud ${Proyect_LIBS})

# Proyect: Segmenter_Image
# Apply diferent filters to capture Point Cloud

# Define source files
set( Segmenter_Image_SRC
    segmenter_image.cpp    # Main file
    segmenter.h            # Segmenter Class
    blob_detector.h        # Blob detector class
)

add_executable(Segmenter_Image ${Segmenter_Image_SRC})
target_link_libraries(Segmenter_Image ${Proyect_LIBS})

# Phase 3:
# ----------------------------------------------------
# Proyect: Heights
# Get heights of diferent points of a point cloud

# Define source files
set( Heights_SRC
    heights.cpp         # Main file
    heights.h           # Heights class
)

add_executable(Heights ${Heights_SRC})
target_link_libraries(Heights ${Proyect_LIBS})


