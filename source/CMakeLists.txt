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
    filter_cloud.cpp             # Main file
    filters_viewer.h         # Filters Class
)

add_executable(Filter_Cloud ${Filter_Cloud_SRC})
target_link_libraries(Filter_Cloud ${Filter_Cloud_LIBS})

# ----------------------------------------------------
# Project Kinect Scanner
set( Kinect_Scanner_SRC
    main.cpp

    openni_viewer.h
    watershed_segmenter.h
)

add_executable(Kinect_Scanner ${Kinect_Scanner_SRC})
target_link_libraries(Kinect_Scanner ${Proyect_LIBS})

## ----------------------------------------------------
## Project Kinect Scanner
#set( Kinect_Scanner_SRC
#    main.cpp

#    openni_viewer.h
#    watershed_segmenter.h
#)

#add_executable(Kinect_Scanner ${Kinect_Scanner_SRC})
#target_link_libraries(Kinect_Scanner ${PCL_LIBS})
