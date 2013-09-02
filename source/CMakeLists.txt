# Libraries all Kinect's proyects
set( Proyect_LIBS
    ${PCL_LIBRARIES}     # All PCL libraries
    ${OpenCV_LIBS}          # OpenCV libraries
)

# ----------------------------------------------------
# Proyect: Capture
# Capture Point Clouds using Kinect, with OpenNi and PCL

# Define source files
set( Capture_SRC
    capture.cpp             # Main file
    openni_viewer.h         # OpenNi Viewer Class
    OpenNiViewer.h
)

add_executable(Capture ${Capture_SRC})
target_link_libraries(Capture ${Proyect_LIBS})

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
