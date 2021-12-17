add_library(VirtualTelescope SHARED VirtualTelescope.cpp
                                    VirtualTelescope.h
        PointModel.h
        PointModel.cpp
        IERS.h
        IERS.cpp
        )


target_include_directories(VirtualTelescope PUBLIC /opt/gcs/ext/gsl_1.16/include)
target_link_directories(VirtualTelescope PUBLIC /opt/gcs/ext/gsl_1.16/lib/linux)

target_include_directories(VirtualTelescope PUBLIC /opt/gcs/ext/sla/include/)
target_link_directories(VirtualTelescope PUBLIC /opt/gcs/ext/sla/lib/linux)

target_include_directories(VirtualTelescope PUBLIC /opt/gcs/ext/tcsPK/include/)
target_link_directories(VirtualTelescope PUBLIC /opt/gcs/ext/tcsPK/lib/linux)

add_executable(main main.cpp PointModel.cpp PointModel.h IERS.cpp IERS.h TelescopeContext.cpp TelescopeContext.h VirtualTelescopeSkyToEncoder.cpp VirtualTelescopeSkyToEncoder.h VirtualTelescopeEncoderToSky.cpp VirtualTelescopeEncoderToSky.h VirtualTelescopeSkyToPointingOrigin.cpp VirtualTelescopeSkyToPointingOrigin.h)
target_link_libraries(main VirtualTelescope gsl gslcblas tcsPK slalib sla  )
