#-------------------------------------------------
#
# Project created by QtCreator 2018-07-24T08:29:07
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QinchuanCattleBMS
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    snapshotthread.cpp \
    ifm3dviewer.cpp \
    imgmarkscene.cpp \
    markitem.cpp \
    imgpoint.cpp \
    bmscore.cpp

HEADERS  += mainwindow.h \
    snapshotthread.h \
    ifm3dviewer.h \
    imgmarkscene.h \
    markitem.h \
    imgpoint.h \
    bmscore.h

FORMS    += mainwindow.ui

# VLC configuration
win32 {
        message("Using win32 configuration")

        # change this variable according to your path of opencv
#        VLC_PATH = C:/VLC-Qt_1.1.0_win64_msvc2015 # Note: update with the correct OpenCV version
        VLC_PATH = $$(VLC_PATH)
        # change this variable according to your version of opencv
        LIBS_PATH = "$$VLC_PATH/lib"

        CONFIG(debug, debug|release) {
            LIBS     += -L$$LIBS_PATH \
                        -lVLCQtCored \
                        -lVLCQtWidgetsd
           }

        CONFIG(release, debug|release) {
            LIBS     += -L$$LIBS_PATH \
                        -lVLCQtCore \
                        -lVLCQtWidgets
           }
}

INCLUDEPATH += \
    $$VLC_PATH/include/

message("VLC path: $$VLC_PATH")
#message("Includes path: $$INCLUDEPATH")
#message("Libraries: $$LIBS")

# ifm3d library
IFM3D_PATH = $$(IFM3D_PATH)

LIBS += -L$$IFM3D_PATH/lib \
        -lifm3d_camera \
        -lifm3d_framegrabber \
        -lifm3d_image \
        -lifm3d_pcicclient \
        -lifm3d_tools

INCLUDEPATH += \
    $$IFM3D_PATH/modules\framegrabber\include \
    $$IFM3D_PATH/modules\camera\include \
    $$IFM3D_PATH/modules\image\include


message("ifm3d path: $$IFM3D_PATH")

# pcl library
PCL_PATH = $$(IFM3D_INSTALL_PATH)

LIBS += -L$$PCL_PATH/lib \
        -lpcl_common_release \
        -lpcl_features_release \
        -lpcl_filters_release \
        -lpcl_io_ply_release \
        -lpcl_io_release \
        -lpcl_kdtree_release \
        -lpcl_keypoints_release \
        -lpcl_ml_release \
        -lpcl_octree_release \
        -lpcl_outofcore_release \
        -lpcl_people_release \
        -lpcl_recognition_release \
        -lpcl_registration_release \
        -lpcl_sample_consensus_release \
        -lpcl_search_release \
        -lpcl_segmentation_release \
        -lpcl_stereo_release \
        -lpcl_surface_release \
        -lpcl_tracking_release \
        -lpcl_visualization_release

INCLUDEPATH += $$PCL_PATH/include/pcl-1.8

message("pcl path: $$PCL_PATH")

# vtk library
VTK_PATH = $$(IFM3D_INSTALL_PATH)

LIBS += -L$$VTK_PATH/lib  \
        -lvtkalglib-6.3 \
        -lvtkChartsCore-6.3 \
        -lvtkCommonColor-6.3 \
        -lvtkCommonComputationalGeometry-6.3 \
        -lvtkCommonCore-6.3 \
        -lvtkCommonDataModel-6.3 \
        -lvtkCommonExecutionModel-6.3 \
        -lvtkCommonMath-6.3 \
        -lvtkCommonMisc-6.3 \
        -lvtkCommonSystem-6.3 \
        -lvtkCommonTransforms-6.3 \
        -lvtkDICOMParser-6.3 \
        -lvtkDomainsChemistry-6.3 \
        -lvtkexoIIc-6.3 \
        -lvtkexpat-6.3 \
        -lvtkFiltersAMR-6.3 \
        -lvtkFiltersCore-6.3 \
        -lvtkFiltersExtraction-6.3 \
        -lvtkFiltersFlowPaths-6.3 \
        -lvtkFiltersGeneral-6.3 \
        -lvtkFiltersGeneric-6.3 \
        -lvtkFiltersGeometry-6.3 \
        -lvtkFiltersHybrid-6.3 \
        -lvtkFiltersHyperTree-6.3 \
        -lvtkFiltersImaging-6.3 \
        -lvtkFiltersModeling-6.3 \
        -lvtkFiltersParallel-6.3 \
        -lvtkFiltersParallelImaging-6.3 \
        -lvtkFiltersProgrammable-6.3 \
        -lvtkFiltersSelection-6.3 \
        -lvtkFiltersSMP-6.3 \
        -lvtkFiltersSources-6.3 \
        -lvtkFiltersStatistics-6.3 \
        -lvtkFiltersTexture-6.3 \
        -lvtkFiltersVerdict-6.3 \
        -lvtkfreetype-6.3 \
        -lvtkftgl-6.3 \
        -lvtkGeovisCore-6.3 \
        -lvtkgl2ps-6.3 \
        -lvtkGUISupportQt-6.3 \
        -lvtkGUISupportQtOpenGL-6.3 \
        -lvtkhdf5_hl-6.3 \
        -lvtkhdf5-6.3 \
        -lvtkImagingColor-6.3 \
        -lvtkImagingCore-6.3 \
        -lvtkImagingFourier-6.3 \
        -lvtkImagingGeneral-6.3 \
        -lvtkImagingHybrid-6.3 \
        -lvtkImagingMath-6.3 \
        -lvtkImagingMorphological-6.3 \
        -lvtkImagingSources-6.3 \
        -lvtkImagingStatistics-6.3 \
        -lvtkImagingStencil-6.3 \
        -lvtkInfovisCore-6.3 \
        -lvtkInfovisLayout-6.3 \
        -lvtkInteractionImage-6.3 \
        -lvtkInteractionStyle-6.3 \
        -lvtkInteractionWidgets-6.3 \
        -lvtkIOAMR-6.3 \
        -lvtkIOCore-6.3 \
        -lvtkIOEnSight-6.3 \
        -lvtkIOExodus-6.3 \
        -lvtkIOExport-6.3 \
        -lvtkIOGeometry-6.3 \
        -lvtkIOImage-6.3 \
        -lvtkIOImport-6.3 \
        -lvtkIOInfovis-6.3 \
        -lvtkIOLegacy-6.3 \
        -lvtkIOLSDyna-6.3 \
        -lvtkIOMINC-6.3 \
        -lvtkIOMovie-6.3 \
        -lvtkIONetCDF-6.3 \
        -lvtkIOParallel-6.3 \
        -lvtkIOParallelXML-6.3 \
        -lvtkIOPLY-6.3 \
        -lvtkIOSQL-6.3 \
        -lvtkIOVideo-6.3 \
        -lvtkIOXML-6.3 \
        -lvtkIOXMLParser-6.3 \
        -lvtkjpeg-6.3 \
        -lvtkjsoncpp-6.3 \
        -lvtklibxml2-6.3 \
        -lvtkmetaio-6.3 \
        -lvtkNetCDF_cxx-6.3 \
        -lvtkNetCDF-6.3 \
        -lvtkoggtheora-6.3 \
        -lvtkParallelCore-6.3 \
        -lvtkpng-6.3 \
        -lvtkproj4-6.3 \
        -lvtkRenderingAnnotation-6.3 \
        -lvtkRenderingContext2D-6.3 \
        -lvtkRenderingContextOpenGL-6.3 \
        -lvtkRenderingCore-6.3 \
        -lvtkRenderingFreeType-6.3 \
        -lvtkRenderingGL2PS-6.3 \
        -lvtkRenderingImage-6.3 \
        -lvtkRenderingLabel-6.3 \
        -lvtkRenderingLIC-6.3 \
        -lvtkRenderingLOD-6.3 \
        -lvtkRenderingOpenGL-6.3 \
        -lvtkRenderingVolume-6.3 \
        -lvtkRenderingVolumeOpenGL-6.3 \
        -lvtksqlite-6.3 \
        -lvtksys-6.3 \
        -lvtktiff-6.3 \
        -lvtkverdict-6.3 \
        -lvtkViewsContext2D-6.3 \
        -lvtkViewsCore-6.3 \
        -lvtkViewsInfovis-6.3 \
        -lvtkzlib-6.3

INCLUDEPATH += $$VTK_PATH/include/vtk-6.3

message("vtk path: $$VTK_PATH")

# xmlrpc library
XMLRPC_PATH = $$(IFM3D_INSTALL_PATH)

LIBS += -L$$XMLRPC_PATH/libs \
        -lxmlrpc \
        -lxmlrpc_abyss \
        -lxmlrpc_client \
        -lxmlrpc_client++ \
        -lxmlrpc_cpp \
        -lxmlrpc_packetsocket \
        -lxmlrpc_server \
        -lxmlrpc_server_pstream++ \
        -lxmlrpc_server++ \
        -lxmlrpc_util \
        -lxmlrpc++

INCLUDEPATH += $$XMLRPC_PATH/include

message("xmlrpc path: $$XMLRPC_PATH")

# boost library
BOOST_PATH = $$(BOOST_INSTALL_PATH)

LIBS += -L$$BOOST_PATH/lib64-msvc-14.0 \
        -llibboost_thread-vc140-mt-1_64

INCLUDEPATH += $$BOOST_PATH

# eigen library
EIGEN_PATH = $$(IFM3D_INSTALL_PATH)

#LIBS += -L$$EIGEN_PATH/
INCLUDEPATH += $$EIGEN_PATH/include/eigen3

message("boost path: $$BOOST_PATH")

# opencv library
OPENCV_PATH = $$(IFM3D_INSTALL_PATH)

LIBS += -L$$OPENCV_PATH/x64/vc14/lib \
        -lopencv_core340 \
        -lopencv_imgproc340 \
        -lopencv_highgui340 \
        -lopencv_imgcodecs340

INCLUDEPATH += $$OPENCV_PATH/include

message("opencv path: $$OPENCV_PATH")

message("All include path: $$INCLUDEPATH")
message("All Libraries: $$LIBS")

