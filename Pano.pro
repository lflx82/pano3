QT       += core gui widgets

TARGET = Pano
TEMPLATE = app

SOURCES += main.cpp \
    geometryengine.cpp \
    mainwidget.cpp

INCLUDEPATH += E:/Pano/include

SOURCES +=

path = D:/opencv-3.4.2source/opencv-3.4.2/
INCLUDEPATH += $${path}/include
CONFIG(debug,debug|release){
LIBS += $${path}/x64bulid/lib/Debug/opencv_core342d.lib
LIBS += $${path}/x64bulid/lib/Debug/opencv_videoio342d.lib
LIBS += $${path}/x64bulid/lib/Debug/opencv_videoio342d.lib
LIBS += $${path}/x64bulid/lib/Debug/opencv_imgproc342d.lib
LIBS += $${path}/x64bulid/lib/Debug/opencv_imgcodecs342d.lib
} else {
LIBS += $${path}/x64bulid/lib/Release/opencv_core342.lib
LIBS += $${path}/x64bulid/lib/Release/opencv_videoio342.lib
LIBS += $${path}/x64bulid/lib/Release/opencv_videoio342.lib
LIBS += $${path}/x64bulid/lib/Release/opencv_imgproc342.lib
LIBS += $${path}/x64bulid/lib/Release/opencv_imgcodecs342.lib
}

LIBS += "D:\Windows Kits\10\Lib\10.0.18362.0\um\x64\GlU32.Lib"
HEADERS += \
    geometryengine.h \
    mainwidget.h \
    shader.h

RESOURCES += \
    shaders.qrc \
    textures.qrc

# install
target.path = $$[QT_INSTALL_EXAMPLES]/opengl/cube
INSTALLS += target

