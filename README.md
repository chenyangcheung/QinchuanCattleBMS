# QinchuanCattleBMS

## Overview

This file includes basic environment and configuration of QinchuanCattleBMS. You can set all of them flexible based on your version and installation path, but we perfer the suggested way as follow.

## Introduction

Hi, it's Gloria. This is my first time to write a "READ ME", and I'm so excited that our project is finally on the way to complete something valuable. I added some tips about writing a "READ ME" file in atom at last. Hope it will be helpful for your work.

## Environment

1. Qt    
Suggested version: qt-opensource-windows-x86-msvc2015_64-5.6.3
2. VLC-Qt    
Suggested version: VLC-Qt_1.1.0_win64_msvc2015
3. OpenCV     
Suggested version: opencv-2.4.13.6-vc14
4. Visual Studio    
Suggested version: visual_studio_community_2015_x86

## Configuration

### Static library:

1. Name: OPENCV_VERSION     
   Value: OPENCV2     
   Tips: The value of this variable should be UPPER case.The value can be OPENCV2 or OPENCV3 which depends on the version of your OpenCV.
1. Name:OPENCV_PATH     
   Value: %OPENCV_DIR%
1. Name: BOOST_INSTALL_PATH     
   value: %BOOST_INSTALL_DIR%\local\boost_1_64_0
1. Name: IFM3D_INSTALL_PATH                
   Value: %IFM3D_INSTALL_PATH%\install
1. Name: IFM3D_PATH              
   Value: %IFM3D_PATH_DIR%\ifm3d

### Dynamic link library:

**Name: Path**
1. Value：%OPENCV_DIR%\bin
1. Value：%QT_DIR%\bin
1. Value: %VLC_DIR%\bin


## Tips

### list

- jfksjdkfjds
- fdsjfkjsdf

### number list

1.   a
1. dd
1. sd

### italic

*word*

### bold

**word**

### hyperlink

[baidu](http://www.baidu.com)

### code block

`code`


```
fdsfjksdfjkj
```

### reference

> fdsfdsfjkj
