/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef MAINWIDGET_H
#define MAINWIDGET_H

//#include "geometryengine2.h"
#include "shader.h"
#include "geometryengine.h"

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QVector2D>
#include <QBasicTimer>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QMouseEvent>
#include <QTouchEvent>



#include "opencv2/opencv.hpp"
//class GeometryEngineE;
class GeometryEngine;

class MainWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit MainWidget(QWidget *parent = 0);
    ~MainWidget();

protected:
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void timerEvent(QTimerEvent *e) override;
    void wheelEvent(QWheelEvent *event) override;
 //   void touchEvent(QTouchEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void initShaders();
    void initTextures();

    void init();

    bool IntersectTriangle(const glm::vec3& orig, const glm::vec3& dir,glm::vec3& v0, glm::vec3& v1, glm::vec3& v2,float* t, float* u, float* v);
    void selectTriangle(int x, int y);
   // void selectTriangle(int x, int y);
private:
 //   QBasicTimer timer;
    QOpenGLShaderProgram program;
    GeometryEngine *geometries;

   // QOpenGLTexture *texture;

 
  //  Shader* m_shader;
  //  Shader* m_shader2;
/*
    QVector2D mousePressPosition;
    QVector3D rotationAxis;
    qreal angularSpeed;
    QQuaternion rotation;
    */
    qreal aspect;
    double m_ddfoy;
     qreal  m_fovy ;

     int  m_lastx;
     int  m_lasty;
     float m_rotationX;
     float m_rotationZ;
     bool m_bl;

     //cv::VideoCapture capAll2;
     cv::VideoCapture cap;
     GLuint textureUniform; //y纹理数据位置
     GLuint id_texture; //y纹理对象ID
     QOpenGLTexture* m_pTexture;  //y纹理对象


     GLuint textureUniformAll; //y纹理数据位置
     GLuint id_textureAll; //y纹理对象ID
     QOpenGLTexture* m_pTextureAll;  //y纹理对象

     GLuint textureUniformAll1; //y纹理数据位置
     GLuint id_textureAll1; //y纹理对象ID
     QOpenGLTexture* m_pTextureAll1;  //y纹理对象


     GLuint textureUniformJtou; //y纹理数据位置
     GLuint id_textureJtou; //y纹理对象ID
     QOpenGLTexture* m_pTextureJtou;  //y纹理对象

    // QOpenGLTexture* m_pTextureU;  //u纹理对象
     //QOpenGLTexture* m_pTextureV;  //v纹理对象

    int m_nVideoW;
    int m_nVideoH;
    int m_nVideoWALL;
    int m_nVideoHALL;
    QStringList m_filespathname;

    int m_nxuhao;
    int m_nxuhaoqian;
    bool m_blmp4 ;
  //  QMatrix4x4 matrixview;
  //  QMatrix4x4 projection;
   // glm::mat4 m_modelMatrix;
   // glm::mat4 m_projection;
   // glm::mat4 m_view;
    // Camera m_camera;
    glm::mat4 m_modelMatrix;
    glm::mat4 m_projection;
    glm::mat4 m_view;

    glm::vec3 m_posIntersect;
    glm::vec3 m_nearPosition;
    glm::vec3 m_farPosition;
    std::vector<glm::vec3>posList;

    float  m_time;

    int  m_qiehuan;

    float m_qfovy;

    GLint m_dviewport[4];


    int m_nw;
    int m_nh;
    bool m_blsize;

    int m_nqiehuanzhen;
};

#endif // MAINWIDGET_H
