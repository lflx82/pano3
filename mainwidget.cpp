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

#include "mainwidget.h"

#include <QDir>
#include <QMouseEvent>

#include <math.h>
#include <QDesktopWidget>
#include <QApplication>
#include "opencv2/imgproc/imgproc.hpp"
#include "gl/glu.h"
#include "glm/gtc/matrix_transform.hpp"
MainWidget::MainWidget(QWidget *parent) :
    QOpenGLWidget(parent),
    geometries(0),m_nxuhao(0),m_nxuhaoqian(0)
{
    m_ddfoy=0;

//    mousePressPosition = QVector2D{0,0};
//    angularSpeed += 0.1;
    m_fovy=80;
    m_rotationX=0;
    m_rotationZ=90;
    m_lastx =0;
    m_lasty =0;
    double r = .8;

    setGeometry(100,100,qApp->desktop()->screenGeometry().width()*r,qApp->desktop()->screenGeometry().height()*r);
//    setGeometry(100,100,800,600);
    m_bl=true;

    textureUniform = 0;
    id_texture = 0;
    m_pTexture = NULL;


    textureUniformAll = 0;
    id_textureAll = 0;
    m_pTextureAll = NULL;

    textureUniformAll1 = 0;
    id_textureAll1 = 0;
    m_pTextureAll1 = NULL;

    textureUniformJtou= 0;
    id_textureJtou = 0;
    m_pTextureJtou = NULL;

    startTimer(40);
    init();
    m_blmp4=false;
    m_time=1.0f;

    m_qiehuan=0;

    m_nqiehuanzhen=30;
}

MainWidget::~MainWidget()
{
    // Make sure the context is current when deleting the texture
    // and the buffers.
    makeCurrent();
/*
    delete m_shader;
      m_shader = NULL;
      delete m_shader2;
      m_shader2 = NULL;
      */
    delete geometries;
    delete  m_pTextureAll;
    delete  m_pTexture;
    delete  m_pTextureAll1;
    delete  m_pTextureJtou;
    doneCurrent();
}

//! [0]
void MainWidget::mousePressEvent(QMouseEvent *e)
{

    if(e->button()==Qt::LeftButton)
    {

        m_lastx =e->x();
        m_lasty =e->y();
    }

   if(e->button()==Qt::RightButton)
    {
        m_nxuhaoqian=m_nxuhao;
        m_nxuhao++;
        m_bl=true;
        m_blmp4=false;

        m_rotationX=180;
        m_rotationZ=90;

    }

    if(e->button()==Qt::RightButton)
     {
     //   selectTriangle(e->x(), e->y());
      // int x= e->x();
      // int y= e->y();
        //selectTriangle(x, y);
    }

}

bool MainWidget:: IntersectTriangle(const glm::vec3& orig, const glm::vec3& dir,glm::vec3& v0, glm::vec3& v1, glm::vec3& v2,float* t, float* u, float* v)
{
    // E1
    glm::vec3 E1 = v1 - v0;
    // E2
    glm::vec3 E2 = v2 - v0;
    // P
    glm::vec3 P = glm::cross(dir, E2);
    // determinant
    float det = glm::dot(E1, P);

    // keep det > 0, modify T accordingly
    glm::vec3 T;
    if (det > 0)
    {
        T = orig - v0;
    }
    else
    {
        T = v0 - orig;
        det = -det;
    }

    // If determinant is near zero, ray lies in plane of triangle
    if (det < 0.0001f)
        return false;

    // Calculate u and make sure u <= 1
    *u = glm::dot(T, P);

    if (*u < 0.0f || *u > det)
        return false;

    // Q
    glm::vec3 Q = glm::cross(T, E1);


    // Calculate v and make sure u + v <= 1
    *v = glm::dot(dir, Q);

    if (*v < 0.0f || *u + *v > det)
        return false;

    // Calculate t, scale parameters, ray intersects triangle
    *t = glm::dot(E2, Q);

    float fInvDet = 1.0f / det;
    *t *= fInvDet;
    *u *= fInvDet;
    *v *= fInvDet;
    return true;
}


void MainWidget::selectTriangle(int x, int y)
{
     int screenX = (x);
     int screenY = qApp->desktop()->screenGeometry().height() - (y);
      qDebug()<<"shubiaox="<<x<<"shuobiaoy="<<y;

    posList.clear();
    glm::mat4 modelMatrix;
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
           // qDebug()<<"i="<<i<<"j="<<j<<"modelMatrix[i][j]="<<modelMatrix[i][j];
        }
    }


    float* viewArray = (float*)glm::value_ptr(m_view);
    float* projectArray = (float*)glm::value_ptr(m_projection);

    double View[16];
    for (int i = 0; i < 16; i++)
    {
        View[i] = viewArray[i];
    }
    double project[16];
    for (int i = 0; i < 16; i++)
    {
        project[i] = projectArray[i];
    }
    double posX, posY, posZ;
    bool bResult = gluUnProject(screenX, screenY, 0.0, View, project, m_dviewport, &posX, &posY, &posZ);
    m_nearPosition = glm::vec3(posX, posY, posZ);

    bResult = gluUnProject(screenX, screenY, 1.0, View, project, m_dviewport, &posX, &posY, &posZ);
    m_farPosition = glm::vec3(posX, posY, posZ);
    glm::mat4 matrix;

    float tMin = 999999.9;


    int num=0;
    for (int j = 0; j < geometries->m_nsize / 3; j++)
    {
        GLuint index0 = geometries->m_pindices[j * 3];
        GLuint index1 = geometries->m_pindices[j * 3+1];
        GLuint index2 = geometries->m_pindices[j * 3+2];

        VertexData vertex0 =geometries->m_pvertices[index0];
        VertexData vertex1 =geometries->m_pvertices[index1];
        VertexData vertex2 =geometries->m_pvertices[index2];

        glm::vec3 vert0(vertex0.position.x(), vertex0.position.y(), vertex0.position.z());
        glm::vec3 vert1(vertex1.position.x(), vertex1.position.y(), vertex1.position.z());
        glm::vec3 vert2(vertex2.position.x(), vertex2.position.y(), vertex2.position.z());

        glm::vec3 vert00 = glm::vec3(modelMatrix * glm::vec4(vert0, 1.0));
        glm::vec3 vert01 = glm::vec3(modelMatrix * glm::vec4(vert1, 1.0));
        glm::vec3 vert02 = glm::vec3(modelMatrix * glm::vec4(vert2, 1.0));

        matrix = glm::inverse(modelMatrix);
        float t, u, v;
        bool isIntersect = IntersectTriangle(m_nearPosition, (m_farPosition - m_nearPosition), vert00, vert01, vert02, &t, &u, &v);

        if (isIntersect)
        {
            num++;
            std::cout << "jiaodiangeshu=" <<num<< std::endl;
            glm::vec3 interPoint = m_nearPosition + (m_farPosition - m_nearPosition) * t;
            interPoint = glm::vec3(matrix * glm::vec4(interPoint, 1.0));
            if (t < tMin)
            {
                m_posIntersect = interPoint;
                tMin = t;
            }
            posList.push_back(interPoint);
        }
    }
       m_nearPosition = glm::vec3(matrix * glm::vec4(m_nearPosition, 1.0));
       m_farPosition = glm::vec3(matrix * glm::vec4(m_farPosition, 1.0));
        std::cout << m_nearPosition.x << "neary=" <<m_nearPosition.y << "nearz=" <<m_nearPosition.z << "near=" <<sqrt(m_nearPosition.x*m_nearPosition.x+m_nearPosition.y*m_nearPosition.y+m_nearPosition.z*m_nearPosition.z)  << std::endl;
        std::cout << m_posIntersect.x << "posy=" <<m_posIntersect.y << "posz=" << m_posIntersect.z <<"pos="<<sqrt(m_posIntersect.x*m_posIntersect.x+m_posIntersect.y*m_posIntersect.y+m_posIntersect.z*m_posIntersect.z)  <<std::endl;
        std::cout << m_farPosition.x << "fary=" <<m_farPosition.y << "farz=" <<m_farPosition.z<< "far="<<sqrt(m_farPosition.x*m_farPosition.x+m_farPosition.y*m_farPosition.y+m_farPosition.z*m_farPosition.z)  << std::endl;
}

void MainWidget::mouseMoveEvent(QMouseEvent *e)
{

    if(e->buttons()&Qt::LeftButton)
    {
       m_rotationX=m_rotationX-(e->x()-m_lastx)/devicePixelRatio();
       m_rotationZ=m_rotationZ- (e->y()-m_lasty)/devicePixelRatio();
       m_lastx =e->x();
       m_lasty =e->y();
    }

}

void MainWidget::mouseReleaseEvent(QMouseEvent *e)
{

    if(e->button()==Qt::LeftButton)
    {

    }

}

void MainWidget::timerEvent(QTimerEvent *)
{
     update();

}

void MainWidget::wheelEvent(QWheelEvent *event)
{
   if(event->angleDelta().y()>0)
   {
       m_ddfoy=-1;
   }
   else if(event->angleDelta().y()<0)
   {
       m_ddfoy=1;
   }
}

 void MainWidget::keyPressEvent(QKeyEvent *event)
 {
     if(event->key() == Qt::Key_A)
     {
         m_blmp4=!m_blmp4;
     }

     if(event->key() == Qt::Key_Z)
     {
         m_qiehuan++;
     }
     if(event->key() == Qt::Key_Q)
     {
         m_nqiehuanzhen=m_nqiehuanzhen+5;
          qDebug() <<"m_nqiehuanzhen="<< m_nqiehuanzhen;
     }
     if(event->key() == Qt::Key_W)
     {
         m_nqiehuanzhen=fmax(m_nqiehuanzhen-5,0);
         qDebug() <<"m_nqiehuanzhen="<< m_nqiehuanzhen;
     }
 }
//! [1]

void MainWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0, 0, 0, 1);

    initShaders();
    initTextures();

//! [2]
    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    // Enable back face culling

//! [2]

    geometries = new GeometryEngine;

    // Use QBasicTimer because its faster than QTimer
}

//! [3]
void MainWidget::initShaders()
{
    // Compile vertex shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vshader.glsl"))
        close();

    // Compile fragment shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fshader.glsl"))
        close();

    // Link shader pipeline
    if (!program.link())
        close();

    // Bind shader pipeline for use
    if (!program.bind())
        close();
}
//! [3]

//! [4]
void MainWidget::initTextures()
{


    m_pTextureAll= new QOpenGLTexture(QOpenGLTexture::Target2D);
    m_pTextureAll->create();
    //获取返回y分量的纹理索引值
    id_textureAll= m_pTextureAll->textureId();


    m_pTextureAll1= new QOpenGLTexture(QOpenGLTexture::Target2D);
    m_pTextureAll1->create();
    //获取返回y分量的纹理索引值
    id_textureAll1= m_pTextureAll1->textureId();

    m_pTexture= new QOpenGLTexture(QOpenGLTexture::Target2D);
    m_pTexture->create();
    //获取返回y分量的纹理索引值
    id_texture= m_pTexture->textureId();
    //获取返回u分量的纹理索引值


    m_pTextureJtou= new QOpenGLTexture(QOpenGLTexture::Target2D);
    m_pTextureJtou->create();
    //获取返回y分量的纹理索引值
    id_textureJtou= m_pTextureJtou->textureId();


}

void MainWidget::init()
{
    m_filespathname.clear();
    QDir dir(qApp->applicationDirPath()+"/srcjpg");
    qDebug() <<dir.path()<<"====================";

    for(auto file: dir.entryList())
    {
        auto info = QFileInfo(dir.path()+"/"+file);
        if(info.suffix()=="jpg"||info.suffix()=="JPG")

        {
             qDebug() <<info.absoluteFilePath()<<"===================="<< info.baseName();
             m_filespathname << info.absoluteFilePath();
        }
    }
}

void MainWidget::resizeGL(int w, int h)
{
    m_blsize=true;
    glViewport(0,0,width(),height());
    aspect = qreal(w) / qreal(h ? h : 1);
    // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
}
//! [5]

static  int  a=0;
bool  blxiaoshi=false; //切换场景做渐变用
static  int n2=0;
static  int n3=0;
void MainWidget::paintGL()
{
     n3++;
    if(m_blsize)
    {
        glViewport(0,0,width(),height());
        qDebug() <<" glViewport"<<100<<" glViewport"<<100<<" glViewport"<<width()-200<<" glViewport"<<height()-200;
       // GLint dviewport[4];

        glGetIntegerv(GL_VIEWPORT, m_dviewport);
        qDebug()<<"dviewport11111="<<m_dviewport[0]<<"dviewport="<<m_dviewport[1]<<"dviewport="<<m_dviewport[2]<<"dviewport="<<m_dviewport[3];
        m_blsize=false;
    }
    glDisable(GL_DEPTH_TEST);
    if(m_filespathname.size()<1)
    {
       qDebug() <<"no src jpg";
       return;
    }

    if(m_bl)
    {

        if(m_nxuhao>=(m_filespathname.size()-1))
            m_nxuhao = 0;

        qDebug() << m_filespathname.at(m_nxuhao);
        cv::Mat frameALL=cv::imread(m_filespathname.at(m_nxuhao).toStdString());
        m_nVideoWALL=frameALL.cols;
        m_nVideoHALL=frameALL.rows;
        cv::cvtColor(frameALL,frameALL,cv::COLOR_BGR2RGB);
       // cv::flip(frameALL, frameALL, -1);

        // QImage img0(m_filespathname.at(m_nxuhaoqian));

        glActiveTexture(GL_TEXTURE0);

            //使用来自y数据生成纹理
        glBindTexture(GL_TEXTURE_2D, id_textureAll);
        //frame.data
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nVideoWALL, m_nVideoHALL, 0, GL_RGB, GL_UNSIGNED_BYTE, frameALL.data);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);


        if(m_nxuhaoqian>=(m_filespathname.size()-1))
            m_nxuhaoqian = 0;
        cv::Mat frameALL1=cv::imread(m_filespathname.at(m_nxuhaoqian).toStdString());
        m_nVideoWALL=frameALL1.cols;
        m_nVideoHALL=frameALL1.rows;
        cv::cvtColor(frameALL1,frameALL1,cv::COLOR_BGR2RGB);
        //cv::flip(frameALL1, frameALL1, -1);

        glActiveTexture(GL_TEXTURE1);

            //使用来自y数据生成纹理Z
        glBindTexture(GL_TEXTURE_2D, id_textureAll1);
        //使用内存中m_pBufYuv420p数据创建真正的y数据纹理
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nVideoWALL, m_nVideoHALL, 0, GL_RGB, GL_UNSIGNED_BYTE, frameALL1.data);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        m_time=0;
        blxiaoshi=true;
        m_bl=false;
        m_qfovy=m_fovy;
    }

    if(blxiaoshi)
    {
        a++;
        m_time=a/double(m_nqiehuanzhen);

        if(m_qiehuan%3==2)
        {
            if(a<m_nqiehuanzhen/2)
            {
              m_fovy=m_fovy-2;
            }
            else
            {
                m_fovy=fmin(m_qfovy,m_fovy+2);
            }
            m_fovy=fminf(m_qfovy,m_fovy);
            qDebug()  <<"m_fovy= "<< m_fovy;
        }
    }

    if(a>=m_nqiehuanzhen)
    {
       blxiaoshi=false;
       m_fovy=m_qfovy;
       a=0;
       m_time=1.0;
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    const qreal zNear = 1.0, zFar = 100.0;

    m_fovy=m_fovy+m_ddfoy;
    m_ddfoy=0;
    m_fovy=fmin(90.0,m_fovy);
    m_fovy=fmax(25.0,m_fovy);
    // Reset projection
    m_projection = glm::perspective(m_fovy*PI/180.0,aspect, zNear, zFar);
    float* p= (float*)glm::value_ptr(m_projection);
    float proj[4][4];
    for (int i = 0; i < 16; i++)
    {
        proj[i%4][i/4]=p[i];
    }

    m_rotationZ=fmax(m_rotationZ,0.2);
    m_rotationZ=fmin(m_rotationZ,179.8);


     // QVector3D eye(0,0,0);
     QVector3D center(1,0,0);
     float fai=m_rotationX*PI/180.0;
     float stai=m_rotationZ*PI/180.0;
      center.setX((float)(2 * sin(stai) * cos(fai)));
      center.setY(((float)(2* sin(stai) * sin(fai))));
      center.setZ((float)(2 * cos(stai)));

      glm::vec3 eye2(0,0,0);
       glm::vec3 center2(center.x(),center.y(),center.z());
     //glm::vec3 center2(-1.0,0,0);
      glm::vec3 up2(0,0,1);

      m_view=glm::lookAt(eye2,center2 , up2);


      float* p2= (float*)glm::value_ptr(m_view);
      float pview[4][4];
      for (int i = 0; i < 16; i++)
      {
          pview[i%4][i/4]=p2[i];
      }
      QMatrix4x4 QMview(pview[0][0],pview[0][1],pview[0][2],pview[0][3],pview[1][0],pview[1][1],pview[1][2],pview[1][3],pview[2][0],pview[2][1],pview[2][2],pview[2][3],pview[3][0],pview[3][1],pview[3][2],pview[3][3]);
      program.setUniformValue("mview_matrix",QMview);

      glm::mat4 pv=m_projection;
      float*  ppv= (float*)glm::value_ptr(pv);

      float dpv[4][4];
      for (int i = 0; i < 16; i++)
      {
          dpv[i%4][i/4] = ppv[i];
      }
     QMatrix4x4 QMpv(dpv[0][0],dpv[0][1],dpv[0][2],dpv[0][3],dpv[1][0],dpv[1][1],dpv[1][2],dpv[1][3],dpv[2][0],dpv[2][1],dpv[2][2],dpv[2][3],dpv[3][0],dpv[3][1],dpv[3][2],dpv[3][3]);

      program.setUniformValue("mproj_matrix",QMpv);

      program.setUniformValue("time", m_time);

//! [6]
     program.setUniformValue("state", 1);
     m_pTextureAll->bind(id_textureAll);
     program.setUniformValue("texture", id_textureAll);
     m_pTextureAll1->bind(id_textureAll1);
     program.setUniformValue("texture1", id_textureAll1);
    // Draw cube geometry
    geometries->drawCubeGeometry(&program);

    //return;
    program.setUniformValue("state", 2);
    program.setUniformValue("qiehuan", m_qiehuan%3);

    if(m_blmp4)
    {
        if (!cap.isOpened())
        {
            cap.open("E:/cube/11.mp4");
            printf("Could not open camera1...\n");
        }

        cv::Mat frame;
        if(!cap.read(frame))
        {
            cap.set(cv::CAP_PROP_POS_FRAMES,0);
            cap.read(frame);
        }
        //img = matToQImage(frame).mirrored();
        m_nVideoW=frame.cols;
        m_nVideoH=frame.rows;
        cv::cvtColor(frame,frame,cv::COLOR_BGR2RGB);
        //cv::flip(frame, frame, -1);
        glActiveTexture(GL_TEXTURE2);
            //使用来自y数据生成纹理
        glBindTexture(GL_TEXTURE_2D, id_texture);
        //使用内存中m_pBufYuv420p数据创建真正的y数据纹理
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_nVideoW, m_nVideoH, 0, GL_RGB, GL_UNSIGNED_BYTE, frame.data);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        //bl=true;


        m_pTexture->bind(id_texture);
        program.setUniformValue("texture2", id_texture);

        geometries->draw2(&program);
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    program.setUniformValue("state", 3);
    if(n2==0)
    {
       n2=1;
         cv::Mat framepng=cv::imread("E:/cube/122.png",cv::IMREAD_UNCHANGED);
         qDebug()  <<"tupianleixing "<< framepng.type();
          m_nVideoWALL=framepng.cols;
          m_nVideoHALL=framepng.rows;
          cv::cvtColor(framepng,framepng,cv::COLOR_BGRA2RGBA);


          glActiveTexture(GL_TEXTURE3);

              //使用来自y数据生成纹理
          glBindTexture(GL_TEXTURE_2D, id_textureJtou);
          //使用内存中m_pBufYuv420p数据创建真正的y数据纹理
           glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_nVideoWALL, m_nVideoHALL, 0, GL_RGBA, GL_UNSIGNED_BYTE, framepng.data);
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

          m_pTextureJtou->bind(id_textureJtou);
          program.setUniformValue("texture3", id_textureJtou);
          program.setUniformValue("texture3", 3);



    }


     geometries->draw8(&program);

     program.setUniformValue("state", 4);
     geometries->drawPiont(&program,m_nearPosition);
     program.setUniformValue("state", 5);
     if(posList.size()>0)
     {
         for(int i=0;i<posList.size();i++)
         {
            geometries->drawPiont(&program,posList[i]);
         }
     }

     geometries->drawLine(&program);
}
