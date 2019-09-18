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
#include"math.h"
#include "geometryengine.h"



//! [0]
GeometryEngine::GeometryEngine()
{
    initializeOpenGLFunctions();
    cap_H = 1;
    cap_W = 1;
    SetBall(4.0);
    SetTV(4.0);
    SetSign(4.0);

}

GeometryEngine::~GeometryEngine()
{
    if(m_pvertices)
    {
        delete[] m_pvertices;
        m_pvertices=nullptr;
    }
    if(m_pindices)
    {
        delete[] m_pindices;
        m_pindices=nullptr;
    }
}


void GeometryEngine::SetBall(GLfloat radius, float x , float y , float z)
{
    m_pvertices=new VertexData[(180 / cap_H) * (360 / cap_W) * 6];
    m_pindices=new int[(180 / cap_H) * (360 / cap_W) * 6];
    VertexData* vertices=m_pvertices;

    int* indices=m_pindices;

    float r = radius;//球体半径
    double d = cap_H * PI / 180;//每次递增的弧度
    int vnum=0;

    for (int i = 0; i < 180; i += cap_H)
    {
        double d1 = i * PI / 180;
        for (int j = 0; j < 360; j += cap_W)
        {
            //获得球体上切分的超小片矩形的顶点坐标（两个三角形组成，所以有六点顶点）
            double d2 = j * PI / 180;
            vertices[vnum].position.setX((float)(x + r * sin(d1 + d) * cos(d2 + d)));
            vertices[vnum].position.setY((float)(y + r * sin(d1 + d) * sin(d2 + d)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1 + d)));

            //获得球体上切分的超小片三角形的纹理坐标
            vertices[vnum].texCoord.setX(1-(j + cap_W) * 1.0f / 360);
            vertices[vnum].texCoord.setY((i + cap_H) * 1.0f / 180);
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX((float)(x + r * sin(d1) * cos(d2)));
            vertices[vnum].position.setY((float)(y + r * sin(d1) * sin(d2)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1)));

            vertices[vnum].texCoord.setX( 1-j * 1.0f / 360);
            vertices[vnum].texCoord.setY(i * 1.0f / 180);
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX( (float)(x + r * sin(d1) * cos(d2 + d)));
           vertices[vnum].position.setY( (float)(y + r * sin(d1) * sin(d2 + d)));
           vertices[vnum].position.setZ( (float)(z + r * cos(d1)));


            vertices[vnum].texCoord.setX(1- (j + cap_W) * 1.0f / 360);
            vertices[vnum].texCoord.setY(i * 1.0f / 180);
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX((float)(x + r * sin(d1 + d) * cos(d2 + d)));
            vertices[vnum].position.setY((float)(y + r * sin(d1 + d) * sin(d2 + d)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1 + d)));


            vertices[vnum].texCoord.setX(1- (j + cap_W) * 1.0f / 360);
            vertices[vnum].texCoord.setY((i + cap_H) * 1.0f / 180);
            indices[vnum]=vnum;

            vnum++;

            vertices[vnum].position.setX((float)(x + r * sin(d1 + d) * cos(d2)));
            vertices[vnum].position.setY( (float)(y + r * sin(d1 + d) * sin(d2)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1 + d)));


            vertices[vnum].texCoord.setX(1-j * 1.0f / 360);
            vertices[vnum].texCoord.setY((i + cap_H) * 1.0f / 180);
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX((float)(x + r * sin(d1) * cos(d2)));
            vertices[vnum].position.setY((float)(y + r * sin(d1) * sin(d2)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1)));


            vertices[vnum].texCoord.setX(1-j * 1.0f / 360);
            vertices[vnum].texCoord.setY(i * 1.0f / 180);
            indices[vnum]=vnum;

            vnum++;
        }
    }

    m_nsize=vnum;
    //qDebug() << vnum;

    QOpenGLBuffer arrayBuff;
    QOpenGLBuffer indexBuff(QOpenGLBuffer::IndexBuffer);
    arrayBuff.create();
    indexBuff.create();

    arrayBuff.bind();
    arrayBuff.allocate(vertices, (180 / cap_H) * (360 / cap_W) * 6 * sizeof(VertexData));

    indexBuff.bind();
    indexBuff.allocate(indices, (180 / cap_H) * (360 / cap_W) * 6 * sizeof(int));

    VertexBuff vf1;
    vf1.arrayBuff=arrayBuff;
    vf1.indexBuff=indexBuff;
    m_buff.push_back(vf1);

}
void GeometryEngine:: SetTV(GLfloat radius, float x, float y , float z)
{
    GLfloat r=radius;
    VertexData* vertices=new VertexData[6];
     int* indices=new int[6];

       int vnum=0;

        int i=90,j=0;
         double dx = 16*cap_H * PI / 180;//每次递增的弧度
          double dy = 9*cap_H * PI / 180;//每次递增的弧度
          double d1 = i * PI / 180;
          //获得球体上切分的超小片矩形的顶点坐标（两个三角形组成，所以有六点顶点）
           double d2 = j * PI / 180;
           double i0=0,j0=0;

            vertices[vnum].position.setX((float)(x + r * sin(d1 + dy) * cos(d2 + dx)));
            vertices[vnum].position.setY((float)(y + r * sin(d1 + dy) * sin(d2 + dx)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1 + dy)));

            //获得球体上切分的超小片三角形的纹理坐标
            vertices[vnum].texCoord.setX(1-(j0+1));
            vertices[vnum].texCoord.setY((i0 + 1));
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX((float)(x + r * sin(d1) * cos(d2)));
            vertices[vnum].position.setY((float)(y + r * sin(d1) * sin(d2)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1)));

            vertices[vnum].texCoord.setX(1- j0 );
            vertices[vnum].texCoord.setY(i0 );
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX( (float)(x + r * sin(d1) * cos(d2 + dx)));
            vertices[vnum].position.setY( (float)(y + r * sin(d1) * sin(d2 + dx)));
            vertices[vnum].position.setZ( (float)(z + r * cos(d1)));


            vertices[vnum].texCoord.setX(1-  (j0 +1));
            vertices[vnum].texCoord.setY( i0 );
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX((float)(x + r * sin(d1 + dy) * cos(d2 + dx)));
            vertices[vnum].position.setY((float)(y + r * sin(d1 + dy) * sin(d2 + dx)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1 + dy)));


            vertices[vnum].texCoord.setX(1- (j0 + 1) );
            vertices[vnum].texCoord.setY(  (i0 + 1));
            indices[vnum]=vnum;

            vnum++;

            vertices[vnum].position.setX((float)(x + r * sin(d1 + dy) * cos(d2)));
            vertices[vnum].position.setY( (float)(y + r * sin(d1 + dy) * sin(d2)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1 + dy)));


            vertices[vnum].texCoord.setX( 1-j0 );
            vertices[vnum].texCoord.setY((i0 +1));
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX((float)(x + r * sin(d1) * cos(d2)));
            vertices[vnum].position.setY((float)(y + r * sin(d1) * sin(d2)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1)));


            vertices[vnum].texCoord.setX(1- j0 );
            vertices[vnum].texCoord.setY(  i0 );
            indices[vnum]=vnum;

            //vnum++;

     // qDebug() << vnum;

      QOpenGLBuffer arrayBuff;
      QOpenGLBuffer indexBuff(QOpenGLBuffer::IndexBuffer);
      arrayBuff.create();
      indexBuff.create();

      arrayBuff.bind();
      arrayBuff.allocate(vertices, 6 * sizeof(VertexData));

      indexBuff.bind();
      indexBuff.allocate(indices,6 * sizeof(int));

      VertexBuff vf1;
      vf1.arrayBuff=arrayBuff;
      vf1.indexBuff=indexBuff;
      m_buff.push_back(vf1);

        if(vertices)
        {
            delete[] vertices;
            vertices=nullptr;
        }
        if(indices)
        {
            delete[] indices;
            indices=nullptr;
        }

}


void GeometryEngine:: SetSign(GLfloat radius, float x, float y , float z)
{
        GLfloat r=radius;
        VertexData* vertices=new VertexData[6];
        int* indices=new int[6];
        int vnum=0;

         int i=70,j=180;
         double dx = 2*cap_H * PI / 180;//每次递增的弧度
          double dy = 2*cap_H * PI / 180;//每次递增的弧度
          double d1 = i * PI / 180;
          //获得球体上切分的超小片矩形的顶点坐标（两个三角形组成，所以有六点顶点）
           double d2 = j * PI / 180;
           double i0=0,j0=0;

            vertices[vnum].position.setX((float)(x + r * sin(d1 + dy) * cos(d2 + dx)));
            vertices[vnum].position.setY((float)(y + r * sin(d1 + dy) * sin(d2 + dx)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1 + dy)));

            //获得球体上切分的超小片三角形的纹理坐标
            vertices[vnum].texCoord.setX(1-(j0+1));
            vertices[vnum].texCoord.setY((i0 + 1));
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX((float)(x + r * sin(d1) * cos(d2)));
            vertices[vnum].position.setY((float)(y + r * sin(d1) * sin(d2)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1)));

            vertices[vnum].texCoord.setX(1- j0 );
            vertices[vnum].texCoord.setY(i0 );
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX( (float)(x + r * sin(d1) * cos(d2 + dx)));
            vertices[vnum].position.setY( (float)(y + r * sin(d1) * sin(d2 + dx)));
            vertices[vnum].position.setZ( (float)(z + r * cos(d1)));


            vertices[vnum].texCoord.setX(1-  (j0 +1));
            vertices[vnum].texCoord.setY( i0 );
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX((float)(x + r * sin(d1 + dy) * cos(d2 + dx)));
            vertices[vnum].position.setY((float)(y + r * sin(d1 + dy) * sin(d2 + dx)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1 + dy)));


            vertices[vnum].texCoord.setX(1- (j0 + 1) );
            vertices[vnum].texCoord.setY(  (i0 + 1));
            indices[vnum]=vnum;

            vnum++;

            vertices[vnum].position.setX((float)(x + r * sin(d1 + dy) * cos(d2)));
            vertices[vnum].position.setY( (float)(y + r * sin(d1 + dy) * sin(d2)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1 + dy)));


            vertices[vnum].texCoord.setX( 1-j0 );
            vertices[vnum].texCoord.setY((i0 +1));
            indices[vnum]=vnum;

            vnum++;
            vertices[vnum].position.setX((float)(x + r * sin(d1) * cos(d2)));
            vertices[vnum].position.setY((float)(y + r * sin(d1) * sin(d2)));
            vertices[vnum].position.setZ((float)(z + r * cos(d1)));


            vertices[vnum].texCoord.setX(1- j0 );
            vertices[vnum].texCoord.setY(  i0 );
            indices[vnum]=vnum;

            //vnum++;

      //qDebug() << vnum;


      QOpenGLBuffer arrayBuff;
      QOpenGLBuffer indexBuff(QOpenGLBuffer::IndexBuffer);
      arrayBuff.create();
      indexBuff.create();

      arrayBuff.bind();
      arrayBuff.allocate(vertices, 6 * sizeof(VertexData));

      indexBuff.bind();
      indexBuff.allocate(indices,6 * sizeof(int));

      VertexBuff vf1;
      vf1.arrayBuff=arrayBuff;
      vf1.indexBuff=indexBuff;
      m_buff.push_back(vf1);


    if(vertices)
    {
        delete[] vertices;
        vertices=nullptr;
    }
    if(indices)
    {
        delete[] indices;
        indices=nullptr;
    }

}




void GeometryEngine::drawCubeGeometry(QOpenGLShaderProgram *program)
{
    m_buff[0].arrayBuff.bind();
    //m_buff[0].indexBuff.bind();

    quintptr offset = 0;

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int texcoordLocation = program->attributeLocation("a_texcoord");
    program->enableAttributeArray(texcoordLocation);
    program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

  //  glDrawElements(GL_TRIANGLES, (180.0 / cap_H) * (360.0 / cap_W) * 6, GL_UNSIGNED_INT, 0);
    glDrawArrays(GL_TRIANGLES, 0, (180.0 / cap_H) * (360.0 / cap_W) * 6);
}

void GeometryEngine::draw2(QOpenGLShaderProgram *program)
{
    // Tell OpenGL which VBOs to use


    m_buff[1].arrayBuff.bind();
    //m_buff[1].indexBuff.bind();
    // Offset for position
    quintptr offset = 0;

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int texcoordLocation = program->attributeLocation("a_texcoord");
    program->enableAttributeArray(texcoordLocation);
    program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

    // Draw cube geometry using indices from VBO 1

     glDrawArrays(GL_TRIANGLES, 0, 6);
   //glDrawElements(GL_TRIANGLES,6, GL_UNSIGNED_INT, 0);

}


void GeometryEngine::draw8(QOpenGLShaderProgram *program)
{
    // Tell OpenGL which VBOs to use

    m_buff[2].arrayBuff.bind();
   // m_buff[2].indexBuff.bind();
    // Offset for position
    quintptr offset = 0;

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int texcoordLocation = program->attributeLocation("a_texcoord");
    program->enableAttributeArray(texcoordLocation);
    program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

    // Draw cube geometry using indices from VBO 1

    //glDrawElements(GL_TRIANGLES,6, GL_UNSIGNED_INT, 0);
      glDrawArrays(GL_TRIANGLES, 0, 6);

}


void GeometryEngine::drawPiont(QOpenGLShaderProgram *program,glm::vec3 posIntersect)
{
     // Tell OpenGL which VBOs to use
     VertexData* vertices=new VertexData[3];
     int* indices=new int[3];
     int vnum=0;
     float dx=0.1;
     float dy=0.1;
     float dz=0.1;
     vertices[vnum].position.setX((float)(posIntersect.x)-dx);
     vertices[vnum].position.setY((float)(posIntersect.y));
     vertices[vnum].position.setZ((float)(posIntersect.z));

     //获得球体上切分的超小片三角形的纹理坐标
     vertices[vnum].texCoord.setX(1);
     vertices[vnum].texCoord.setY(0);
     indices[vnum]=vnum;

     vnum++;
     vertices[vnum].position.setX((float)(posIntersect.x)+dx);
     vertices[vnum].position.setY((float)(posIntersect.y));
     vertices[vnum].position.setZ((float)(posIntersect.z));

     vertices[vnum].texCoord.setX(0 );
     vertices[vnum].texCoord.setY(0 );
     indices[vnum]=vnum;

     vnum++;
     vertices[vnum].position.setX((float)(posIntersect.x));
     vertices[vnum].position.setY((float)(posIntersect.y)+dy);
     vertices[vnum].position.setZ((float)(posIntersect.z));


     vertices[vnum].texCoord.setX(0);
     vertices[vnum].texCoord.setY(1);
     indices[vnum]=vnum;

    QOpenGLBuffer arrayBuff;
   // QOpenGLBuffer indexBuff(QOpenGLBuffer::IndexBuffer);
    arrayBuff.create();
   // indexBuff.create();

    arrayBuff.bind();
    arrayBuff.allocate(vertices, 3* sizeof(VertexData));

    //indexBuff.bind();
    //indexBuff.allocate(indices, 3* sizeof(int));

    //m_buff[2].arrayBuff.bind();
   // m_buff[2].indexBuff.bind();
    // Offset for position
    arrayBuff.bind();
    //indexBuff.bind();
    quintptr offset = 0;

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = program->attributeLocation("a_position");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

    // Offset for texture coordinate
    offset += sizeof(QVector3D);

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int texcoordLocation = program->attributeLocation("a_texcoord");
    program->enableAttributeArray(texcoordLocation);
    program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

    // Draw cube geometry using indices from VBO 1
    //qDebug()  << posIntersect.x << "&&&" <<posIntersect.y << "&&&" << posIntersect.z <<"&&&"<<sqrt(posIntersect.x*posIntersect.x+posIntersect.y*posIntersect.y+posIntersect.z*posIntersect.z)  ;

  //  glDrawElements(GL_TRIANGLES,3, GL_UNSIGNED_INT, 0);
     glDrawArrays(GL_TRIANGLES,0,3);

}


void GeometryEngine:: drawLine(QOpenGLShaderProgram *program)
{
    //glPointSize(25);
    // Tell OpenGL which VBOs to use
    VertexData* vertices=new VertexData[6];
    int* indices=new int[6];
    int vnum=0;
    float dx=0.1;
    float dy=0.1;
    float dz=0.1;
    vertices[vnum].position.setX((float)(0.0));
    vertices[vnum].position.setY((float)(0.0));
    vertices[vnum].position.setZ((float)(0.0));

    //获得球体上切分的超小片三角形的纹理坐标
    vertices[vnum].texCoord.setX(1);
    vertices[vnum].texCoord.setY(0);
    indices[vnum]=vnum;

    vnum++;
    vertices[vnum].position.setX((float)(0+2));
    vertices[vnum].position.setY((float)(0));
    vertices[vnum].position.setZ((float)(0));

    vertices[vnum].texCoord.setX(0 );
    vertices[vnum].texCoord.setY(0 );
    indices[vnum]=vnum;

    vnum++;
    vertices[vnum].position.setX((float)(0));
    vertices[vnum].position.setY((float)(0));
    vertices[vnum].position.setZ((float)(0));


    vertices[vnum].texCoord.setX(0);
    vertices[vnum].texCoord.setY(1);
    indices[vnum]=vnum;

    vnum++;
    vertices[vnum].position.setX((float)(0));
    vertices[vnum].position.setY((float)(0+2));
    vertices[vnum].position.setZ((float)(0));

    vertices[vnum].texCoord.setX(0 );
    vertices[vnum].texCoord.setY(0 );
    indices[vnum]=vnum;

    vnum++;
    vertices[vnum].position.setX((float)(0));
    vertices[vnum].position.setY((float)(0));
    vertices[vnum].position.setZ((float)(0));


    vertices[vnum].texCoord.setX(0);
    vertices[vnum].texCoord.setY(1);
    indices[vnum]=vnum;


    vnum++;
    vertices[vnum].position.setX((float)(0));
    vertices[vnum].position.setY((float)(0));
    vertices[vnum].position.setZ((float)(0+2));


    vertices[vnum].texCoord.setX(0);
    vertices[vnum].texCoord.setY(1);
    indices[vnum]=vnum;

   QOpenGLBuffer arrayBuff;
   QOpenGLBuffer indexBuff(QOpenGLBuffer::IndexBuffer);
   arrayBuff.create();
   indexBuff.create();

   arrayBuff.bind();
   arrayBuff.allocate(vertices, 6* sizeof(VertexData));

   indexBuff.bind();
   indexBuff.allocate(indices, 6* sizeof(int));

   //m_buff[2].arrayBuff.bind();
  // m_buff[2].indexBuff.bind();
   // Offset for position
   arrayBuff.bind();
   indexBuff.bind();
   quintptr offset = 0;

   // Tell OpenGL programmable pipeline how to locate vertex position data
   int vertexLocation = program->attributeLocation("a_position");
   program->enableAttributeArray(vertexLocation);
   program->setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(VertexData));

   // Offset for texture coordinate
   offset += sizeof(QVector3D);

   // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
   int texcoordLocation = program->attributeLocation("a_texcoord");
   program->enableAttributeArray(texcoordLocation);
   program->setAttributeBuffer(texcoordLocation, GL_FLOAT, offset, 2, sizeof(VertexData));

   // Draw cube geometry using indices from VBO 1
   //qDebug()  << posIntersect.x << "&&&" <<posIntersect.y << "&&&" << posIntersect.z <<"&&&"<<sqrt(posIntersect.x*posIntersect.x+posIntersect.y*posIntersect.y+posIntersect.z*posIntersect.z)  ;

  // glDrawElements(GL_LINES,6, GL_UNSIGNED_INT, 0);

    //glDrawElements(GL_POINTS,6, GL_UNSIGNED_INT, 0);
    glDrawArrays(GL_POINTS,0,6);

}

