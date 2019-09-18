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

#ifndef GEOMETRYENGINE_H
#define GEOMETRYENGINE_H

#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#define  PI 3.1415926
#include <QVector2D>
#include <QVector3D>
#include <vector>

#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.inl"

struct VertexData
{
    QVector3D position;
    QVector2D texCoord;
};

struct VertexBuff
{
    public:
    QOpenGLBuffer arrayBuff;
    QOpenGLBuffer indexBuff;
};

class GeometryEngine : protected QOpenGLFunctions
{
public:
    GeometryEngine();
    virtual ~GeometryEngine();

    void drawCubeGeometry(QOpenGLShaderProgram *program);
    void draw2(QOpenGLShaderProgram *program);
    void draw8(QOpenGLShaderProgram *program);


    void drawPiont(QOpenGLShaderProgram *program,glm::vec3 posIntersect);

    void drawLine(QOpenGLShaderProgram *program);

    VertexData*  m_pvertices;
    int*  m_pindices;
    int   m_nsize;

    GLuint quadVAOId;
    GLuint quadVBOId;
    GLuint quadVBOId2;
    GLuint quadVBOId3;

    GLuint quadVAO2Id;
    GLuint quadVBO2Id;
    GLuint quadVBO2Id2;
    GLuint quadVBO2Id3;

    GLuint quadVAO3Id;
    GLuint quadVBO3Id;
    GLuint quadVBO3Id2;
    GLuint quadVBO3Id3;

    GLuint quadVAO4Id;
    GLuint quadVBO4Id;
    GLuint quadVBO4Id2;
    GLuint quadVBO4Id3;

private:

    void SetBall(GLfloat radius, float x = 0, float y = 0, float z = 0);
    void SetTV(GLfloat radius, float x = 0, float y = 0, float z = 0);
    void SetSign(GLfloat radius, float x = 0, float y = 0, float z = 0);

    int cap_H ;//必须大于0,且cap_H应等于cap_W
    int cap_W ;//绘制球体时，每次增加的角度


    std::vector<VertexBuff> m_buff;

};

#endif // GEOMETRYENGINE_H
