/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features for OpenDRIVE
 * Purpose:  Implementation of common transformation matrix functions.
 * Author:   Michael Scholz, michael.scholz@dlr.de, German Aerospace Center (DLR)
 *
 ******************************************************************************
 * Copyright 2017 German Aerospace Center (DLR), Institute of Transportation Systems 
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ****************************************************************************/

#include "MatrixTransformations2D.h"
#include <math.h>

MatrixTransformations::MatrixTransformations(void)
{
}

MatrixTransformations::~MatrixTransformations(void)
{
}

void MatrixTransformations::initMatrix(Matrix2D& matrix) {
    matrix[0][0] = 1.0; matrix[0][1] = 0.0;	matrix[0][2] = 0.0;
    matrix[1][0] = 0.0; matrix[1][1] = 1.0;	matrix[1][2] = 0.0;
    matrix[2][0] = 0.0; matrix[2][1] = 0.0; matrix[2][2] = 1.0;
}

void MatrixTransformations::translate(Matrix2D& matrix, const double xOffset, const double yOffset) {
    Matrix2D trans, result;

    trans[0][0] = 1.0;     trans[0][1] = 0.0;     trans[0][2] = 0.0;
    trans[1][0] = 0.0;     trans[1][1] = 1.0;     trans[1][2] = 0.0;
    trans[2][0] = xOffset; trans[2][1] = yOffset; trans[2][2] = 1.0;

    multiplyMatrix(trans, matrix, result);
    copyMatrix(result, matrix); 
}

void MatrixTransformations::rotate(Matrix2D& matrix, const double radians) {
    Matrix2D rot, result;

    rot[0][0] = cos(radians);  rot[0][1] = sin(radians); rot[0][2] = 0.0;
    rot[1][0] = -sin(radians); rot[1][1] = cos(radians); rot[1][2] = 0.0;
    rot[2][0] = 0.0;           rot[2][1] = 0.0;          rot[2][2] = 1.0;

    multiplyMatrix(rot, matrix, result);
    copyMatrix(result, matrix); 
}

void MatrixTransformations::reflectAtX(Matrix2D& matrix) {
    Matrix2D refl, result;
    
    refl[0][0] = 1.0; refl[0][1] = 0.0;  refl[0][2] = 0.0;
    refl[1][0] = 0.0; refl[1][1] = -1.0; refl[1][2] = 0.0;
    refl[2][0] = 0.0; refl[2][1] = 0.0;  refl[2][2] = 1.0;
    
    multiplyMatrix(refl, matrix, result);
    copyMatrix(result, matrix); 
}

void MatrixTransformations::reflectAtY(Matrix2D& matrix) {
    Matrix2D refl, result;
    
    refl[0][0] = -1.0; refl[0][1] = 0.0; refl[0][2] = 0.0;
    refl[1][0] = 0.0;  refl[1][1] = 1.0; refl[1][2] = 0.0;
    refl[2][0] = 0.0;  refl[2][1] = 0.0; refl[2][2] = 1.0;
    
    multiplyMatrix(refl, matrix, result);
    copyMatrix(result, matrix); 
}

void MatrixTransformations::multiplyMatrix(const Matrix2D& matrix1, const Matrix2D& matrix2, Matrix2D& product) {
    for (int x = 0; x < 3; ++x) {
        for (int y = 0; y < 3; ++y) {
            double sum = 0;
            for (int z = 0; z < 3; ++z) {
                sum += matrix1[x][z] * matrix2[z][y];
            }
            product[x][y] = sum;
        }
    }
}

void MatrixTransformations::copyMatrix(const Matrix2D& src, Matrix2D& dst) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            dst[i][j] = src[i][j];
        }
    }
}

void MatrixTransformations::transform(Vector2D& vector, const Matrix2D& matrix) {
    Vector2D transformedVector;
    transformedVector.x = vector.x * matrix[0][0] + vector.y * matrix[1][0] + vector.w * matrix[2][0];
    transformedVector.y = vector.x * matrix[0][1] + vector.y * matrix[1][1] + vector.w * matrix[2][1];
    transformedVector.w = vector.x * matrix[0][2] + vector.y * matrix[1][2] + vector.w * matrix[2][2];
    vector = transformedVector;
}