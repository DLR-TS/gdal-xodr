/******************************************************************************
 * $Id$
 *
 * Project:  OpenGIS Simple Features for OpenDRIVE
 * Purpose:  Declaration of common transformation matrix functions.
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

#ifndef _MATRIXTRANSFORMATIONS2D_H_INCLUDED
#define _MATRIXTRANSFORMATIONS2D_H_INCLUDED

/**
 * A 2D vector with w-component.
 */
typedef struct vector {
    double x, y, w;
} Vector2D;

/**
 * A 2D transformation matrix being 3x3.
 */
typedef double Matrix2D[3][3];

/**
 * Bundles transformation matrix functions commonly used in computer graphics applications. These functions allow 
 * easy concatenation of geometry transformations. Should be moved to GEOS one day.
 */
class MatrixTransformations {
private:
    /* http://www.informit.com/articles/article.aspx?p=98117&seqNum=4 */
    MatrixTransformations(void);
    ~MatrixTransformations(void);
public:
    /**
     * Initialises an identity matrix.
     * @param matrix The matrix to initialise.
     */
    static void initMatrix(Matrix2D& matrix);
    
    /**
     * Adds a translation to the given matrix.
     * @param matrix The matrix to add the translation to.
     * @param xOffset Translation offset along x-axis.
     * @param yOffset Translation offset along y-axis.
     */
    static void translate(Matrix2D& matrix, const double xOffset, const double yOffset);
    
    /**
     * Adds a rotation to the given matrix. The rotation is performed counter-clockwisely, i.e. around an 
     * imaginary z-axis in a right-handed coordinate system.
     * @param matrix The matrix to add the rotation to.
     * @param radians Amount of radians to rotate.
     */
    static void rotate(Matrix2D& matrix, const double radians);
    
    /**
     * Adds a reflection at the x-axis to the given matrix. Basically y-values are being negated.
     * @param matrix The matrix to add the reflection to.
     */
    static void reflectAtX(Matrix2D& matrix);
    
    /**
     * Adds a reflection at the y-axis to the given matrix. Basically x-values are being negated.
     * @param matrix The matrix to add the reflection to.
     */
    static void reflectAtY(Matrix2D& matrix);    
    
    /**
     * Multiplies two transformation matrices resembling the concatenation of two geometric transformations.
     * @param matrix1 First matrix.
     * @param matrix2 Second matrix.
     * @param product Product of the multiplication.
     */
    static void multiplyMatrix(const Matrix2D& matrix1, const Matrix2D& matrix2, Matrix2D& product);
    
    /**
     * Copies one matrix to another.
     * @param src Source matrix.
     * @param dst Destination matrix.
     */
    static void copyMatrix(const Matrix2D& src, Matrix2D& dst);
    
    /**
     * Transforms a 2D vector by multiplying it with the desired 2D transformation matrix.
     * @param vector The 2D vector (1x3) to transform.
     * @param matrix The 2D transformation matrix (3x3).
     */
    static void transform(Vector2D& vector, const Matrix2D& matrix);
};

#endif /* ndef _MATRIXTRANSFORMATIONS2D_H_INCLUDED */