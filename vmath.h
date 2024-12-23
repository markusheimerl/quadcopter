#ifndef VMATH_H
#define VMATH_H

#include <math.h>

void multMat3f(const double* a, const double* b, double* result) {
    result[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
    result[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
    result[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

    result[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
    result[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
    result[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

    result[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
    result[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
    result[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];
}

void multMatVec3f(const double* m, const double* v, double* result) {
    result[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    result[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    result[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

void vecToDiagMat3f(const double* v, double* result) {
    result[0] = v[0]; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = v[1]; result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = v[2];
}

int invMat3f(const double* m, double* result) {
    double det = 
        m[0] * (m[4] * m[8] - m[7] * m[5]) -
        m[1] * (m[3] * m[8] - m[5] * m[6]) +
        m[2] * (m[3] * m[7] - m[4] * m[6]);

    if (det == 0) {
        return 0;  // Matrix is not invertible
    }

    double invDet = 1.0f / det;

    result[0] = invDet * (m[4] * m[8] - m[7] * m[5]);
    result[1] = invDet * (m[2] * m[7] - m[1] * m[8]);
    result[2] = invDet * (m[1] * m[5] - m[2] * m[4]);

    result[3] = invDet * (m[5] * m[6] - m[3] * m[8]);
    result[4] = invDet * (m[0] * m[8] - m[2] * m[6]);
    result[5] = invDet * (m[3] * m[2] - m[0] * m[5]);

    result[6] = invDet * (m[3] * m[7] - m[6] * m[4]);
    result[7] = invDet * (m[6] * m[1] - m[0] * m[7]);
    result[8] = invDet * (m[0] * m[4] - m[3] * m[1]);

    return 1;  // Success
}

void transpMat3f(const double* m, double* result) {
    result[0] = m[0]; result[1] = m[3]; result[2] = m[6];
    result[3] = m[1]; result[4] = m[4]; result[5] = m[7];
    result[6] = m[2]; result[7] = m[5]; result[8] = m[8];
}

void identMat3f(double* result) {
    result[0] = 1.0f; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = 1.0f; result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = 1.0f;
}

void xRotMat3f(double rads, double* result) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = 1.0f; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = c;    result[5] = -s;
    result[6] = 0.0f; result[7] = s;    result[8] = c;
}

void yRotMat3f(double rads, double* result) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = c;    result[1] = 0.0f; result[2] = s;
    result[3] = 0.0f; result[4] = 1.0f; result[5] = 0.0f;
    result[6] = -s;   result[7] = 0.0f; result[8] = c;
}

void zRotMat3f(double rads, double* result) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = c;    result[1] = -s;   result[2] = 0.0f;
    result[3] = s;    result[4] = c;    result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = 1.0f;
}

void so3hat(const double* v, double* result) {
    result[0] = 0.0f;  result[1] = -v[2]; result[2] = v[1];
    result[3] = v[2];  result[4] = 0.0f;  result[5] = -v[0];
    result[6] = -v[1]; result[7] = v[0];  result[8] = 0.0f;
}

void so3vee(const double* m, double* result) {
    result[0] = m[7];
    result[1] = m[2];
    result[2] = m[3];
}

void addMat3f(const double* a, const double* b, double* result) {
    for(int i = 0; i < 9; i++) {
        result[i] = a[i] + b[i];
    }
}

void multScalMat3f(double s, const double* m, double* result) {
    for(int i = 0; i < 9; i++) {
        result[i] = s * m[i];
    }
}

void subMat3f(const double* a, const double* b, double* result) {
    for(int i = 0; i < 9; i++) {
        result[i] = a[i] - b[i];
    }
}

void multMat4f(const double* a, const double* b, double* result) {
    result[0] = a[0]*b[0] + a[1]*b[4] + a[2]*b[8] + a[3]*b[12];
    result[1] = a[0]*b[1] + a[1]*b[5] + a[2]*b[9] + a[3]*b[13];
    result[2] = a[0]*b[2] + a[1]*b[6] + a[2]*b[10] + a[3]*b[14];
    result[3] = a[0]*b[3] + a[1]*b[7] + a[2]*b[11] + a[3]*b[15];

    result[4] = a[4]*b[0] + a[5]*b[4] + a[6]*b[8] + a[7]*b[12];
    result[5] = a[4]*b[1] + a[5]*b[5] + a[6]*b[9] + a[7]*b[13];
    result[6] = a[4]*b[2] + a[5]*b[6] + a[6]*b[10] + a[7]*b[14];
    result[7] = a[4]*b[3] + a[5]*b[7] + a[6]*b[11] + a[7]*b[15];

    result[8] = a[8]*b[0] + a[9]*b[4] + a[10]*b[8] + a[11]*b[12];
    result[9] = a[8]*b[1] + a[9]*b[5] + a[10]*b[9] + a[11]*b[13];
    result[10] = a[8]*b[2] + a[9]*b[6] + a[10]*b[10] + a[11]*b[14];
    result[11] = a[8]*b[3] + a[9]*b[7] + a[10]*b[11] + a[11]*b[15];

    result[12] = a[12]*b[0] + a[13]*b[4] + a[14]*b[8] + a[15]*b[12];
    result[13] = a[12]*b[1] + a[13]*b[5] + a[14]*b[9] + a[15]*b[13];
    result[14] = a[12]*b[2] + a[13]*b[6] + a[14]*b[10] + a[15]*b[14];
    result[15] = a[12]*b[3] + a[13]*b[7] + a[14]*b[11] + a[15]*b[15];
}

int inv4Mat4f(const double* m, double* result) {
    double s0 = m[0] * m[5] - m[4] * m[1];
    double s1 = m[0] * m[6] - m[4] * m[2];
    double s2 = m[0] * m[7] - m[4] * m[3];
    double s3 = m[1] * m[6] - m[5] * m[2];
    double s4 = m[1] * m[7] - m[5] * m[3];
    double s5 = m[2] * m[7] - m[6] * m[3];

    double c5 = m[10] * m[15] - m[14] * m[11];
    double c4 = m[9] * m[15] - m[13] * m[11];
    double c3 = m[9] * m[14] - m[13] * m[10];
    double c2 = m[8] * m[15] - m[12] * m[11];
    double c1 = m[8] * m[14] - m[12] * m[10];
    double c0 = m[8] * m[13] - m[12] * m[9];

    double det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;

    if (det == 0.0f) return 0;

    double invdet = 1.0f / det;

    result[0] = (m[5] * c5 - m[6] * c4 + m[7] * c3) * invdet;
    result[1] = (-m[1] * c5 + m[2] * c4 - m[3] * c3) * invdet;
    result[2] = (m[13] * s5 - m[14] * s4 + m[15] * s3) * invdet;
    result[3] = (-m[9] * s5 + m[10] * s4 - m[11] * s3) * invdet;

    result[4] = (-m[4] * c5 + m[6] * c2 - m[7] * c1) * invdet;
    result[5] = (m[0] * c5 - m[2] * c2 + m[3] * c1) * invdet;
    result[6] = (-m[12] * s5 + m[14] * s2 - m[15] * s1) * invdet;
    result[7] = (m[8] * s5 - m[10] * s2 + m[11] * s1) * invdet;

    result[8] = (m[4] * c4 - m[5] * c2 + m[7] * c0) * invdet;
    result[9] = (-m[0] * c4 + m[1] * c2 - m[3] * c0) * invdet;
    result[10] = (m[12] * s4 - m[13] * s2 + m[15] * s0) * invdet;
    result[11] = (-m[8] * s4 + m[9] * s2 - m[11] * s0) * invdet;

    result[12] = (-m[4] * c3 + m[5] * c1 - m[6] * c0) * invdet;
    result[13] = (m[0] * c3 - m[1] * c1 + m[2] * c0) * invdet;
    result[14] = (-m[12] * s3 + m[13] * s1 - m[14] * s0) * invdet;
    result[15] = (m[8] * s3 - m[9] * s1 + m[10] * s0) * invdet;

    return 1;
}

int inv3Mat4f(const double* m, double* result) {
    double a = m[0], b = m[1], c = m[2];
    double d = m[4], e = m[5], f = m[6];
    double g = m[8], h = m[9], i = m[10];

    double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);

    if (det == 0.0f) return 0;

    double invDet = 1.0f / det;

    result[0] = (e * i - f * h) * invDet;
    result[1] = (c * h - b * i) * invDet;
    result[2] = (b * f - c * e) * invDet;
    result[3] = 0.0f;

    result[4] = (f * g - d * i) * invDet;
    result[5] = (a * i - c * g) * invDet;
    result[6] = (c * d - a * f) * invDet;
    result[7] = 0.0f;

    result[8] = (d * h - e * g) * invDet;
    result[9] = (b * g - a * h) * invDet;
    result[10] = (a * e - b * d) * invDet;
    result[11] = 0.0f;

    result[12] = 0.0f;
    result[13] = 0.0f;
    result[14] = 0.0f;
    result[15] = 1.0f;

    return 1;
}

void transp4Mat4f(const double* m, double* result) {
    result[0] = m[0];  result[1] = m[4];  result[2] = m[8];   result[3] = m[12];
    result[4] = m[1];  result[5] = m[5];  result[6] = m[9];   result[7] = m[13];
    result[8] = m[2];  result[9] = m[6];  result[10] = m[10]; result[11] = m[14];
    result[12] = m[3]; result[13] = m[7]; result[14] = m[11]; result[15] = m[15];
}

void identMat4f(double* result) {
    result[0] = 1.0f; result[1] = 0.0f;  result[2] = 0.0f;  result[3] = 0.0f;
    result[4] = 0.0f; result[5] = 1.0f;  result[6] = 0.0f;  result[7] = 0.0f;
    result[8] = 0.0f; result[9] = 0.0f;  result[10] = 1.0f; result[11] = 0.0f;
    result[12] = 0.0f; result[13] = 0.0f; result[14] = 0.0f; result[15] = 1.0f;
}

void translMat4f(double tx, double ty, double tz, double* result) {
    result[0] = 1.0f; result[1] = 0.0f;  result[2] = 0.0f;  result[3] = 0.0f;
    result[4] = 0.0f; result[5] = 1.0f;  result[6] = 0.0f;  result[7] = 0.0f;
    result[8] = 0.0f; result[9] = 0.0f;  result[10] = 1.0f; result[11] = 0.0f;
    result[12] = tx;  result[13] = ty;   result[14] = tz;   result[15] = 1.0f;
}

void xRotMat4f(double rads, double* result) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = 1.0f; result[1] = 0.0f; result[2] = 0.0f;  result[3] = 0.0f;
    result[4] = 0.0f; result[5] = c;    result[6] = -s;    result[7] = 0.0f;
    result[8] = 0.0f; result[9] = s;    result[10] = c;    result[11] = 0.0f;
    result[12] = 0.0f; result[13] = 0.0f; result[14] = 0.0f; result[15] = 1.0f;
}

void yRotMat4f(double rads, double* result) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = c;    result[1] = 0.0f; result[2] = s;    result[3] = 0.0f;
    result[4] = 0.0f; result[5] = 1.0f; result[6] = 0.0f; result[7] = 0.0f;
    result[8] = -s;   result[9] = 0.0f; result[10] = c;   result[11] = 0.0f;
    result[12] = 0.0f; result[13] = 0.0f; result[14] = 0.0f; result[15] = 1.0f;
}

void zRotMat4f(double rads, double* result) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = c;    result[1] = -s;   result[2] = 0.0f; result[3] = 0.0f;
    result[4] = s;    result[5] = c;    result[6] = 0.0f; result[7] = 0.0f;
    result[8] = 0.0f; result[9] = 0.0f; result[10] = 1.0f; result[11] = 0.0f;
    result[12] = 0.0f; result[13] = 0.0f; result[14] = 0.0f; result[15] = 1.0f;
}

void scaleMat4f(double sx, double sy, double sz, double* result) {
    result[0] = sx;   result[1] = 0.0f;  result[2] = 0.0f;  result[3] = 0.0f;
    result[4] = 0.0f; result[5] = sy;    result[6] = 0.0f;  result[7] = 0.0f;
    result[8] = 0.0f; result[9] = 0.0f;  result[10] = sz;   result[11] = 0.0f;
    result[12] = 0.0f; result[13] = 0.0f; result[14] = 0.0f; result[15] = 1.0f;
}

void modelMat4f(double tx, double ty, double tz, 
                double rx, double ry, double rz, 
                double sx, double sy, double sz, 
                double* result) {
    double temp1[16], temp2[16], rotX[16], rotY[16], rotZ[16], scale[16], transl[16];
    
    identMat4f(result);
    translMat4f(tx, ty, tz, transl);
    xRotMat4f(rx, rotX);
    yRotMat4f(ry, rotY);
    zRotMat4f(rz, rotZ);
    scaleMat4f(sx, sy, sz, scale);
    
    multMat4f(transl, result, temp1);
    multMat4f(rotX, rotY, temp2);
    multMat4f(temp2, rotZ, temp1);
    multMat4f(temp1, result, temp2);
    multMat4f(scale, temp2, result);
}

void perspecMat4f(double fov, double aspect, double near, double far, double* result) {
    double f = 1.0f / tanf(M_PI * 0.5f - 0.5f * fov);
    double rangeInv = 1.0f / (near - far);

    result[0] = f / aspect;
    result[1] = 0.0f;
    result[2] = 0.0f;
    result[3] = 0.0f;

    result[4] = 0.0f;
    result[5] = f;
    result[6] = 0.0f;
    result[7] = 0.0f;

    result[8] = 0.0f;
    result[9] = 0.0f;
    result[10] = (near + far) * rangeInv;
    result[11] = -1.0f;

    result[12] = 0.0f;
    result[13] = 0.0f;
    result[14] = near * far * rangeInv * 2.0f;
    result[15] = 0.0f;
}

void multMatVec4f(const double* m, const double* v, double* result) {
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2] + m[3]*v[3];
    result[1] = m[4]*v[0] + m[5]*v[1] + m[6]*v[2] + m[7]*v[3];
    result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
    result[3] = m[12]*v[0] + m[13]*v[1] + m[14]*v[2] + m[15]*v[3];
}

void crossVec3f(const double* v1, const double* v2, double* result) {
    result[0] = v1[1] * v2[2] - v1[2] * v2[1];
    result[1] = v1[2] * v2[0] - v1[0] * v2[2];
    result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void addScalVec3f(double s, const double* v, double* result) {
    result[0] = v[0] + s;
    result[1] = v[1] + s;
    result[2] = v[2] + s;
}

void multScalVec3f(double s, const double* v, double* result) {
    result[0] = v[0] * s;
    result[1] = v[1] * s;
    result[2] = v[2] * s;
}

void addVec3f(const double* v1, const double* v2, double* result) {
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
    result[2] = v1[2] + v2[2];
}

void subVec3f(const double* v1, const double* v2, double* result) {
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    result[2] = v1[2] - v2[2];
}

double dotVec3f(const double* v1, const double* v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

void normVec3f(const double* v, double* result) {
    double magnitude = sqrtf(dotVec3f(v, v));
    result[0] = v[0] / magnitude;
    result[1] = v[1] / magnitude;
    result[2] = v[2] / magnitude;
}

#endif // VMATH_H