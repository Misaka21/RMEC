/**
 ******************************************************************************
 * @file    matrix.cpp/h
 * @brief   Matrix/vector calculation. 矩阵/向量运算
 * @author  Spoon Guan
 ******************************************************************************
 * Copyright (c) 2023 Team JiaoLong-SJTU
 * All rights reserved.
 ******************************************************************************
 */

#ifndef MATRIX_H
#define MATRIX_H

#include "arm_math.h"

// Matrix class
template <int _rows, int _cols>
class Matrixf {
 public:
  // Constructor without input data
  Matrixf(void) : rows_(_rows), cols_(_cols) {
    arm_mat_init_f32(&arm_mat_, _rows, _cols, this->data_);
  }
  // Constructor with input data
  Matrixf(float data[_rows * _cols]) : Matrixf() {
    memcpy(this->data_, data, _rows * _cols * sizeof(float));
  }
  // Copy constructor
  Matrixf(const Matrixf<_rows, _cols>& mat) : Matrixf() {
    memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
  }
  // Destructor
  ~Matrixf(void) {}

  // Row size
  int rows(void) { return _rows; }
  // Column size
  int cols(void) { return _cols; }

  // Element
  float* operator[](const int& row) { return &this->data_[row * _cols]; }

  // Operators
  Matrixf<_rows, _cols>& operator=(const Matrixf<_rows, _cols> mat) {
    memcpy(this->data_, mat.data_, _rows * _cols * sizeof(float));
    return *this;
  }
  Matrixf<_rows, _cols>& operator+=(const Matrixf<_rows, _cols> mat) {
    arm_status s;
    s = arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
    return *this;
  }
  Matrixf<_rows, _cols>& operator-=(const Matrixf<_rows, _cols> mat) {
    arm_status s;
    s = arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &this->arm_mat_);
    return *this;
  }
  Matrixf<_rows, _cols>& operator*=(const float& val) {
    arm_status s;
    s = arm_mat_scale_f32(&this->arm_mat_, val, &this->arm_mat_);
    return *this;
  }
  Matrixf<_rows, _cols>& operator/=(const float& val) {
    arm_status s;
    s = arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &this->arm_mat_);
    return *this;
  }
  Matrixf<_rows, _cols> operator+(const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_add_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
    return res;
  }
  Matrixf<_rows, _cols> operator-(const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_sub_f32(&this->arm_mat_, &mat.arm_mat_, &res.arm_mat_);
    return res;
  }
  Matrixf<_rows, _cols> operator*(const float& val) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&this->arm_mat_, val, &res.arm_mat_);
    return res;
  }
  friend Matrixf<_rows, _cols> operator*(const float& val,
                                         const Matrixf<_rows, _cols>& mat) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&mat.arm_mat_, val, &res.arm_mat_);
    return res;
  }
  Matrixf<_rows, _cols> operator/(const float& val) {
    arm_status s;
    Matrixf<_rows, _cols> res;
    s = arm_mat_scale_f32(&this->arm_mat_, 1.f / val, &res.arm_mat_);
    return res;
  }
  // Matrix multiplication
  template <int cols>
  friend Matrixf<_rows, cols> operator*(const Matrixf<_rows, _cols>& mat1,
                                        const Matrixf<_cols, cols>& mat2) {
    arm_status s;
    Matrixf<_rows, cols> res;
    s = arm_mat_mult_f32(&mat1.arm_mat_, &mat2.arm_mat_, &res.arm_mat_);
    return res;
  }

  // Submatrix
  template <int rows, int cols>
  Matrixf<rows, cols> block(const int& start_row, const int& start_col) {
    Matrixf<rows, cols> res;
    for (int row = start_row; row < start_row + rows; row++) {
      memcpy((float*)res[0] + (row - start_row) * cols,
             (float*)this->data_ + row * _cols + start_col,
             cols * sizeof(float));
    }
    return res;
  }
  // Specific row
  Matrixf<1, _cols> row(const int& row) { return block<1, _cols>(row, 0); }
  // Specific column
  Matrixf<_rows, 1> col(const int& col) { return block<_rows, 1>(0, col); }

  // Transpose
  Matrixf<_cols, _rows> trans(void) {
    Matrixf<_cols, _rows> res;
    arm_mat_trans_f32(&arm_mat_, &res.arm_mat_);
    return res;
  }
  // Trace
  float trace(void) {
    float res = 0;
    for (int i = 0; i < fmin(_rows, _cols); i++) {
      res += (*this)[i][i];
    }
    return res;
  }
  // Norm
  float norm(void) { return sqrtf((this->trans() * *this)[0][0]); }

 public:
  // arm matrix instance
  arm_matrix_instance_f32 arm_mat_;

 protected:
  // size
  int rows_, cols_;
  // data
  float data_[_rows * _cols];
};

// Matrix funtions
namespace matrixf {

// Special Matrices
// Zero matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> zeros(void) {
  float data[_rows * _cols] = {0};
  return Matrixf<_rows, _cols>(data);
}
// Ones matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> ones(void) {
  float data[_rows * _cols] = {0};
  for (int i = 0; i < _rows * _cols; i++) {
    data[i] = 1;
  }
  return Matrixf<_rows, _cols>(data);
}
// Identity matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> eye(void) {
  float data[_rows * _cols] = {0};
  for (int i = 0; i < fmin(_rows, _cols); i++) {
    data[i * _cols + i] = 1;
  }
  return Matrixf<_rows, _cols>(data);
}
// Diagonal matrix
template <int _rows, int _cols>
Matrixf<_rows, _cols> diag(Matrixf<_rows, 1> vec) {
  Matrixf<_rows, _cols> res = matrixf::zeros<_rows, _cols>();
  for (int i = 0; i < fmin(_rows, _cols); i++) {
    res[i][i] = vec[i][0];
  }
  return res;
}

// Inverse
template <int _dim>
Matrixf<_dim, _dim> inv(Matrixf<_dim, _dim> mat) {
  arm_status s;
  // extended matrix [A|I]
  Matrixf<_dim, 2 * _dim> ext_mat = matrixf::zeros<_dim, 2 * _dim>();
  for (int i = 0; i < _dim; i++) {
    memcpy(ext_mat[i], mat[i], _dim * sizeof(float));
    ext_mat[i][_dim + i] = 1;
  }
  // elimination
  for (int i = 0; i < _dim; i++) {
    // find maximum absolute value in the first column in lower right block
    float abs_max = fabs(ext_mat[i][i]);
    int abs_max_row = i;
    for (int row = i; row < _dim; row++) {
      if (abs_max < fabs(ext_mat[row][i])) {
        abs_max = fabs(ext_mat[row][i]);
        abs_max_row = row;
      }
    }
    if (abs_max < 1e-12f) {  // singular
      return matrixf::zeros<_dim, _dim>();
      s = ARM_MATH_SINGULAR;
    }
    if (abs_max_row != i) {  // row exchange
      float tmp;
      Matrixf<1, 2 * _dim> row_i = ext_mat.row(i);
      Matrixf<1, 2 * _dim> row_abs_max = ext_mat.row(abs_max_row);
      memcpy(ext_mat[i], row_abs_max[0], 2 * _dim * sizeof(float));
      memcpy(ext_mat[abs_max_row], row_i[0], 2 * _dim * sizeof(float));
    }
    float k = 1.f / ext_mat[i][i];
    for (int col = i; col < 2 * _dim; col++) {
      ext_mat[i][col] *= k;
    }
    for (int row = 0; row < _dim; row++) {
      if (row == i) {
        continue;
      }
      k = ext_mat[row][i];
      for (int j = i; j < 2 * _dim; j++) {
        ext_mat[row][j] -= k * ext_mat[i][j];
      }
    }
  }
  // inv = ext_mat(:,n+1:2n)
  s = ARM_MATH_SUCCESS;
  Matrixf<_dim, _dim> res;
  for (int i = 0; i < _dim; i++) {
    memcpy(res[i], &ext_mat[i][_dim], _dim * sizeof(float));
  }
  return res;
}

// Cholesky decomposition: A = L * L^T (A must be symmetric positive definite)
// Returns lower triangular matrix L
// Returns zero matrix if decomposition fails
template <int _dim>
Matrixf<_dim, _dim> cholesky(Matrixf<_dim, _dim> mat) {
  Matrixf<_dim, _dim> res = matrixf::zeros<_dim, _dim>();
  arm_status s = arm_mat_cholesky_f32(&mat.arm_mat_, &res.arm_mat_);
  if (s == ARM_MATH_DECOMPOSITION_FAILURE) {
    return matrixf::zeros<_dim, _dim>();
  }
  return res;
}

// LU decomposition with partial pivoting: P * A = L * U
// L: lower triangular with unit diagonal
// U: upper triangular
// P: permutation matrix
// Returns false if matrix is singular
template <int _dim>
bool lu(Matrixf<_dim, _dim> mat,
        Matrixf<_dim, _dim>& L,
        Matrixf<_dim, _dim>& U,
        Matrixf<_dim, _dim>& P) {
  // Initialize P as identity, L as zeros, U as copy of mat
  P = matrixf::eye<_dim, _dim>();
  L = matrixf::zeros<_dim, _dim>();
  U = mat;

  for (int k = 0; k < _dim; k++) {
    // Find pivot: maximum absolute value in column k, rows k.._dim-1
    float abs_max = fabs(U[k][k]);
    int abs_max_row = k;
    for (int row = k + 1; row < _dim; row++) {
      if (fabs(U[row][k]) > abs_max) {
        abs_max = fabs(U[row][k]);
        abs_max_row = row;
      }
    }
    if (abs_max < 1e-12f) {  // singular
      return false;
    }

    // Swap rows in U, P, and L (columns 0..k-1 of L)
    if (abs_max_row != k) {
      for (int col = 0; col < _dim; col++) {
        float tmp;
        // swap U rows
        tmp = U[k][col];
        U[k][col] = U[abs_max_row][col];
        U[abs_max_row][col] = tmp;
        // swap P rows
        tmp = P[k][col];
        P[k][col] = P[abs_max_row][col];
        P[abs_max_row][col] = tmp;
      }
      // swap L rows (only columns 0..k-1)
      for (int col = 0; col < k; col++) {
        float tmp = L[k][col];
        L[k][col] = L[abs_max_row][col];
        L[abs_max_row][col] = tmp;
      }
    }

    // Elimination
    for (int row = k + 1; row < _dim; row++) {
      float factor = U[row][k] / U[k][k];
      L[row][k] = factor;
      for (int col = k; col < _dim; col++) {
        U[row][col] -= factor * U[k][col];
      }
    }
  }

  // Set L diagonal to 1
  for (int i = 0; i < _dim; i++) {
    L[i][i] = 1.0f;
  }

  return true;
}

// Determinant via LU decomposition
template <int _dim>
float det(Matrixf<_dim, _dim> mat) {
  Matrixf<_dim, _dim> L, U, P;
  if (!lu(mat, L, U, P)) {
    return 0.0f;
  }
  // det(A) = det(P^T) * det(L) * det(U)
  // det(L) = 1 (unit diagonal)
  // det(U) = product of diagonal
  // det(P^T) = (-1)^(number of row swaps) = det(P)
  float d = 1.0f;
  for (int i = 0; i < _dim; i++) {
    d *= U[i][i];
  }
  // Count row swaps from P: number of i where P[i][i] != 1
  int swaps = 0;
  // Track which row each position maps to
  int perm[_dim];
  for (int i = 0; i < _dim; i++) {
    for (int j = 0; j < _dim; j++) {
      if (P[i][j] == 1.0f) {
        perm[i] = j;
        break;
      }
    }
  }
  // Count inversions (transpositions) in the permutation
  for (int i = 0; i < _dim; i++) {
    while (perm[i] != i) {
      int target = perm[i];
      perm[i] = perm[target];
      perm[target] = target;
      swaps++;
    }
  }
  if (swaps % 2 != 0) {
    d = -d;
  }
  return d;
}

// Solve A*x = b using Cholesky decomposition (A must be SPD)
// Wraps arm_mat_solve_lower/upper_triangular_f32
template <int _dim>
Matrixf<_dim, 1> cholSolve(Matrixf<_dim, _dim> A, Matrixf<_dim, 1> b) {
  // A = L * L^T
  Matrixf<_dim, _dim> L = cholesky(A);

  // Solve L * y = b (forward substitution)
  Matrixf<_dim, 1> y;
  arm_mat_solve_lower_triangular_f32(&L.arm_mat_, &b.arm_mat_, &y.arm_mat_);

  // Solve L^T * x = y (back substitution)
  Matrixf<_dim, _dim> LT = L.trans();
  Matrixf<_dim, 1> x;
  arm_mat_solve_upper_triangular_f32(&LT.arm_mat_, &y.arm_mat_, &x.arm_mat_);

  return x;
}

// Solve A*x = b using LU decomposition
template <int _dim>
Matrixf<_dim, 1> luSolve(Matrixf<_dim, _dim> A, Matrixf<_dim, 1> b) {
  Matrixf<_dim, _dim> L, U, P;
  if (!lu(A, L, U, P)) {
    return matrixf::zeros<_dim, 1>();
  }

  // Pb = P * b
  Matrixf<_dim, 1> Pb = P * b;

  // Solve L * y = Pb (forward substitution)
  Matrixf<_dim, 1> y;
  for (int i = 0; i < _dim; i++) {
    float sum = 0.0f;
    for (int j = 0; j < i; j++) {
      sum += L[i][j] * y[j][0];
    }
    y[i][0] = Pb[i][0] - sum;  // L diagonal is 1
  }

  // Solve U * x = y (back substitution)
  Matrixf<_dim, 1> x;
  for (int i = _dim - 1; i >= 0; i--) {
    float sum = 0.0f;
    for (int j = i + 1; j < _dim; j++) {
      sum += U[i][j] * x[j][0];
    }
    x[i][0] = (y[i][0] - sum) / U[i][i];
  }

  return x;
}

}  // namespace matrixf

namespace vector3f {

// hat of vector
Matrixf<3, 3> hat(Matrixf<3, 1> vec);

// cross product
Matrixf<3, 1> cross(Matrixf<3, 1> vec1, Matrixf<3, 1> vec2);

}  // namespace vector3f

#endif  // MATRIX_H
