#include "matrix.hpp"

namespace MyMatrix {

   
    Matrix::Matrix(float rows, float cols, float initial_value) : rows(rows), cols(cols) {
        data.resize(rows, std::vector<float>(cols, initial_value));
    }

   
    Matrix::Matrix(const std::vector<std::vector<float>>& mat) : rows(mat.size()), cols(mat[0].size()) {
        data = mat;
    }

    
    int Matrix::get(float row, float col) const {
        return data[row][col];
    }

    void Matrix::set(float row, float col, float value) {
        data[row][col] = value;
    }

  
    void Matrix::print() const {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                std::cout << data[i][j] << " ";
            }
            std::cout << std::endl;
        }
    }

   
    Matrix Matrix::add(const Matrix& other) const {
        if (rows != other.rows || cols != other.cols) {
            throw std::invalid_argument("Matrix dimensions must match for addition");
        }
        Matrix result(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.set(i, j, data[i][j] + other.get(i, j));
            }
        }
        return result;
    }

    
    Matrix Matrix::subtract(const Matrix& other) const {
        if (rows != other.rows || cols != other.cols) {
            throw std::invalid_argument("Matrix dimensions must match for subtraction");
        }
        Matrix result(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.set(i, j, data[i][j] - other.get(i, j));
            }
        }
        return result;
    }

    
    Matrix Matrix::multiply(const Matrix& other) const {
        if (cols != other.rows) {
            throw std::invalid_argument("Matrix dimensions must match for dot product");
        }
        Matrix result(rows, other.cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                int sum = 0;
                for (int k = 0; k < cols; k++) {
                    sum += data[i][k] * other.get(k, j);
                }
                result.set(i, j, sum);
            }
        }
        return result;
    }

    
    Matrix Matrix::dot(const Matrix& other) const {
        if (rows != other.rows || cols != other.cols) {
            throw std::invalid_argument("Matrix dimensions must match for element-wise multiplication");
        }
        Matrix result(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.set(i, j, data[i][j] * other.get(i, j));
            }
        }
        return result;
    }

    
    Matrix Matrix::transpose() const {
        Matrix result(cols, rows);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.set(j, i, data[i][j]);
            }
        }
        return result;
    }

    
    float Matrix::trace() const {
        int trace = 0;
        for (int i = 0; i < std::min(rows, cols); i++) {
            trace += data[i][i];
        }
        return trace;
    }

    
    float Matrix::determinant() const {
        if (rows == 3 && cols == 3) {
            return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1])
             - data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0])
             + data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
        } else if (rows == 2 & cols == 3) {
            return (data[0][0] * data[1][1]) - (data [0][1] * data[1][0]);               
        }else if (rows == 1 & cols == 1) {
            return data[0][0];
        }else {
            throw std::invalid_argument("Matrices larger than 3x3 cannot be calculated");
        }
        
    }

    Matrix Matrix ::inverse() const {
        if (rows == 2 && cols == 2) {
            float det = determinant();
            if (det == 0) {
                throw std::invalid_argument("Matrix is singular and cannot be inverted");         
            }
            float inv_det_2 = 1/ det;
            Matrix result(2,2);
            result.set(0, 0, inv_det_2 * data[1][1]);
            result.set(0, 1, inv_det_2 * -data[0][1]);
            result.set(1, 0, inv_det_2 * -data[1][0]);
            result.set(1, 1, inv_det_2 * data[0][0]);

            return result;
        }else if (rows == 3 && cols == 3) {
            float det = determinant();
            if (det == 0){
                throw std::invalid_argument("Matrix is singular and cannot be inverted");
            }
            float inv_det_3 = 1/ det;
            Matrix result(3, 3);
            result.set(0, 0, inv_det_3 * (data[1][1] * data[2][2] - data[1][2] * data[2][1]));
            result.set(0, 1, inv_det_3 * (data[0][2] * data[2][1] - data[0][1] * data[2][2]));
            result.set(0, 2, inv_det_3 * (data[0][1] * data[1][2] - data[0][2] * data[1][1]));
            result.set(1, 0, inv_det_3 * (data[1][2] * data[2][0] - data[1][0] * data[2][2]));
            result.set(1, 1, inv_det_3 * (data[0][0] * data[2][2] - data[0][2] * data[2][0]));
            result.set(1, 2, inv_det_3 * (data[0][2] * data[1][0] - data[0][0] * data[1][2]));
            result.set(2, 0, inv_det_3 * (data[1][0] * data[2][1] - data[1][1] * data[2][0]));
            result.set(2, 1, inv_det_3 * (data[0][1] * data[2][0] - data[0][0] * data[2][1]));
            result.set(2, 2, inv_det_3 * (data[0][0] * data[1][1] - data[0][1] * data[1][0]));

            return result;
        }else if (rows == 1 && cols == 1) {
            if (data[0][0] == 0) {
                throw std::invalid_argument("0 has no inverse");
            }
            Matrix result(1,1);
            float inv_result = 1/data[0][0];
            result.set(0, 0, inv_result);

            return result;
        }else {
            throw std::invalid_argument("Matrices larger than 3x3 cannot be calculated");
        }
    }

    double Matrix::magnitude() const {//
        if (rows != 1) {
            throw std::invalid_argument("Magnitude is only defined for 1xN vectors");
        }
        double sum = 0;
        for (int i = 0; i < cols; i++) {
            sum += data[0][i] * data[0][i];
        }
        return std::sqrt(sum);
    }

    
    Matrix Matrix::zeroes(float rows, float cols) {
        return Matrix(rows, cols, 0);
    }

    Matrix Matrix::ones(float rows, float cols) {
        return Matrix(rows, cols, 1);
    }

    Matrix Matrix::identity(float size) {
        Matrix mat(size, size);
        for (int i = 0; i < size; i++) {
            mat.set(i, i, 1);
        }
        return mat;
    }
}
