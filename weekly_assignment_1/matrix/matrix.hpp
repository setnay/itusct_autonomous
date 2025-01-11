#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <iostream>
#include <stdexcept>
#include <cmath>

namespace MyMatrix {

    class Matrix {
    private:
        float rows, cols;
        std::vector<std::vector<float>> data;

    public:
        
        Matrix(float rows, float cols, float initial_value = 0);
        Matrix(const std::vector<std::vector<float>>& mat);

       
        int get(float row, float col) const;
        void set(float row, float col, float value);

        
        Matrix add(const Matrix& other) const;
        Matrix subtract(const Matrix& other) const;
        Matrix multiply(const Matrix& other) const;
        Matrix dot(const Matrix& other) const;
        Matrix transpose() const;
        float trace() const;
        float determinant() const; // float
        Matrix inverse() const;
        double magnitude() const;

        
        static Matrix zeroes(float rows, float cols);
        static Matrix ones(float rows, float cols);
        static Matrix identity(float size);

        
        void print() const;
    };

}

#endif // MATRIX_H


