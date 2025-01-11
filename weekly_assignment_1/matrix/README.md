# Matrix Operations Library

The Matrix Operations Library is a C++ implementation that provides fundamental matrix operations and utilities for handling matrices and vectors. The library encapsulates a `Matrix` class within the `MyMatrix` namespace, which allows for the creation, manipulation, and transformation of matrices. It supports basic arithmetic operations such as addition, subtraction, and multiplication, as well as more advanced functionalities like calculating the inverse, determinant, trace, and magnitude.

This project is ideal for mathematical, engineering, and scientific applications that require efficient and flexible handling of matrix operations.

## Table of Contents

1. [Project Overview](#project-overview)
2. [Features](#features)
3. [Classes and Methods](#classes-and-methods)
   - [Matrix Class](#matrix-class)
   - [Constructor and Methods Overview](#constructor-and-methods-overview)
   - [Static Utility Methods](#static-utility-methods)
4. [How to Use the Library](#how-to-use-the-library)
5. [Matrix Operations](#matrix-operations)
   - [Arithmetic Operations](#arithmetic-operations)
   - [Matrix Transformations](#matrix-transformations)
6. [Advanced Matrix Operations](#advanced-matrix-operations)
7. [Error Handling](#error-handling)
8. [Compile and Run Instructions](#compile-and-run-instructions)
9. [License](#license)

---

## Project Overview

The Matrix Operations Library provides a class that abstracts the mathematical operations typically performed on matrices. It is implemented in C++ and uses standard libraries such as `vector` for matrix data storage and `stdexcept` for exception handling. The library is designed for flexibility and efficiency, with support for operations like matrix addition, multiplication, transpose, and determinant calculation, as well as advanced features like matrix inversion and magnitude computation.

### Key Concepts

- **Matrix**: A matrix is represented as a 2D array (a vector of vectors). The `Matrix` class encapsulates the matrix data and provides a variety of operations.
- **Vector**: A 1D matrix (a single row or column) is supported, and magnitude operations are specifically designed for such vectors.
- **Matrix Operations**: The library supports a wide range of matrix operations that are frequently used in linear algebra and machine learning algorithms.

---

## Features

### Matrix Creation and Initialization

- **Flexible Dimensions**: Create matrices of any size by specifying the number of rows and columns.
- **Initialization**: Fill matrices with custom values, or initialize them with default values (such as zero or one).

### Arithmetic Operations

- **Addition and Subtraction**: Support for adding and subtracting matrices element-wise, requiring matching dimensions.
- **Element-wise Multiplication**: The `dot` method performs element-wise multiplication between matrices.
- **Matrix Multiplication**: Supports multiplication of matrices with matching dimensions using the standard row-by-column multiplication rule.

### Matrix Transformations

- **Transpose**: The transpose operation swaps the rows and columns of a matrix.
- **Trace**: The trace method returns the sum of the diagonal elements of the matrix.
  
### Advanced Matrix Operations

- **Determinant**: Calculates the determinant of a square matrix. It is currently supported for 2x2, 3x3, and 1x1 matrices.
- **Inverse**: Computes the inverse of 2x2 and 3x3 matrices using the determinant and cofactor method.
- **Magnitude**: Computes the magnitude of a 1xN vector (sum of squares of elements followed by square root).

### Utility Methods

- **Zero Matrices**: Generates matrices filled with zeros.
- **Identity Matrices**: Generates identity matrices of size `n x n`.
- **Ones Matrices**: Creates matrices where all elements are set to one.

---

## Classes and Methods

### Matrix Class

The primary class for handling matrices is the `Matrix` class, which is defined in the `MyMatrix` namespace.

#### Constructor

```cpp
Matrix(float rows, float cols, float initial_value = 0);
Matrix(const std::vector<std::vector<float>>& mat);
```
- `Matrix(float rows, float cols, float initial_value)`: Constructs a matrix with the specified number of rows and columns, and initializes all elements to **initial_value** (default is 0).
- `Matrix(const std::vector<std::vector<float>>& mat)`: Constructs a matrix from a 2D vector, allowing initialization from an external 2D array or a **vector<vector<float>>**.

#### Methods Overview

  - **Matrix Operations**:
      - `add(const Matrix& other)`: Adds two matrices of the same dimensions element-wise.
      - `subtract(const Matrix& other)`: Subtracts one matrix from another element-wise.
      - `multiply(const Matrix& other)`: Multiplies two matrices using the row-by-column multiplication rule.
      - `dot(const Matrix& other)`: Performs element-wise multiplication of two matrices (also known as the Hadamard product).
      - `transpose()`: Returns a new matrix that is the transpose of the current matrix (rows become columns and vice versa).
      - `trace()`: Computes the trace of the matrix (sum of diagonal elements).
      - `determinant()`: Calculates the determinant of a matrix (currently supported for 1x1, 2x2, and 3x3 matrices).
      - `inverse()`: Computes the inverse of a matrix (currently supported for 2x2 and 3x3 matrices).
      - `magnitude()`: Computes the magnitude of a 1xN vector, which is the square root of the sum of squares of its elements.

  - **Matrix Utility**:
      - `get(float row, float col)`: Returns the element at the specified position (row, col).
      - `set(float row, float col, float value)`: Sets the value at position (row, col) to value.
      - `print()`: Prints the matrix to the console in a readable format.

#### Static Utility Methods
  - `zeroes(float rows, float cols)`: Creates a matrix of the specified size filled with zeroes.
  - `ones(float rows, float cols)`: Creates a matrix of the specified size filled with ones.
  - `identity(float size)`: Creates an identity matrix of size size x size.

## How to Use the Library
Here’s a simple example demonstrating how to use the Matrix Operations Library:

```cpp
int main() {
    using namespace MyMatrix;

    // Create a 3x3 matrix m1
    Matrix m1(3, 3);
    m1.set(0, 0, 8);
    m1.set(0, 1, 2);
    m1.set(0, 2, 3);
    m1.set(1, 0, 0);
    m1.set(1, 1, 1);
    m1.set(1, 2, 4);
    m1.set(2, 0, 5);
    m1.set(2, 1, 6);
    m1.set(2, 2, 0);

    // Print m1
    std::cout << "Matrix m1:" << std::endl;
    m1.print();

    // Create another matrix m2
    Matrix m2(3, 3);
    m2.set(0, 0, 9);
    m2.set(0, 1, 8);
    m2.set(0, 2, 7);
    m2.set(1, 0, 6);
    m2.set(1, 1, 5);
    m2.set(1, 2, 4);
    m2.set(2, 0, 3);
    m2.set(2, 1, 2);
    m2.set(2, 2, 1);

    // Perform matrix addition and print the result
    Matrix sum = m1.add(m2);
    std::cout << "Sum of m1 and m2:" << std::endl;
    sum.print();

    // Perform matrix multiplication and print the result
    Matrix product = m1.multiply(m2);
    std::cout << "Matrix multiplication of m1 and m2:" << std::endl;
    product.print();

    // Print the trace and determinant of m1
    std::cout << "Trace of m1: " << m1.trace() << std::endl;
    std::cout << "Determinant of m1: " << m1.determinant() << std::endl;

    // Try to find the inverse of m1
    try {
        Matrix inv = m1.inverse();
        std::cout << "Inverse of m1:" << std::endl;
        inv.print();
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

    return 0;
}
```
### Expected Output: 




