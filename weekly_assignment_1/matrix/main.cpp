#include "matrix.hpp"

int main() {
    using namespace MyMatrix;

    
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

    
    std::cout << "Matrix m1:" << std::endl;
    m1.print();

    
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

    
    std::cout << "Matrix m2:" << std::endl;
    m2.print();

    
    Matrix sum = m1.add(m2);
    std::cout << "Sum of m1 and m2:" << std::endl;
    sum.print();

    
    Matrix product = m1.multiply(m2);
    std::cout << "Element-wise multiplication of m1 and m2:" << std::endl;
    product.print();

   
    Matrix dotProd = m1.dot(m2);
    std::cout << "Dot product of m1 and m2:" << std::endl;
    dotProd.print();

   
    std::cout << "Trace of m1: " << m1.trace() << std::endl;

    
    Matrix m1Transposed = m1.transpose();
    std::cout << "Transpose of m1:" << std::endl;
    m1Transposed.print();

    
    std::cout << "Determinant of m1: " << m1.determinant() << std::endl;

    
    try {
        Matrix inv = m1.inverse();
        std::cout << "Inverse of m1:" << std::endl;
        inv.print();
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }

   
    Matrix zeroMatrix = Matrix::zeroes(3, 3);
    std::cout << "Zero Matrix:" << std::endl;
    zeroMatrix.print();

    Matrix identityMatrix = Matrix::identity(3);
    std::cout << "Identity Matrix:" << std::endl;
    identityMatrix.print();

        
    Matrix vector(1, 3);
    vector.set(0, 0, 3);
    vector.set(0, 1, 4);
    vector.set(0, 2, 12);
    std::cout << "Magnitude of the vector:" << std::endl;
    std::cout << vector.magnitude() << std::endl;

    return 0;
}

