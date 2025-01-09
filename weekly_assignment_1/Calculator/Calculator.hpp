// Calculator.hpp
#ifndef CALCULATOR_HPP
#define CALCULATOR_HPP
#include<iostream>
#include<stdexcept>
template <typename T>
class Calculator {
public:
    // Fonksiyon prototipleri
    T addition(T num1, T num2);
    T subtract(T num1, T num2);
    T multiplication(T num1, T num2);
    T division(T num1, T num2);
    T square(T num);
    T exponentiation (T base, T exp);
    T modulus(T num1, T num2);

    static T inputErrorHandler (const std::string &input); //To use the std:: stod function in the standard library of C++
};

#endif
