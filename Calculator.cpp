// Calculator.cpp
#include "Calculator.hpp"
#include <stdexcept> // std::invalid_argument kullanımı için
#include <cmath>
template <typename T>
T Calculator<T>::addition(T num1, T num2) {
    return num1 + num2;
}

template <typename T>
T Calculator<T>::subtract(T num1, T num2) {
    return num1 - num2;
}

template <typename T>
T Calculator<T>::multiplication(T num1, T num2) {
    return num1 * num2;
}

template <typename T>
T Calculator<T>::division(T num1, T num2) {
    if (num2 == 0) {
        throw std::invalid_argument("Error! Denominator cannot be zero.");
    }
    return num1 / num2;
}

template <typename T>
T Calculator<T>::square(T num) {
    return num * num;
}

template <typename T>
T Calculator<T>::exponentiation(T base,T exp){
    return std :: pow(base,exp);
}

template <typename T>
T Calculator<T>::modulus(T num1, T num2){
    if(num2 ==0){
        throw std::invalid_argument("Error! Denominator cannot be zero.");
    }
    return std::fmod(num1, num2);
}

template <typename T>
T Calculator <T>::inputErrorHandler(const std :: string &input){ //This function converts the string to a value of type double.
    try {
        size_t pos;
        T value = std::stod(input, &pos);
        if(pos < input.length()){ //Here, what is wanted to be checked with the pos variable is whether the entire string is converted to a valid number during the conversion.
        //If the pos value is less than input.length(), then part of the string was invalid during the conversion (for example, letters were appended to the number).
            throw std::invalid_argument("Error: Invalid input.");          
        }
        return value; //If the entire string has been converted to a valid number, then value (numeric value) is returned.
    }catch(...){ //This catch block is used to catch all exceptions and in case of any error a general error message will be "Error: Invalid input." throws the message.
        throw std:: invalid_argument("Error: Invalid input.");
    }

}

template class Calculator <float>;
template class Calculator <double>;

