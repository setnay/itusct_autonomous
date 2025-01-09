# Calculator Project

## Description

This is a basic calculator implemented in C++ that provides basic mathematical operations such as addition, subtraction, multiplication, division, square, exponentiation, and modulus. It is designed to handle errors like division by zero and invalid input through proper exception handling.

The project is composed of three files:
- **Calculator.hpp**: Header file containing the declaration of the Calculator class and its member functions.
- **Calculator.cpp**: Implementation file for the functions declared in `Calculator.hpp`.
- **main.cpp**: The entry point for the program, where the user interacts with the calculator via a text-based interface.

## Features

- **Addition**: Adds two numbers.
- **Subtraction**: Subtracts the second number from the first.
- **Multiplication**: Multiplies two numbers.
- **Division**: Divides the first number by the second (with error handling for division by zero).
- **Square**: Calculates the square of a number.
- **Exponentiation**: Raises a base number to the power of an exponent.
- **Modulus**: Computes the remainder when the first number is divided by the second (with error handling for division by zero).
  
The program includes robust error handling for invalid inputs and divides the operations into separate functions within the Calculator class. It supports floating-point calculations and ensures a clean user interface by allowing users to input values directly.

## Files

- **Calculator.hpp**: Header file containing the class definition and function prototypes.
- **Calculator.cpp**: Source file implementing the Calculator class methods.
- **main.cpp**: Main program file that drives user interaction with the calculator.

### Example:

The following operations are supported:

1. Addition (e.g., 5 + 3)
2. Subtraction (e.g., 5 - 3)
3. Multiplication (e.g., 5 * 3)
4. Division (e.g., 5 / 3)
5. Square of a number (e.g., square(5))
6. Exponentiation (e.g., 2 ^ 3)
7. Modulus (e.g., 5 % 3)

## Usage

To use the calculator, simply run the program, and you will be presented with a menu of operations. You can choose an operation by entering the corresponding number.

```bash
g++ main.cpp Calculator.cpp -o calculator
./calculator
#### Example Run:
Welcome to the Basic Calculator!

Select an operation:
1. Addition
2. Subtraction
3. Multiplication
4. Division
5. Square
6. Exponentiation
7. Modulus
8. Exit

Enter your choice (1-8): 1
Enter first number: 5
Enter second number: 3
Result: 8

