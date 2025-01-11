# Basic Calculator (C++)

This is a basic calculator program built using C++. The calculator can perform several arithmetic operations including addition, subtraction, multiplication, division, square, exponentiation, and modulus. The program also handles input errors gracefully.

## Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [File Structure](#file-structure)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Example Run](#example-run)
- [Example: Handling Division by Zero](example-handling-division-by-zero)
- [How it Works](#how-it-works)
- [Functions Overview](#functions-overview)
- [Contact](#contact)

## Project Overview

The Basic C++ Calculator is a console-based program developed in C++ that provides a variety of fundamental mathematical operations. These operations include addition, subtraction, multiplication, division, modulus, exponentiation, and squaring. The project demonstrates the use of C++ templates, exception handling, and input validation. The goal of the project is to provide a simple yet robust calculator capable of performing real-time mathematical computations with efficient error handling.

## Features

- **Addition**: Performs the sum of two input numbers.
- **Subtraction**: Computes the difference between two input numbers.
- **Multiplication**: Computes the product of two input numbers.
- **Division**: Divides the first number by the second, with built-in error handling to prevent division by zero.
- **Modulus**: Calculates the remainder when the first number is divided by the second, also with checks to prevent division by zero.
- **Square**: Returns the square of the input number.
- **Exponentiation**: Raises a given base to a specified exponent using the power function.
- **Error Handling**: Includes extensive error handling mechanisms for invalid input and mathematical errors (e.g., division by zero, invalid choices).

The program is designed using **templates**, allowing flexibility to work with different numeric data types such as float and double.

## File Structure

The project consists of three main files:

- **Calculator.hpp**: The header file that defines the `Calculator` class template, which includes member function declarations for each operation and a static input validation method.
  
- **Calculator.cpp**: The implementation file where the member functions of the `Calculator` class are defined. This file contains the logic for each arithmetic operation and exception handling.

- **main.cpp**: The entry point of the application, which provides the user interface. It displays a menu for the user to select operations and processes the input to call the appropriate methods of the `Calculator` class.

## Requirements

Before using this project, ensure that you have the following:

- A C++ compiler that supports C++11 or later (e.g., GCC, Clang, Visual Studio).
- Standard C++ library support.

## Installation

To get started with the Basic Calculator project:

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/basic-calculator.git
   cd basic-calculator
3. Compile the code:
   ```bash
   g++ main.cpp Calculator.cpp -o calculator
4. Run the program:
   ```bash
   ./calculator

## Usage 

1. Once you run the program, you will be prompted with a menu to select an operation.
2. Enter the corresponding number for the desired operation.
3. The program will then ask you to input the required numbers for the operation.
4. The result of the operation will be displayed, and you can continue performing more calculations or exit the program.


## Example Run

Here is an example of how the program works:

```cpp
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
```
In this example, the user selects the Addition operation, inputs two numbers (5 and 3), and the result (8) is displayed.

## Example: Handling Division by Zero 
```cpp
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
Enter your choice (1-8): 4
Enter first number: 10
Enter second number: 0
Error! Denominator cannot be zero.
```
In this example, when the user attempts to perform a division by zero, the program displays the error message: **Error! Denominator cannot be zero.** and prompts for another input.

## How it Works
### Templates
The program uses **C++ templates** to allow the calculator to work with different numeric types like float and double. By defining the Calculator class as a template, the operations are performed on the data type provided during object creation.
### Exception Handling
The program handles several types of errors:
- **Invalid Input**: If the user provides a non-numeric input, an error message is shown.
- **Division by Zero**: An exception is thrown when attempting to divide by zero, and an appropriate error message is displayed.
- **Invalid Menu Choice**:  If the user selects an invalid operation (not between 1 and 8), the program will notify the user of the invalid choice.
### User Interaction
The program runs in a loop until the user chooses to exit. After each operation, the result is displayed, and the user is asked if they want to perform another calculation or exit. The program gracefully handles errors, ensures valid inputs, and continues operation without crashing.

##  Functions Overview
The program includes the following functions within the _Calculator class_:
- **addition()**: Adds two numbers.
- **subtract()**: Subtracts the second number from the first.
- **multiplication()**: Multiplies two numbers.
- **division()**: Divides the first number by the second and handles division by zero.
- **square()**: Returns the square of a number.
- **exponentiation()**: Raises the base to the power of the exponent.
- **modulus()**: Returns the modulus (remainder) of the division of two numbers, with error handling for division by zero.
- **inputErrorHandler()**: Handles input errors, converts the input to the appropriate numeric type, and throws exceptions in case of invalid inputs.

## Contact
If you have any questions or suggestions, feel free to reach out at setenay.ttc@gmail.com
 







     
   
     
