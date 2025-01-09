# Basic Calculator (C++)

This is a basic calculator program built using C++. The calculator can perform several arithmetic operations including addition, subtraction, multiplication, division, square, exponentiation, and modulus. The program also handles input errors gracefully.

## Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Example Run](#example-run)
- [How it Works](#how-it-works)
- [Functions Overview](#functions-overview)
- [Error Handling](#error-handling)
- [License](#license)

## Project Overview

This C++ project implements a basic calculator with support for multiple operations such as addition, subtraction, multiplication, division, square, exponentiation, and modulus. It is designed to be simple, easy to use, and able to handle various error cases like division by zero and invalid inputs.

## Features

- Addition, subtraction, multiplication, and division.
- Square and exponentiation operations.
- Modulus operation to find the remainder.
- Input error handling for invalid inputs (e.g., text input when a number is expected).
- Reusable `Calculator` class with a templated interface (supports different numeric types like `float` and `double`).
- Simple interactive text-based menu.

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

---

### Example Run:
#### Operations Menu

- **1. Addition**: Adds two numbers.
- **2. Subtraction**: Subtracts the second number from the first.
- **3. Multiplication**: Multiplies two numbers.
- **4. Division**: Divides the first number by the second.
- **5. Square**: Returns the square of a number.
- **6. Exponentiation**: Raises a number (base) to the power of another number (exponent).
- **7. Modulus**: Returns the remainder when dividing the first number by the second.
- **8. Exit**: Exits the program.
##### Example 1: Addition 
    ```bash
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


 ##### Example 2: Division (with error handling)
 







     
   
     
