# C++ Data Types and Pointer Sizes

This program is designed to demonstrate various C++ data types, their sizes in bytes, their minimum and maximum values, and pointer sizes. It also illustrates how to work with pointers in C++.

## Features

1. **Data Type Sizes**: The program prints out the sizes of various data types such as `int`, `long`, `short`, `float`, `double`, etc., in bytes.
2. **Min/Max Values**: For each data type, the minimum and maximum values are printed using the `std::numeric_limits` class.
3. **Pointer Sizes**: The program prints out the sizes of pointers for each data type along with their memory addresses.

## Data Types Displayed

The following C++ data types are displayed:
- `int`
- `long`
- `long long`
- `short`
- `short int`
- `unsigned int`
- `float`
- `double`
- `long double`
- `char`

The program outputs the following information for each data type:
- Size (in bytes)
- Minimum value
- Maximum value
- Pointer address
- Pointer size (in bytes)

## Example Output
The output of the program is divided into three sections:

### 1. Data Type Sizes

<p align="center">
  <img src="https://github.com/user-attachments/assets/0062920b-26b3-4397-a142-f00cbec4c7b0" />
</p>

### 2. Min/Max Values

<p align="center">
  <img src="https://github.com/user-attachments/assets/32f43d09-6544-4ab1-9090-5abf4ed8481a" />
</p>

### 3. Pointer Sizes
<p align="center">
  <img src="https://github.com/user-attachments/assets/f3fd7cff-151c-4417-8b52-5a7eec18b9fd" />
</p>

## How to Run

### Prerequisites

- **C++ Compiler**: A C++ compiler like `g++` or `clang++` should be installed on your system.
- **Operating System**: This program can run on Linux, macOS, and Windows with a suitable C++ development environment.

### Compilation

To compile the program, run the following command in your terminal:

```bash
g++ -o data_types_sizes data_types_sizes.cpp
```
### Running the Program
Once compiled, you can run the program with:
```bash
./data_types_sizes
```
The program will output the size of each data type, their minimum and maximum values, and the sizes of the corresponding pointers.

## Code Breakdown
1. **Header Files**:

   - `<iostream>` for input and output operations.
   - `<limits>` for retrieving the minimum and maximum values for data types.
   - `<iomanip>` for formatting the output.

2. **Output Formatting**:

   - `std::setw()` is used to control the width of each column in the table.
   - `std::left` and std::right are used to align the text in the columns properly.

3. **Using std::numeric_limits**:

   - This class provides the minimum and maximum values for various built-in types.
   - std::numeric_limits<int>::min() gives the minimum value for an int, and similarly for other types.

4. **Pointer Addresses and Sizes**:

   - Pointers are used to store memory addresses of variables.
   - The `sizeof` operator is used to determine the size of each pointer.

   ## Contactc
   If you have any questions or suggestions, feel free to reach out at setenay.ttc@gmail.com
