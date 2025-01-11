# C++ Smart Pointers and Custom Pointer Implementation

This project demonstrates the implementation of a custom smart pointer class named `MyPointer`, and provides a comparison of various C++ pointer types, including `std::auto_ptr`, `std::unique_ptr`, `std::shared_ptr`, and `std::weak_ptr`. These pointer types are crucial for managing dynamic memory in modern C++ and help ensure proper memory management and ownership semantics.

## Contents
- **MyPointer Class**: A custom smart pointer that implements basic memory management functionality.
- **C++ Pointer Types**:
  - `std::auto_ptr`
  - `std::unique_ptr`
  - `std::shared_ptr`
  - `std::weak_ptr`

## Objectives

The primary objectives of this project are as follows:
1. **Custom Pointer Class**: To design and implement a basic smart pointer class, `MyPointer`, that encapsulates dynamic memory management functionalities.
2. **Comparison of Pointer Types**: To demonstrate and compare the functionality of various C++ pointer types, such as `std::auto_ptr`, `std::unique_ptr`, `std::shared_ptr`, and `std::weak_ptr`.
3. **Pointer Management**: To showcase how pointers are managed in C++, with a focus on memory ownership, lifetime management, and pointer semantics.

## Overview of C++ Pointer Types

### 1. `MyPointer` (Custom Smart Pointer)
`MyPointer` is a custom template class designed to manage memory dynamically. It includes basic functionalities like dereferencing and pointer access via overloaded operators `operator*` and `operator->`. Additionally, it ensures proper memory deallocation through its destructor.

- **Operator Overloading**: It overloads the dereference (`*`) and member access (`->`) operators.
- **Destructor**: The destructor automatically releases the allocated memory, mimicking the behavior of smart pointers.
  
### 2. `std::auto_ptr`
`std::auto_ptr` was a smart pointer type introduced in C++98, providing automatic memory management with ownership transfer semantics. However, it has been deprecated in C++11 and should be replaced with `std::unique_ptr`. When copied, an `auto_ptr` transfers ownership of the managed object to the destination pointer, leaving the original pointer null.

### 3. `std::unique_ptr`
`std::unique_ptr` is the modern replacement for `auto_ptr`. It ensures exclusive ownership of a dynamically allocated object, and unlike `auto_ptr`, it cannot be copied, only moved. This pointer type supports move semantics, making it ideal for transferring ownership between objects or functions.

### 4. `std::shared_ptr`
`std::shared_ptr` is a reference-counted smart pointer that allows multiple pointers to share ownership of the same dynamically allocated object. The object is automatically deallocated when the last `shared_ptr` goes out of scope. The `use_count()` function allows checking how many shared owners exist for the object.

### 5. `std::weak_ptr`
`std::weak_ptr` is associated with a `std::shared_ptr`, but it does not contribute to the reference count. It provides a mechanism to observe the object managed by `shared_ptr` without taking ownership. The `lock()` function can be used to obtain a `shared_ptr` if the object is still alive.

## Class Definition: `Test`

The `Test` class is a simple class used to demonstrate the functionality of smart pointers. It includes a method `printMessage` that outputs a confirmation message to the console to verify that the smart pointer is working correctly.

## Example Usage

The following code demonstrates the use of `MyPointer` and various C++ smart pointers:

```cpp
// Using MyPointer (Custom Smart Pointer)
MyPointer<int> sp1(new int(10));
std::cout << "Value: " << *sp1 << std::endl;
MyPointer<Test> sp2(new Test());
sp2->printMessage();

// Using std::auto_ptr
std::auto_ptr<int> auto_p1(new int);
*auto_p1.get() = 10;
std::cout << "auto_p1 points to: " << *auto_p1 << std::endl;
std::auto_ptr<int> auto_p2 = std::move(auto_p1);
std::cout << "auto_p2 points to: " << *auto_p2 << std::endl;

// Using std::unique_ptr
std::unique_ptr<int> unique_p1(new int);
*unique_p1.get() = 20;
std::cout << "unique_p1 points to: " << *unique_p1 << std::endl;
std::unique_ptr<int> unique_p2 = std::move(unique_p1);

// Using std::shared_ptr
std::shared_ptr<int> shared_p1(new int);
*shared_p1.get() = 30;
std::cout << "shared_p1 points to: " << *shared_p1 << std::endl;
std::shared_ptr<int> shared_p2 = shared_p1;
std::cout << "shared_p1 use count: " << shared_p1.use_count() << std::endl;

// Using std::weak_ptr
std::weak_ptr<int> wptr1 = shared_p1;
std::shared_ptr<int> locked_ptr = wptr1.lock();
```
### Expected Output:
<p align="center">
  <img src="https://github.com/user-attachments/assets/2967f594-7d97-4eeb-98eb-552a0beda8ec" alt="Alt metin" />
</p>

## Compilation and Execution
To compile the project, follow these steps:
1. Save the code as pointer_demo.cpp.
2. Use a C++ compiler to compile the code with C++11 support:
```bash
g++ pointer_demo.cpp -o pointer_demo -std=c++11
```
3. Run the program:
```bash
./pointer_demo
```
## Conclusion
This project provides a comprehensive overview of smart pointers in C++. Through the implementation of a custom pointer class (**MyPointer**), as well as the demonstration of standard C++ pointer types (**std::auto_ptr**,**std::unique_ptr**, **std::shared_ptr**, **std::weak_ptr**), it highlights the importance of proper memory management in C++. This project also serves as an educational tool for understanding how different smart pointers work and their practical applications in modern C++ programming.
## Contact
If you have any questions or suggestions, feel free to reach out at setenay.ttc@gmail.com
