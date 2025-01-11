# 3D Point Class and Region Classification

This project defines a C++ program to represent points in 3-dimensional space, calculate their distances from the origin, compare their distances from the origin, and classify their regions based on their coordinates. It includes a `Point` class with methods for computing distances, determining which region a point lies in, and checking whether two points lie in the same region.

## Table of Contents

1. [Project Overview](#project-overview)
2. [Class and Structure Details](#class-and-structure-details)
   - [Point Class](#point-class)
   - [Region Enum](#region-enum)
3. [Functions](#functions)
   - [Point Constructor](#point-constructor)
   - [zero_distance Method](#zero_distance-method)
   - [distance Method](#distance-method)
   - [compare Method](#compare-method)
   - [region Method](#region-method)
   - [is_in_same_region Method](#is_in_same_region-method)
   - [region_to_string Method](#region_to_string-method)
4. [Usage Example](#usage-example)
5. [Compile and Run Instructions](#compile-and-run-instructions)
6. [Contact](#contact)

---

## Project Overview

The project focuses on creating a `Point` class to handle operations in 3D space. The main purpose of this program is to:

- Calculate the Euclidean distance of a point from the origin.
- Calculate the distance between two points.
- Determine the point farther from the origin.
- Classify the point into one of the eight octants of 3D space, or identify if the point lies on one of the axes.
- Check if two points are in the same region of 3D space.

This project can be used in a variety of fields, such as computational geometry, physics simulations, and computer graphics, where working with 3D points and their relationships is essential.

---

## Class and Structure Details

### Point Class

The `Point` class represents a point in 3D space with the following attributes:

- `x`: The x-coordinate of the point.
- `y`: The y-coordinate of the point.
- `z`: The z-coordinate of the point.

#### Point Constructor
The `Point` class constructor initializes the point's coordinates with default values of 0 for each coordinate, but it also allows specifying coordinates when creating an object.

```cpp
Point(float x_val = 0, float y_val = 0, float z_val = 0)
```
#### zero_distance Method 
The **zero_distance** method calculates the Euclidean distance of the point from the origin (0, 0, 0). It uses the formula:

$$
\text{distance} = \sqrt{x^2 + y^2 + z^2}
$$

```cpp
float zero_distance() const;
```
#### distance Method
The **distance** method computes the Euclidean distance between two points, p1 and p2, using the formula:

$$
\text{distance} = \sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2 + (z_1 - z_2)^2}
$$

```cpp
static float distance(Point& p1, Point& p2);
```
#### compare Method
The **compare** method compares the distances of two points from the origin and returns the point that is farther from the origin.
```cpp
static Point compare(Point& p1, Point& p2);
```
#### region Method
The **region** method classifies the point into one of the eight octants of 3D space based on the signs of its **x**, **y**, and **z** coordinates. It returns one of the following regions:
- First Octant
- Second Octant
- Third Octant
- Fourth Octant
- Fifth Octant
- Sixth Octant
- Seventh Octant
- Eighth Octant
- On Axis (if any coordinate is zero)

```cpp
Region region() const;
```
#### is_in_same_region Method 
The **is_in_same_region** method checks if two points are in the same octant (region) of 3D space.
```cpp
static bool is_in_same_region(const Point& p1, const Point& p2);
```
#### region_to_string Method
The **region_to_string** method converts a Region enum value to a human-readable string, representing the octant name.
```cpp
static std::string region_to_string(Region r);
```

## Usage Example
Here’s an example of how to use the **Point class** to create points, compute distances, compare points, and determine their regions.

```cpp
int main() {
    // Create two points in 3D space
    Point p1(3.0, -2.0, 5.0);
    Point p2(-1.0, 4.0, 3.0);

    // Calculate distance of p1 from the origin
    std::cout << "Distance of p1 to the origin: " << p1.zero_distance() << std::endl;

    // Calculate distance between p1 and p2
    std::cout << "Distance between p1 and p2: " << Point::distance(p1, p2) << std::endl;

    // Compare points to find the one farther from the origin
    Point farther = Point::compare(p1, p2);
    std::cout << "Point farther from the origin: (" << farther.x << "," << farther.y << ", " << farther.z << ")" << std::endl;

    // Display the region of p1 and p2
    Region p1_region = p1.region();
    std::cout << "Region of p1: " << Point::region_to_string(p1_region) << std::endl;
    
    Region p2_region = p2.region();
    std::cout << "Region of p2: " << Point::region_to_string(p2_region) << std::endl;

    // Check if p1 and p2 are in the same region
    if (Point::is_in_same_region(p1, p2)) {
        std::cout << "p1 and p2 are in the same region." << std::endl;
    } else {
        std::cout << "p1 and p2 are not in the same region." << std::endl;
    }

    return 0;
}
```
### Expected Output:
<p align="center">
  <img src="https://github.com/user-attachments/assets/ea3c1316-a277-47df-aa13-79d647e6d5ab" alt="Alt metin" />
</p>

## Compile and Run Instructions
To compile and run this program, ensure you have a C++ compiler (e.g., g++) installed on your system. Follow these steps:
1. Open a terminal or command prompt.
2. Navigate to the directory containing the **main.cpp** file.
3. Compile the program using the following command:
```bash
g++ main.cpp -o point_program -std=c++11
```
4. Run the program.
```bash
./point_program
```

## Contact 
If you have any questions or suggestions, feel free to reach out at setenay.ttc@gmail.com













