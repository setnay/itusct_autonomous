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
6. [License](#license)

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






