#include<iostream>
#include <cmath> // For sqrt function to calculate distance.

enum Region{
    first,second,third,fourth,fifth,sixth,seventh,eight,on_axis
};

struct Point {
    float x,y,z;

    Point(float x_val = 0, float y_val = 0, float z_val = 0):x(x_val), y(y_val), z(z_val) {}
    // to initialize point with given coordinates.

    float zero_distance() const{
        return sqrt(x*x + y*y + z*z);
    }

    static float  distance(Point& p1, Point& p2) {
        return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z));
    }

    static Point compare(Point& p1 , Point&p2){
        if(p1.zero_distance()> p2.zero_distance()){
            return p1;
        }else{
            return p2;
        }
    }

    Region region() const{
        if (x > 0 && y > 0 && z > 0) return first;
        if (x < 0 && y > 0 && z > 0) return second;
        if (x < 0 && y < 0 && z > 0) return third;
        if (x > 0 && y < 0 && z > 0) return fourth;
        if (x > 0 && y > 0 && z < 0) return fifth;
        if (x < 0 && y > 0 && z < 0) return sixth;
        if (x < 0 && y < 0 && z < 0) return seventh;
        if (x > 0 && y < 0 && z < 0) return eight;
        return on_axis;  // If any of x, y, or z is 0
    }

    static bool is_in_same_ragion(const Point& p1, const Point& p2) {
        return p1.region() == p2.region();
    }

    static std::string region_to_string(Region r) {
        switch (r)
        {
        case first:
            return "First Octant";
            break;
        case second:
            return "Second Octant";
            break;
        case third: 
            return "Third Octant";
            break;
        case fourth:
            return "Fourth Octant";
            break;
        case fifth:
            return "Fifth Octant";
            break;   
        case sixth:
            return "Sixth Octant";
            break;     
        case seventh:
            return "Seventh Octant";
            break;    
        case eight:
            return "Eight Octant";
            break;   
             
        default:
            return "On Axis";
            break;
        }
    }

};


int main(){
    Point p1(3.0, -2.0, 5.0);
    Point p2(-1.0, 4.0, 3.0);

    std :: cout << "Distance of p1 to the origin: " << p1.zero_distance() <<std::endl;
    
    std :: cout << "Distance between p1 and p2: " << Point::distance(p1,p2) << std::endl;
    
    Point farther = Point::compare(p1,p2);
    std:: cout << "Point farther from the origin: (" << farther.x <<"," << farther.y << ", " << farther.z<< ")" << std::endl;
    
    Region p1_region = p1.region();
    std::cout << "Region of p1: " << Point::region_to_string(p1_region)<< std::endl;
    Region p2_region = p2.region();
    std:: cout << "Region of p2: " << Point::region_to_string(p2_region) << std::endl;


    if (Point::is_in_same_ragion(p1,p2)){
        std:: cout << "p1 and p2 are in the same ragion." << std::endl;
    }else{
        std:: cout << "p1 and p2 are not in different ragion."<< std::endl;

    }
    return 0;

}