#include <iostream>
#include <limits>
#include <iomanip>

int main() {
    std :: cout << std::setw(15) << std::left<< "--------------------";
    std :: cout << std::setw(15) << std::right<< "--------------------" << std::endl;
    std :: cout << std::setw(15) << std::left<< "Type";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(15) << std::right<< "Size (byte)" << std::endl;
    std :: cout << std::setw(15) << std::left<< "--------------------";
    std :: cout << std::setw(15) << std::right<< "--------------------" << std::endl;

    std :: cout << std::setw(15) << std:: left<< "int";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< sizeof(int) << std::endl;
    std :: cout << std::setw(15) << std:: left<< "long";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< sizeof(long) << std::endl;
    std :: cout << std::setw(15) << std:: left<< "long long";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< sizeof(long long) << std::endl;
    std :: cout << std::setw(15) << std:: left<< "long int";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< sizeof(long int) << std::endl;
    std :: cout << std::setw(15) << std:: left<< "long long int";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< sizeof(long long int) << std::endl;
    std :: cout << std::setw(15) << std:: left<< "short";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< sizeof(short) << std::endl;
    std :: cout << std::setw(15) << std:: left<< "short int";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< sizeof(short int) << std::endl;
    std :: cout << std::setw(15) << std:: left<< "unsigned int";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< sizeof(unsigned int) << std::endl;
    std :: cout << std::setw(15) << std:: left<< "float";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< sizeof(float) << std::endl;
    std :: cout << std::setw(15) << std:: left<< "double";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< sizeof(double) << std::endl;
    std :: cout << std::setw(15) << std:: left<< "long double";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< sizeof(long double) << std::endl;
    std :: cout << std::setw(15) << std:: left<< "char";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< int(sizeof(char)) << std::endl << std::endl;
    

    
    
    std :: cout << std::setw(15) << std::left<< "------------------------------";
    std :: cout << std::setw(15) << std::right<< "------------------------------" << std::endl;
    std :: cout << std::setw(15) << std::left<< "Type";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< "min";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(20) << std:: right << "max" << std::endl;
    std :: cout << std::setw(15) << std::left<< "------------------------------";
    std :: cout << std::setw(15) << std::right<< "------------------------------" << std::endl;
    std :: cout << std::setw(15) << std::left<< "int";
    std :: cout << std::setw(3) << std::left << " ";
    std :: cout << std::setw(15) << std::right << std::numeric_limits<int>::min();
    std :: cout << std::setw(25) << std::right << std::numeric_limits<int>::max() << std::endl;
    std :: cout << std::setw(15) << std::left<< "long int";
    std :: cout << std::setw(3) << std::left << " ";
    std :: cout << std::setw(15) << std::right << std::numeric_limits<long int>::min();
    std :: cout << std::setw(25) << std::right << std::numeric_limits<long int>::max() << std::endl;
    std :: cout << std::setw(15) << std::left<< "short int";
    std :: cout << std::setw(3) << std::left << " ";
    std :: cout << std::setw(10) << std::right << std::numeric_limits<short int>::min();
    std :: cout << std::setw(25) << std::right << std::numeric_limits<short int>::max() << std::endl;
    std :: cout << std::setw(15) << std::left<< "unsigned int";
    std :: cout << std::setw(3) << std::left << " ";
    std :: cout << std::setw(15) << std::right << std::numeric_limits<int>::min();
    std :: cout << std::setw(25) << std::right << std::numeric_limits<int>::max() << std::endl;
    std :: cout << std::setw(15) << std::left<< "float";
    std :: cout << std::setw(3) << std::left << " ";
    std :: cout << std::setw(15) << std::right << std::numeric_limits<float>::lowest();
    std :: cout << std::setw(25) << std::right << std::numeric_limits<float>::max() << std::endl;
    std :: cout << std::setw(15) << std::left<< "double";
    std :: cout << std::setw(3) << std::left << " ";
    std :: cout << std::setw(15) << std::right << std::numeric_limits<double>::lowest();
    std :: cout << std::setw(25) << std::right << std::numeric_limits<double>::max() << std::endl;
    std :: cout << std::setw(15) << std::left<< "long double";
    std :: cout << std::setw(3) << std::left << " ";
    std :: cout << std::setw(15) << std::right << std::numeric_limits<long double>::lowest();
    std :: cout << std::setw(25) << std::right << std::numeric_limits<long double>::max() << std::endl;
    std :: cout << std::setw(15) << std::left<< "char";
    std :: cout << std::setw(3) << std::left << " ";
    std :: cout << std::setw(15) << std::right << int(std::numeric_limits<char>::min());
    std :: cout << std::setw(25) << std::right << int(std::numeric_limits<char>::max()) << std::endl<< std::endl << std::endl;



    std :: cout << std::setw(15) << std::left<< "------------------------------";
    std :: cout << std::setw(15) << std::right<< "------------------------------" << std::endl;
    std :: cout << std::setw(15) << std::left<< "Type";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(10) << std::right<< "Pointer";
    std :: cout << std::setw(3) << std::left<< " ";
    std :: cout << std::setw(20) << std:: right << "Size" << std::endl;
    std :: cout << std::setw(15) << std::left<< "------------------------------";
    std :: cout << std::setw(15) << std::right<< "------------------------------" << std::endl;
    
    
    std::string string = "Hello World!";
    std::string *pStr = &string;
    std::cout << std::setw(15) << std::left << "string";
    std::cout << std::setw(3) << std::left << " ";
    std::cout << std::setw(15) << std::right << pStr;
    std::cout << std::setw(17) << std::right << sizeof(pStr) << std::endl;

    int integer = 5;
    int *pInt = &integer;
    std::cout << std::setw(15) << std::left << "int";
    std::cout << std::setw(3) << std::left << " ";
    std::cout << std::setw(15) << std::right << pInt;
    std::cout << std::setw(17) << std::right << sizeof(pInt) << std::endl;
    long int longInt = 2;
    long int *pLongInt = &longInt;
    std::cout << std::setw(15) << std::left << "long int";
    std::cout << std::setw(3) << std::left << " ";
    std::cout << std::setw(15) << std::right << pLongInt;
    std::cout << std::setw(17) << std::right << sizeof(pLongInt) << std::endl;
    short int shortInt = 3;
    short int *pShortInt = &shortInt;
    std::cout << std::setw(15) << std::left << "short int";
    std::cout << std::setw(3) << std::left << " ";
    std::cout << std::setw(15) << std::right << pShortInt;
    std::cout << std::setw(17) << std::right << sizeof(pShortInt) << std::endl;
    unsigned int unsignedInt = 4;
    unsigned int *pUnsignedInt = &unsignedInt;
    std::cout << std::setw(15) << std::left << "unsigned int";
    std::cout << std::setw(3) << std::left << " ";
    std::cout << std::setw(15) << std::right << pUnsignedInt;
    std::cout << std::setw(17) << std::right << sizeof(pUnsignedInt) << std::endl;  
    float floatVar = 5.5;
    float *pFloat = &floatVar;
    std::cout << std::setw(15) << std::left << "float";
    std::cout << std::setw(3) << std::left << " ";
    std::cout << std::setw(15) << std::right << pFloat;
    std::cout << std::setw(17) << std::right << sizeof(pFloat) << std::endl;
    double doubleVar = 6.6;
    double *pDouble = &doubleVar;
    std::cout << std::setw(15) << std::left << "double";
    std::cout << std::setw(3) << std::left << " ";
    std::cout << std::setw(15) << std::right << pDouble;
    std::cout << std::setw(17) << std::right << sizeof(pDouble) << std::endl;
    long double longDoubleVar = 7.7;
    long double *pLongDouble = &longDoubleVar;
    std::cout << std::setw(15) << std::left << "long double";
    std::cout << std::setw(3) << std::left << " ";
    std::cout << std::setw(15) << std::right << pLongDouble;
    std::cout << std::setw(17) << std::right << sizeof(pLongDouble) << std::endl;
    char charVar = 'a';
    char *pChar = &charVar;
    std::cout << std::setw(15) << std::left << "char";
    std::cout << std::setw(3) << std::left << " ";
    std::cout << std::setw(10) << std::right << pChar;
    std::cout << std::setw(23) << std::right << sizeof(pChar) << std::endl;
    return 0;



}