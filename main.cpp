// main.cpp
#include <iostream>
#include<string>
#include "Calculator.hpp"

int main() {
    Calculator<float> calc ; //Creating an object with float type from the Calculator class
    std ::string input; //A string variable to store the input to be received from the user
    float num1, num2;
    float base, exp;
    int choice;

    std :: cout<< "Welcome to the Basic Calculator!" << std::endl;

    do{
        std:: cout<< "\nSelect an operation: "<< std::endl;
        std:: cout<< "1. Addition" <<std::endl;
        std::cout << "2. Subtraction" << std::endl;
        std::cout << "3. Multiplication" << std::endl;
        std::cout << "4. Division" << std::endl;
        std::cout << "5. Square" << std::endl;
        std::cout << "6. Exponentiation" << std::endl;
        std::cout << "7. Modulus" << std::endl;
        std::cout << "8. Exit" << std::endl;

        std::cout << "Enter your choice (1-8): ";
        std::getline(std::cin,input);

        try{
            choice = calc.inputErrorHandler(input);
            if (choice==8) {
                std :: cout<< "Exiting the program." << std::endl;
                break;
            }
            if (choice < 1 || choice >8 ) {
                throw std:: invalid_argument("Error: Invalid choice");
            }
            if (choice ==5) {
                std :: cout<< "Enter number: ";
                std::getline(std::cin,input);
                num1 =calc.inputErrorHandler(input);
                std::cout << "Result: " << calc.square(num1) <<std::endl;
            }else if(choice ==6){
                std :: cout<< "Enter base: ";
                std:: getline(std::cin,input);
                base=calc.inputErrorHandler(input);
                std :: cout<< "Enter exponent: ";
                std:: getline(std::cin,input);
                exp= calc.inputErrorHandler(input);
                std:: cout<< "Result: " <<calc.exponentiation(base,exp) <<std::endl;
            }else if (choice==7){
                std::cout<< "Enter first number: ";
                std ::getline(std::cin,input);
                num1=calc.inputErrorHandler(input);
                std:: cout<< "Enter second number: ";
                std :: getline(std::cin,input);
                num2=calc.inputErrorHandler(input);
                std::cout<< "Result: " << calc.modulus(num1,num2) << std::endl;
            }else {
                std:: cout<<"Enter first number: ";
                std::getline(std::cin,input);
                num1 = calc.inputErrorHandler(input);
                std:: cout<<"Enter second number: ";
                std ::getline(std::cin,input);
                num2= calc.inputErrorHandler(input);

                switch (choice)
                {
                case 1:
                    std :: cout<< "Result: " <<calc.addition(num1,num2) <<std::endl;
                    break;
                case 2:
                    std :: cout<< "Result: " <<calc.subtract(num1,num2) << std::endl;
                    break;
                case 3:
                    std :: cout<< "Result: " <<calc.multiplication(num1,num2) <<std::endl;
                    break;
                case 4:
                    std :: cout<< "Result: " <<calc.division(num1,num2) <<std::endl;
                    break;
                default:
                    break;
                }

            }


        }catch(const std::exception &e){
            std::cout <<e.what() <<std::endl;
        }

    }while(true);
    return 0;
}




