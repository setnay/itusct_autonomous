#ifndef MENU_BOT_MENU_HPP
#define MENU_BOT_MENU_HPP

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <map>

namespace Menu {
    enum Taste {
        SWEET,
        SOUR,
        BITTER,
        SALTY,
        SAVORY,
        NUM_TASTES
    };

    class TasteBalance {
    private:
        std::map<Taste, int> tasteMap;

    public:
        TasteBalance();
        TasteBalance(int sweet, int sour, int salty, int bitter, int savory);
        void print() const;
        const std::map<Taste, int> &getTasteMap() const;
    };

    struct Dish {
        std::string name;
        double price;
        TasteBalance taste_balance;
    };

     // Menüdeki tüm yemekleri içeren bir liste
    extern std::map<std::string, std::vector<Dish>> menu;

    // Menü dosyasını yükleme fonksiyonu
    void loadMenu(const std::string& menuJson);

    // Menüdeki yemekleri yazdırma fonksiyonu
    void printMenu();

    class MenuItem {
        private:
            std::string name;
            double price;
            TasteBalance taste_balance;
        public:
            // Default constructor. Initializes the name to an empty string, the price to 0, and the taste balance and TasteBalance object with default values.
            MenuItem() : name(""), price(0), taste_balance(TasteBalance()) {};
            // Parametreli constructor
            MenuItem (std::string foodName, double foodPrice, TasteBalance _taste_balance )
                : name(foodName), price(foodPrice), taste_balance (_taste_balance) {};

            // Getter functions
            std::string getName() const {return this->name;}
            double getPrice() const {return this->price;}
            TasteBalance getTasteBalance() const {return this -> taste_balance;}

            //Setter functios
            void setName(std::string foodName) {this->name= foodName; }
            void setPrice(double foodPrice) {this->price = foodPrice; }
            void setTasteBalance(TasteBalance _taste_balance) {this->taste_balance = _taste_balance; }

            virtual void display() const = 0 ;
            
            // Sanal yıkıcı
            virtual ~MenuItem() = default;
    };

    class Starter : public MenuItem {
        private:
            std::string hotOrCold; // Hot or cold preference
        public:
            Starter(std::string foodName, double foodPrice, TasteBalance _taste_balance)
            : MenuItem(foodName, foodPrice, _taste_balance), hotOrCold("Unknown") {}

             // Getter and Setter for hotOrCold
            std::string getHotOrCold() const { return this->hotOrCold; }
            void setHotOrCold(std::string preference) { this->hotOrCold = preference; }   

            void display() const override;
    };

    class Salad : public MenuItem {
        private:
            std::string topping; // Topping preference (e.g., Cheese, Olives, etc.)
            bool hasTopping; // Whether the salad has topping or not
        public:
            Salad(std::string foodName, double foodPrice, TasteBalance _taste_balance)
            : MenuItem(foodName, foodPrice, _taste_balance), topping("None"), hasTopping(false) {}

            std::string getTopping() const {return this->topping;}
            bool getHasTopping() const {return this->hasTopping;}

            void setTopping(std::string _topping) {this->topping = _topping;}
            void setHasTopping(bool _hasTopping) {this ->hasTopping = _hasTopping;}  

            void display() const override;     
    };

    class MainCourse : public MenuItem {
        private:
            bool isVegetarian; // Whether the user wants vegetarian food or not
            std::vector<std::string> vegetarianOptions {"Vegetable Stir-Fry", "Pesto Pasta" };
            std::vector<std::string> nonVegetarianOptions {"Grilled Salmon", "Chicken Alfredo", "Beef Stroganoff" };
        public:
            MainCourse(std::string foodName, double foodPrice, TasteBalance _taste_balance)
            : MenuItem(foodName, foodPrice, _taste_balance), isVegetarian(false) {}

            bool getIsVegetarian() const {return this->isVegetarian;}
            void setIsVegetarian(bool _isVegetarian) {this->isVegetarian = _isVegetarian;}
            void askVegetarianPreference();
            void getVegetarianOptions();
            void getNonVegetarianOptions();

            void display() const override;
    };

    class Drink : public MenuItem{
        private:
            bool isCarbonated;
            bool hasAlcoholShot;
            double carbonationCost = 0.5;
            double alcoholShotCost = 2.5;
        public:
            Drink(std::string foodName, double foodPrice, TasteBalance _taste_balance)
            : MenuItem(foodName, foodPrice, _taste_balance), isCarbonated(false), hasAlcoholShot(false) {}

            bool getIsCarbonated() const {return this->isCarbonated;}
            void setIsCarbonated(bool _isCarbonated) {this ->isCarbonated = _isCarbonated;}

            bool getHasAlcoholshot() const {return this->hasAlcoholShot;}
            void setHasAlcoholShot(bool _hasAlcoholShot) {this ->hasAlcoholShot = _hasAlcoholShot;}

            void display() const override;
    };

    class Appetizer : public MenuItem{
        private:
            std::string serveTime;
        public:
            Appetizer(std::string _name, double _price, TasteBalance _taste_balance)
            : MenuItem(_name, _price, _taste_balance), serveTime("Unknown") {}

            std::string getServeTime() const {return this->serveTime;}
            void setServeTime(std::string _serveTime) {this ->serveTime = _serveTime;}

            void display() const override;
    };

    class Dessert : public MenuItem{
        private:
            bool extraChocolate;
        public:
            Dessert(std::string foodName, double foodPrice, TasteBalance _taste_balance)
            : MenuItem(foodName, foodPrice, _taste_balance), extraChocolate(false) {}

            bool getExtraChocolate() const {return this->extraChocolate;}
            void setExtraChocolate(bool _extraChocolate) {this->extraChocolate = _extraChocolate;}  

            void display() const override;  
    };

    class Menu{
        private:
            std::vector<std::shared_ptr<MenuItem>> menuItems;
            double totalPrice;
            TasteBalance tasteBalance;
        public:
            //default constructer
            Menu();

            //parametreli constructer
            Menu(double _totalPrice, TasteBalance _tasteBalance);

            double getTotalPrice() const {return this->totalPrice;}
            TasteBalance getTasteBalance() const {return this->tasteBalance;}

            const std::vector<std::shared_ptr<MenuItem>>& getMenuItems() const {return this->menuItems;}

            void addMenuItem(std::shared_ptr<MenuItem> item);

            void removeMenuItem(const std::string& name);

            // Menü öğesini bulmak
            std::shared_ptr<MenuItem> findMenuItemByName(const std::string& name) const;

           // Menüdeki öğeleri ve toplam fiyatı yazdır
            void printMenu() const;

           // Menüdeki tat dengesini güncellemek
           void updateMenuBalance();

};

class User {
    private:
        std::string firstName;
        std::string lastName;
        std::string gender;
        std::shared_ptr<Menu> userMenu;
        std::vector<Dish> selectedDishes;
    public:
         User() : firstName(""), lastName(""), gender(""), userMenu(nullptr) {}

         User(std::string _firstName, std::string _lastName, std::string _gender, std::shared_ptr<Menu> _userMenu)
         : firstName(_firstName), lastName (_lastName), gender(_gender), userMenu(_userMenu) {} 

         std::string getFirstName() const {return firstName;}
         std::string getLastName() const {return lastName;}
         std::string getGender() const {return gender;}
         std::shared_ptr<Menu> getUserMenu() const {return userMenu;}
         
         void setFirstName(const std::string& _firstName) {firstName = _firstName;}
         void setLastName(const std::string& _lastName) {lastName = _lastName;}
         void setGender(const std::string& _gender) { gender = _gender; }
         void setUserMenu(std::shared_ptr<Menu> _userMenu) {this->userMenu = _userMenu;}

         void displayUserMenu() const;


         //hitap fonksiyonu
         std::string getGreeting() const ;
         void inputUserInfo();

         void suggestMenuBasedOnTasteBalance(User& user);  // ✅ Fixed declaration
                       
         std::vector<Dish> recommendDishes(const TasteBalance& userTaste);

          void addDish(const Dish& dish) {
             selectedDishes.push_back(dish);
         }

         // Eğer dışarıdan erişim gerekiyorsa getter:
         const std::vector<Dish>& getSelectedDishes() const { return selectedDishes; }
};

void askMenuChoice(User& user);


} // namespace Menu
                

#endif //MENU_BOT_MENU_HPP