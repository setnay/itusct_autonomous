#include <iostream>
#include "menu.hpp"
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>
#include <vector>
#include <map>
#include <memory>
#include <cstdlib>
#include <ctime>

using json = nlohmann::json;
using namespace Menu;
namespace Menu {

// --------------- TasteBalance Metodları ----------------
TasteBalance::TasteBalance() {
    for (int i = 0; i < NUM_TASTES; i++) {
        tasteMap[static_cast<Taste>(i)] = 0;
    }
}

TasteBalance::TasteBalance(int sweet, int sour, int salty, int bitter, int savory) {
    tasteMap[SWEET] = sweet;
    tasteMap[SOUR] = sour;
    tasteMap[SALTY] = salty;
    tasteMap[BITTER] = bitter;
    tasteMap[SAVORY] = savory;
}

void TasteBalance::print() const {
    std::cout << "Taste Balance - Sweet: " << tasteMap.at(SWEET)
              << ", Sour: " << tasteMap.at(SOUR)
              << ", Salty: " << tasteMap.at(SALTY)
              << ", Bitter: " << tasteMap.at(BITTER)
              << ", Savory: " << tasteMap.at(SAVORY) << std::endl;
}

const std::map<Taste, int>& TasteBalance::getTasteMap() const {
    return tasteMap;
}



//-------------------- Global Menu Map --------------------//
std::map<std::string, std::vector<Dish>> menu;

void loadMenu(const std::string& menuJson) {
    std::ifstream file(menuJson);
    if (!file) {
        std::cerr << "Menü dosyası açılamadı!" << std::endl;
        return;
    }
    
    json data;
    file >> data;

    for (const auto& category : data.items()) {
        std::vector<Dish> dishes;
        for (const auto& item : category.value()) {
            TasteBalance tb(
                item["taste_balance"]["sweet"],
                item["taste_balance"]["sour"],
                item["taste_balance"]["salty"],
                item["taste_balance"]["bitter"],
                item["taste_balance"]["savory"]
            );
            dishes.push_back({item["name"], item["price"], tb});
        }
        menu[category.key()] = dishes;
    }
}


    void Menu::addMenuItem(std::shared_ptr<MenuItem> item) {
        this ->menuItems.push_back(item);
    }

    // Menü öğelerini yazdırma fonksiyonu
    void Menu::printMenu() const {
        std::cout << "====== Your Menu ======\n";
        for (const auto& item : menuItems) {
            item->display(); // Menü öğesinin gösterilmesi
            std::cout << "-----------------------\n";
        }
        std::cout << "Total price: " << this->totalPrice << "\n";
    }

void User::displayUserMenu() const {
    if (selectedDishes.empty()) {
        std::cout << "Your menu is currently empty.\n";
        return;
    }
    double totalPrice = 0.0;
    std::cout << "\nYour Menu:\n";
    for (const auto& dish : selectedDishes) {
        std::cout << "- " << dish.name << " ($" << dish.price << ")\n";
        totalPrice += dish.price;
    }
    std::cout << "Total Price: $"  << totalPrice << "\n";
}

std::vector<Dish> User ::recommendDishes(const TasteBalance& userTaste) {
        std::vector<std::pair<Dish, int>> dishScores;

        for (const auto& category : menu) {
            for (const auto& dish : category.second) {
                int score = 0;

                // Kullanıcının tat tercihi ile yemeklerin tat dengesini karşılaştır
                for (int i = 0; i < NUM_TASTES; i++) {
                    Taste tasteType = static_cast<Taste>(i);
                    int difference = std::abs(userTaste.getTasteMap().at(tasteType) - dish.taste_balance.getTasteMap().at(tasteType));
                    score += difference;
                }

                // Skor ne kadar düşükse, yemek o kadar uygun
                dishScores.push_back({dish, score});
            }
        }

        // Yemekleri en düşük farktan başlayarak sırala
        std::sort(dishScores.begin(), dishScores.end(), [](const auto& a, const auto& b) {
            return a.second < b.second;
    });

    // En iyi 3 öneriyi döndür
    std::vector<Dish> recommendations;
    for (int i = 0; i < std::min(3, (int)dishScores.size()); i++) {
        recommendations.push_back(dishScores[i].first);
    }

    return recommendations;
} 

void suggestMenuBasedOnTasteBalance(User& user) {
    char continueChoice = ' ';
    do {
        int sweet, sour, salty, bitter, savory;
        std::cout << "\nPlease enter your taste preferences on a scale of 1 to 10:\n";
        std::cout << "Sweet: ";
        std::cin >> sweet;
        std::cout << "Sour: ";
        std::cin >> sour;
        std::cout << "Salty: ";
        std::cin >> salty;
        std::cout << "Bitter: ";
        std::cin >> bitter;
        std::cout << "Savory: ";
        std::cin >> savory;
        std::cin.ignore(); // clear input buffer

        TasteBalance userTaste(sweet, sour, salty, bitter, savory);
        std::vector<Dish> recommendations = user.recommendDishes(userTaste);

        // Predefined categories matching the global menu keys
        std::vector<std::string> categories = {"starters", "salads", "main_courses", "drinks", "appetizers", "desserts"};
        std::cout << "\nHere are the recommended dishes based on your taste preferences:\n";
        for (const auto& cat : categories) {
            auto it = menu.find(cat);
            if (it != menu.end() && !it->second.empty()) {
                bool found = false;
                // Find the first dish in this category that appears in the recommendations
                for (const auto& dish : it->second) {
                    auto recIt = std::find_if(recommendations.begin(), recommendations.end(),
                                              [&dish](const Dish& recDish) {
                                                  return recDish.name == dish.name;
                                              });
                    if (recIt != recommendations.end()) {
                        std::cout << "\nCategory: " << cat << "\n";
                        std::cout << "Recommended dish: " << dish.name << " ($" << dish.price << ")\n";
                        char add;
                        std::cout << "Would you like to add this dish to your menu? (e for yes / h for no): ";
                        std::cin >> add;
                        std::cin.ignore(); // clear input buffer
                        if (add == 'e' || add == 'E') {
                            user.addDish(dish);
                            std::cout << dish.name << " has been added to your menu.\n";
                        }
                        found = true;
                        break; // Offer one recommendation per category
                    }
                }
                if (!found) {
                    std::cout << "\nNo recommendation found for category: " << cat << "\n";
                }
            }
            else {
                std::cout << "\nCategory: " << cat << " is not defined or is empty in the menu.\n";
            }
        }

        std::cout << "\nHere is your final menu with the selected dishes:\n";
        user.displayUserMenu();
        
        std::cout << "\nPress 'q' to quit, 'c' to confirm and proceed, or any other key to go back to main menu: ";
        std::cin >> continueChoice;
        std::cin.ignore(); // clear input buffer
        
        if (continueChoice == 'c' || continueChoice == 'C') {
            std::cout << "Menu confirmed. Proceeding with final menu.\n";
            break;
        }
    } while(continueChoice != 'q' && continueChoice != 'Q');
    
    if (continueChoice == 'q' || continueChoice == 'Q') {
        std::cout << "Exiting the program. Goodbye!\n";
        exit(0);
    }
}

void generateRandomMenu() {
    srand(static_cast<unsigned int>(time(0)));  // Rastgele sayı üretici için başlangıç

    std::cout << "Random Menu:\n";
    double totalPrice = 0.0;
    
    for (const auto& category : menu) {
        const std::string& categoryName = category.first;
        const std::vector<Dish>& dishes = category.second;

        // Her kategoriden rastgele bir yemek seç
        int randomIndex = rand() % dishes.size();
        const Dish& randomDish = dishes[randomIndex];
        
        // Seçilen yemeği yazdır
        std::cout << categoryName << ": " << randomDish.name << " ($" << randomDish.price << ")\n";
        randomDish.taste_balance.print();
        
        // Fiyatı ekle
        totalPrice += randomDish.price;
    }

    std::cout << "\nTotal Price of the Random Menu: $" << totalPrice << "\n";
}


 //Kullanıcıya menü öneri tarzı sorusu

// --------------- MenuItem Metodları ----------------


void asHotOrCold(Starter &starter) {
    std::string userResponseForHotOrCold;
    while(true) {
        std::cout << "\nWould you like your starter hot or cold? ";
        std::getline(std::cin, userResponseForHotOrCold);
         
        // Convert input to lowercase for consistency
        for (auto &c : userResponseForHotOrCold) c = tolower(c);

        if (userResponseForHotOrCold == "hot" || userResponseForHotOrCold == "h"){
            starter.setHotOrCold("Hot");
            break;
        }else if(userResponseForHotOrCold == "cold" || userResponseForHotOrCold == "c") {
            starter.setHotOrCold("Cold");
            break;
        }else {
            std::cout << "Invalid input. Please enter 'hot' or 'cold'.\n";
        }
 
    }
}

void askForTopping(Salad& salad) {
    std::string userResponseForTopping;
    std::cout << "\nWould you like to add a topping to your salad? The cost of adding a topping is $2.25 USD.  (yes(y)/no(n)): ";
    std::getline(std::cin, userResponseForTopping);
    
    for (auto& c : userResponseForTopping) c = tolower(c);
    if (userResponseForTopping == "yes" || userResponseForTopping == "y") {
        std::string toppingName;
        std::cout << "\nWhat topping would you like to add? ";
        std::getline(std::cin, toppingName);
        
        salad.setTopping(toppingName);   // Add topping to the salad
        salad.setHasTopping(true);    // Set topping flag to true
        salad.setPrice(salad.getPrice() + 2.25);  // Increase price by 2.25 USD
    } else {
        std::cout << "No topping added.\n";
    }
}


    // Menüdeki öğeleri rastgele oluşturma fonksiyonu
void customizerDrinkPreferences(Drink& drink) {
    char choice;
    
    std::cout << "\nDo you want your drink carbonated? The cost for carbonation is 0.5 USD. (y/n): ";
    std::cin >> choice;
    if (choice == 'y' || choice == 'Y') {
        drink.setIsCarbonated(true);
        drink.setPrice(drink.getPrice() + 0.5);
    }
    std::cin.ignore(); // Clear input buffer

    std::cout << "\nDo you want an additional alcohol shot? The extra shot costs 2.5 USD. (y/n): ";
    std::cin >> choice;
    if (choice == 'y' || choice == 'Y') {
        drink.setHasAlcoholShot(true);
        drink.setPrice(drink.getPrice() + 2.5);
    }


}

void askAppetizerServeTime(Appetizer& appetizer){
    std::string choice;
    while(true) {
        std::cout << "\nWhen would you like to have your appetizer? (Before/After the main course): ";
        std::getline(std::cin, choice);

        //convert to lowercase for consistency
        for (auto& c : choice) c = tolower(c);

        if(choice == "before" ) {
            appetizer.setServeTime("Before main course");
            break;
        } else if (choice == "after"){
            appetizer.setServeTime("After main course");
            break;
        } else {
            std::cout << "Invalid choice. Please enter 'Before' or 'After'.\n";
        }

    }

}

void askExtraChocolate(Dessert& dessert) {
    std::string choice; 
    while(true) {
        std::cout << "\nWould you like to add extra chocolate to your dessert? The cost of the extra chocolate is 1.5 USD. (Yes/No): ";
        std::getline(std::cin, choice);

        for (auto& c : choice) c = tolower(c);

        if (choice == "yes") {
            dessert.setExtraChocolate(true);
            std::cout << "Extra chocolate added to your dessert.\n";
            dessert.setPrice(dessert.getPrice() + 1.5);  
            break;
        } else if (choice == "no") {
            dessert.setExtraChocolate(false);
            std::cout << "No extra chocolate added.\n";
            break;
        } else {
            std::cout << "Invalid choice. Please enter 'Yes' or 'No'.\n";
        }
    }
}

   void removeItemFromMenu(std::vector<Dish>& selectedItems, double& totalPrice) {
    if (selectedItems.empty()) {
        std::cout << "Your menu is currently empty.\n";
        return;
    }

    std::cout << "\nYour Current Menu:\n";
    for (size_t i = 0; i < selectedItems.size(); ++i) {
        std::cout << i + 1 << ". " << selectedItems[i].name << " ($" << selectedItems[i].price << ")\n";
    }

    int removeChoice;
    std::cout << "Enter the number of the item you want to remove (0 to cancel): ";
    std::cin >> removeChoice;
    std::cin.ignore();

    if (removeChoice == 0) {
        std::cout << "No items were removed.\n";
        return;
    }

    if (removeChoice >= 1 && removeChoice <= (int)selectedItems.size()) {
        totalPrice -= selectedItems[removeChoice - 1].price;
        std::cout << selectedItems[removeChoice - 1].name << " has been removed from your menu.\n";
        selectedItems.erase(selectedItems.begin() + removeChoice - 1);
    } else {
        std::cout << "Invalid choice. Please try again.\n";
        removeItemFromMenu(selectedItems, totalPrice);
    }
}

void createCustomMenu(User& user, const std::map<std::string, std::vector<Dish>>& menu) {
    double totalPrice = 0.0;
    std::vector<Dish> selectedItems;
    std::vector<std::string> selectedOptions; // ✅ Declare selectedOptions


    std::vector<std::string> categories = {"starters", "salads", "main_courses", "drinks", "appetizers", "desserts"};
    bool ordering = true;
    while (ordering) { // Loop until the user completes their order
        std::cout << "\nPlease select a category:\n";
        for (size_t i = 0; i < categories.size(); ++i) {
            std::cout << i + 1 << ". " << categories[i] << "\n";
        }
        std::cout << "0. Finish order\n";

        int categoryChoice;
        std::cout << "Your choice: ";
        std::cin >> categoryChoice;
        std::cin.ignore();

        if (categoryChoice == 0) {
            std::cout << "Order completed!\n";
            break; // Exit the loop, order is finished
        }

        if (categoryChoice < 1 || categoryChoice > (int)categories.size()) {
            std::cout << "Invalid selection! Please try again.\n";
            continue;
        }

        std::string selectedCategory = categories[categoryChoice - 1];

        std::cout << "\nCategory: " << selectedCategory << "\n";

        if (menu.find(selectedCategory) == menu.end() || menu.at(selectedCategory).empty()) {
            std::cout << "No dishes available in this category.\n";
            continue;
        }

        

        // Display dishes
        const auto& dishes = menu.at(selectedCategory);
        for (size_t i = 0; i < dishes.size(); ++i) {
            std::cout << i + 1 << ". " << dishes[i].name << " ($" << dishes[i].price << ")\n";
            std::cout << "   🍽️  Taste Balance - "
                      << "Sweet: " << dishes[i].taste_balance.getTasteMap().at(SWEET) << ", "
                      << "Sour: " << dishes[i].taste_balance.getTasteMap().at(SOUR) << ", "
                      << "Salty: " << dishes[i].taste_balance.getTasteMap().at(SALTY) << ", "
                      << "Bitter: " << dishes[i].taste_balance.getTasteMap().at(BITTER) << ", "
                      << "Savory: " << dishes[i].taste_balance.getTasteMap().at(SAVORY) << "\n";
        }


        std::cout << "0. Choose another category\n";
        int choice;
        std::cout << "\nPlease select a dish (1-" << dishes.size() << "): ";
        std::cin >> choice;
        std::cin.ignore();

        if (choice == 0) {
            continue; // User wants to select another category
        }

        if (choice < 1 || choice > (int)dishes.size()) {
            std::cout << "Invalid selection! Please try again.\n";
            continue;
        }

        // Get the selected dish
        Dish selectedDish = dishes[choice - 1];
         std::string customOption = ""; // Stores custom options

        // **Apply custom modifications based on dish category**
        if (selectedCategory == "starters") {
            Starter starter(selectedDish.name, selectedDish.price, selectedDish.taste_balance);
            asHotOrCold(starter);
            selectedDish.price = starter.getPrice();
            customOption = "(" + starter.getHotOrCold() + ")";
        } else if (selectedCategory == "salads") {
            Salad salad(selectedDish.name, selectedDish.price, selectedDish.taste_balance);
            askForTopping(salad);
            selectedDish.price = salad.getPrice();
            if (salad.getHasTopping()) {
                customOption = "(Topping: " + salad.getTopping() + ")";
            }
        } else if (selectedCategory == "drinks") {
            Drink drink(selectedDish.name, selectedDish.price, selectedDish.taste_balance);
            customizerDrinkPreferences(drink);
            selectedDish.price = drink.getPrice();
            if (drink.getIsCarbonated()) {
                customOption += "(Carbonated)";
            }
            if (drink.getHasAlcoholshot()) {
                if (!customOption.empty()) customOption += " ";
                customOption += "(Alcohol Added)";
            }
        } else if (selectedCategory == "appetizers") {
            Appetizer appetizer(selectedDish.name, selectedDish.price, selectedDish.taste_balance);
            askAppetizerServeTime(appetizer);
            selectedDish.price = appetizer.getPrice();
            customOption = "(Serve Time: " + appetizer.getServeTime() + ")";
        } else if (selectedCategory == "desserts") {
            Dessert dessert(selectedDish.name, selectedDish.price, selectedDish.taste_balance);
            askExtraChocolate(dessert);
            selectedDish.price = dessert.getPrice();
            if (dessert.getExtraChocolate()) {
                customOption = "(Extra Chocolate)";
            }
        } 
        selectedItems.push_back(selectedDish);
        selectedOptions.push_back(customOption);
        totalPrice += selectedDish.price;
        std::cout << "\n" << selectedDish.name << " has been added to your menu!\n"

        // Ask user if they want to add more items
        char addMore;
        std::cout << "\nWould you like to add another item? (y/n): ";
        std::cin >> addMore;
        std::cin.ignore();

        if (addMore == 'n' || addMore == 'N') {
            ordering = false; // User doesn't want to add more items
        }
    }

    // **At the end, show the final order summary**
    std::cout << "\nYour Final Order:\n";
    for (size_t i = 0; i < selectedItems.size(); ++i) {
        std::cout << "- " << selectedItems[i].name;
        
        if (!selectedOptions[i].empty()) { // ✅ Only print custom options if they exist
            std::cout << " " << selectedOptions[i];
        }

        std::cout << " ($" << selectedItems[i].price << ")\n";
    }
    std::cout << "\nTotal price: $" << totalPrice << "\n";

    // **Ask the user if they want to remove an item**
    char removeItem;
    std::cout << "\nWould you like to remove an item? (y/n): ";
    std::cin >> removeItem;
    std::cin.ignore();

    if (removeItem == 'y' || removeItem == 'Y') {
        removeItemFromMenu(selectedItems, totalPrice);
    }

    // **Save the final order to the user's menu**
    for (const auto& dish : selectedItems) {
        user.addDish(dish);
    }

    std::cout << "\nYour order has been successfully placed!\n";
}


void askMenuChoice(User& user);
void askMenuChoice(User& user) {

    Starter starter("", 0.0, TasteBalance());
    Salad salad("", 0.0, TasteBalance());
    MainCourse mainCourse("", 0.0, TasteBalance());
    Drink drink("", 0.0, TasteBalance());
    Appetizer appetizer("", 0.0, TasteBalance());
    Dessert dessert("", 0.0, TasteBalance());

    std::string greeting;
    std::string gender = user.getGender();
    std::transform(gender.begin(),  gender.end(), gender.begin(), ::tolower); // Küçük harfe çevir
    if(gender ==  "female" || gender == "f") {
        greeting = "Ms. " + user.getFirstName();
    } else if(gender ==  "male" || gender == "m"){
        greeting = "Mr. " + user.getFirstName();
    } else {
        greeting = user.getFirstName();
    }

     int choice;
     std::cout << "\n" << greeting << " How would you like to create your menu?\n";
     std::cout << "1. Let the program randomly generate the menu for you.\n";
     std::cout << "2. Select a menu based on your preferred taste balance.\n";
     std::cout << "3. Create your own menu from scratch.\n";
     
     std::cin >> choice;

        if (choice == 1) {
            std::cout << "The program will randomly generate the menu for you.\n";
            // Rastgele menü oluşturma işlemi burada yapılacak
            generateRandomMenu();
        } else if (choice == 2) {
            std::cout << "You will select the menu based on your taste balance.\n";
            // Tat dengesine göre menü önerisi işlemi burada yapılacak
            suggestMenuBasedOnTasteBalance(user);
        } else if (choice == 3) {
            std::cout << "You can create your own menu from scratch.\n";
            // Kullanıcı menüsünü sıfırdan oluşturacak
            createCustomMenu(user, menu);
        } else {
            std::cout << "Invalid choice. Try again.\n";
        }
    }

    std::string User::getGreeting () const {
        std::string lowerGender = gender;
        for (auto &c : lowerGender) c = tolower(c);
        if (gender == "male" || gender == "m"){
            return "Hello Mr." + firstName + " " + lastName + "!" ;
        } else if(gender == "female" || gender == "f"){
            return "Hello Ms." + firstName + " " + lastName + "!" ;
        } else {
            return "Hello " + firstName + " "  + lastName +  "!" ;
        }
    }

    void User::inputUserInfo() {
        std::cout << "Welcome to the Restaurant Bot!\n";
        std::cout << "I am here to help you choosing the perfect menu for you." << std::endl;
        std::cout << "Please enter your first name: ";
        std::getline(std::cin, firstName);
        setFirstName(firstName);

        std::cout << "Please enter your last name: ";
        std::getline(std::cin, lastName);
        setLastName(lastName);

        std::cout << "Please enter your gender(Male(M)/Female(F))";
        std::getline(std::cin, gender);
        setGender(gender);

        std::cout << getGreeting() << std::endl;
    }


void Starter::display() const {
    std::cout << "Starter: " <<getName() << " ($" << getPrice() << ")" << std::endl;
    getTasteBalance().print();
}

void Salad::display() const {
    std::cout << "Salad: " << getName() << " ($" << getPrice() << ")";
    getTasteBalance().print();
}

void MainCourse::display() const {
    std::cout << "Main Course: " << getName() << " ($" << getPrice() << ")";
    getTasteBalance().print();
}

void Drink::display() const {
    std::cout << "Drink: " << getName() << " ($" << getPrice() << ")";
    getTasteBalance().print();

    std::cout << "- Carbonated: " << (getIsCarbonated() ? "Yes" : "No") << "\n";
    std::cout << "- Additional Alcohol Shot: " << (getHasAlcoholshot() ? "Yes" : "No") << "\n";
}

void Appetizer::display() const {
    std::cout << "Appetizer: " << getName() << " ($" << getPrice() << ")";
    std::cout << "Serve Time: " << getServeTime() << "\n";
    getTasteBalance().print();
}

void Dessert::display() const {
    std::cout << "Dessert: " << getName() << " ($" << getPrice() << ")";
    std::cout << "Extra Chocolate: " << (getExtraChocolate() ? "Yes" : "No") << "\n";
    getTasteBalance().print();
}
    
         
}
            
 