#include "menu.hpp"
#include <iostream>
#include <memory>

using namespace Menu;

int main() {
    // Menü dosyasını yükle
    loadMenu("menu.json");

    // Kullanıcı bilgilerini al
    User user;
    user.inputUserInfo();
    askMenuChoice(user);

    return 0;

}
   


