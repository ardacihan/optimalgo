#include <iostream>
#include <random>
#include <vector>


class Rectangle {
    public:
        const int width;
        const int height;
        const bool rotated;

        Rectangle(int w, int h, bool rot) : width(w), height(h), rotated(rot) {};

        int area() const { return width * height; };

};

class BoundingBox {
    public:
        const int L;
        const int id;
        BoundingBox(int l,int id) : L(l), id(id) {};

    bool canFitRectable(Rectangle r) {
        return (r.width > L || r.height > L);
    };
};

class RectanglePlacement {
    public:
        Rectangle rectangle;
        int x,y;
        bool rotated;
        int boundingBoxId;

        RectanglePlacement(Rectangle r, int x, int y, bool rotated, int boundingBoxid){}


};

class InstanceGenerator


int main() {
    // TIP Press <shortcut actionId="RenameElement"/> when your caret is at the <b>lang</b> variable name to see how CLion can help you rename it.
    auto lang = "C++";
    std::cout << "Hello and welcome to " << lang << "!\n";
    Rectangle r(1,2,false);

    std::cout << r.a << r.b << std::endl;

    for (int i = 1; i <= 5; i++) {
        // TIP Press <shortcut actionId="Debug"/> to start debugging your code. We have set one <icon src="AllIcons.Debugger.Db_set_breakpoint"/> breakpoint for you, but you can always add more by pressing <shortcut actionId="ToggleLineBreakpoint"/>.
        std::cout << "i = " << i << std::endl;
    }

    return 0;
    // TIP See CLion help at <a href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>. Also, you can try interactive lessons for CLion by selecting 'Help | Learn IDE Features' from the main menu.
}