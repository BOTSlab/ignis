#include "Ignis.hpp"

int main() {
    Ignis ignis;
    for (int i = 0; i < 1000; ++i)
        ignis.step();
    return 0;
}
