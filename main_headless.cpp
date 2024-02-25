#include "Ignis.hpp"

int main() {
    Ignis ignis;
    ignis.step();
    ignis.simWorldState->print();
    return 0;
}
