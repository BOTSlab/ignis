#include "IgnisScenario.hpp"

int main() {
    IgnisScenario ignis;
    for (int i = 0; i < 1000; ++i)
        ignis.step();
    return 0;
}
