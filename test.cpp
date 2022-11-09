#include <vector>
#include "aeb.hpp"

using namespace LineDrawing;

int main() {
    Aeb<51> aeb{3, 0, 1.08, std::tuple{Pointf{1, 5}, Pointf{-1, 2}}, KartPoint{0.0, 1.3}, 2};
    Grid<41> g{};

    std::vector v{
            Linef{std::tuple{5.0f, 10.0f}, std::tuple{10.0f, 10.0f}},
            Linef{std::tuple{10.0f, 10.0f}, std::tuple{10.0f, 20.0f}},
            Linef{std::tuple{10.0f, 20.0f}, std::tuple{5.0f, 20.0f}},
            Linef{std::tuple{5.0f, 20.0f}, std::tuple{5.0f, 10.0f}},
    };

    g.draw_polygon(v);

    g.print();
}