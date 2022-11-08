#include <vector>
#include "aeb.hpp"

int main() {
    Aeb<51> aeb{3, 0, 1.08, std::tuple{Point{1, 5}, Point{-1, 2}}, KartPoint{0.0, 1.3}, 2};

    std::vector points{KartPoint{1, 2}, KartPoint{3, 4}};

    aeb.add_points(points);

    auto [pos, yaw] = aeb.predict_pos(1000);
}