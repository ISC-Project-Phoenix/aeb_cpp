# aeb_cpp

This library is a C++ implementation of the automatic emergency braking algorithm for [Project Phoenix](https://github.com/ISC-Project-Phoenix).

The code in this repo utilises the STL, however it goes out of its way to avoid allocations. It should be usable in embedded contexts, so long as
an implementation of the STL is available.

For algorithm and system integration details, please see the [AEB design document](https://github.com/ISC-Project-Phoenix/design/blob/main/software/AEB.md).

# Example
```c++
#include <vector>
#include "aeb_cpp/aeb.hpp"

using namespace LineDrawing;

int main() {
    // Configure AEB struct with values from Phoenix
    auto aeb = Aeb<51>{3, 0, 1.08, std::tuple{Pointf{-0.675, 1.43}, Pointf{0.675, -0.59}}, KartPoint{1.43, 0}, 10};
    
    // Update parameters
    aeb.update_velocity(5.0);
    aeb.update_steering(0.0);
    aeb.update_ttc(2.0);
    
    // Add some (very fake) obstacle readings
    auto points = {KartPoint{1, 0}, KartPoint{10, 5}};
    aeb.add_points(points.begin(), points.end());
    
    // Collision detect!
    auto [should_stop, collides_at] = aeb.collision_check();
    
    printf("Collides at: %uz", collides_at);
}
```