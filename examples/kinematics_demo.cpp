#include <vector>
#include <thread>
#include "aeb_cpp/aeb.hpp"

using namespace LineDrawing;
using namespace std::chrono_literals;

/**
 * Plays an animation of the predictions created by the kinematics function
 * @author Andrew Ealovega
 */
int main() {
    auto aeb = Aeb<51>{3, 0, 1.08, std::tuple{Pointf{-0.675, 1.43}, Pointf{0.675, -0.59}}, KartPoint{1.43, 0}, 10};
    Grid<51> g{};

    // Loop over steering angle
    for (int i = 0; i < 29; ++i) {
        aeb.update_steering(float(i));

        for (int t = 0; t < 3000; t += 20) {
            auto [pred_p, pred_y] = aeb.predict_pos(t);

            if (auto point = pred_p.transform_to_grid(g.get_size())) {
                g.mark_occupied(*point);
            }

            auto obb = aeb.create_obb(pred_p, pred_y);
            g.draw_polygon(obb.begin(), obb.end());

            printf("angle: %d, time: %dms\n", i, t);
            g.print();
            fputs("\x1b[2j", stdout);
            g.reset();

            std::this_thread::sleep_for(15ms);
        }
        g.reset();
    }
}