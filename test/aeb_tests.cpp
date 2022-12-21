#include "gtest/gtest.h"
#include "aeb_cpp/aeb.hpp"

using namespace LineDrawing;

TEST(AebTests, PredictionWorks) {
    auto aeb = Aeb<71>{5, 0, 3, std::tuple{Pointf{-0.5, 3}, Pointf{0.5, -0.2}}, KartPoint{3.1, 0}, 10};

    // Straight line
    auto [pos, yaw] = aeb.predict_pos(1000);
    EXPECT_NEAR(pos.x, 5 - 3.1, 0.0001);
    EXPECT_EQ(yaw, 0);

    //TODO create more tests off IRL data, particularly with turning
}

TEST(AebTests, FullAebWroks) {
    auto aeb = Aeb<411>{3, 0, 1.08, std::tuple{Pointf{-0.675, 1.43}, Pointf{0.675, -0.59}}, KartPoint{1.43, 0}, 10};

    // Single point in front
    {
        auto points = {KartPoint{1, 0}};
        aeb.add_points(points.begin(), points.end());

        auto [collides, time] = aeb.collision_check();

        EXPECT_TRUE(collides);
    }

    // Points on edge of kart
    {
        auto points = {KartPoint{7, 0.675}, KartPoint{7, -0.675}};
        aeb.add_points(points.begin(), points.end());

        auto [collides, time] = aeb.collision_check();

        EXPECT_TRUE(collides);
    }


    // Points off edges dont collide
    {
        auto points = {KartPoint{7, 0.716}, KartPoint{7, -0.716}};
        aeb.add_points(points.begin(), points.end());

        auto [collides, time] = aeb.collision_check();

        EXPECT_FALSE(collides);
    }
}