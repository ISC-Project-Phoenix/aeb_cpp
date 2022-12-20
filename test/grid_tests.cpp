#include "gtest/gtest.h"
#include "aeb_cpp/grid.hpp"

TEST(GridTests, TransformPointWorks) {
    // Test normal point
    auto k = KartPoint{2, -3}.transform_to_grid(5);
    EXPECT_EQ(k->row, 4);
    EXPECT_EQ(k->col, 0);
    // Test exact max bounds
    auto k2 = KartPoint{10, 0}.transform_to_grid(5);
    EXPECT_TRUE(k2.has_value());
    // Test OOB
    auto k3 = KartPoint{10.1, 0}.transform_to_grid(5);
    EXPECT_FALSE(k3.has_value());
    // Test min
    auto k4 = KartPoint{0, 0}.transform_to_grid(5);
    EXPECT_FALSE(k4.has_value());
}

TEST(Gridtests, KartPointPolarWorks) {
    auto stright = KartPoint::from_polar(10, 90);
    EXPECT_NEAR(10, stright.x, 0.001);
    EXPECT_NEAR(0, stright.y, 0.001);

    auto right = KartPoint::from_polar(10, 0);
    EXPECT_NEAR(0, right.x, 0.001);
    EXPECT_NEAR(10, right.y, 0.001);

    auto left = KartPoint::from_polar(10, 180);
    EXPECT_NEAR(0, left.x, 0.001);
    EXPECT_NEAR(-10, left.y, 0.001);
}

TEST(GridTests, GridConstructs) {
    auto grid = Grid<5>{};

    EXPECT_EQ(5, grid.get_size());
}

TEST(GridTests, GridResets) {
    auto grid = Grid<5>{};
    grid[0][0] = Cell::Occupied;

    EXPECT_EQ(grid[0][0], Cell::Occupied);
    grid.reset();
    EXPECT_EQ(grid[0][0], Cell::Unoccupied);
}

TEST(GridTests, GridIndexes) {
    auto grid = Grid<5>{};

    auto idx = KartPoint{2, -3}.transform_to_grid(grid.get_size());
    grid.mark_occupied(*idx);
    EXPECT_TRUE(grid.is_occupied(*idx));

    auto idx2 = KartPoint{10, 0}.transform_to_grid(grid.get_size());
    grid.mark_occupied(*idx2);
    EXPECT_TRUE(grid.is_occupied(*idx2));
}

TEST(GridTests, PolygonMethodsWork) {
    using namespace LineDrawing;
    auto grid = Grid<41>{};

    std::vector tri1{
            Linef{std::tuple{1.0f, 1.0f}, std::tuple{10.0f, 10.0f}},
            Linef{std::tuple{1.0f, 1.0f}, std::tuple{1.0f, 44.0f}},
            Linef{std::tuple{1.0f, 44.0f}, std::tuple{10.0f, 10.0f}},
    };
    grid.draw_polygon(tri1.begin(), tri1.end());
    grid.print();

    std::vector tri2{
            Linef{std::tuple{1.0f, 1.0f}, std::tuple{10.0f, 10.0f}},
            Linef{std::tuple{1.0f, 1.0f}, std::tuple{1.0f, 14.0f}},
            Linef{std::tuple{10.0f, 14.0f}, std::tuple{10.0f, 10.0f}},
    };

    auto res = grid.polygon_collide(tri2.begin(), tri2.end());
    EXPECT_TRUE(res);
}