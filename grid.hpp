/**
 * Deterministic occupancy grid and associated utilities
 */

#pragma once

#include <cstddef>
#include <limits>
#include <array>
#include <exception>
#include <cmath>
#include <tuple>
#include <optional>
#include <ostream>

struct Point {
    float x;
    float y;
};

struct Line {
    Point p1;
    Point p2;
};

/// Converts degrees to rad
constexpr float dtor(const float degree) {
    return (degree * (float) M_PI) / 180.0f;
}

/// An index indo the grid, garrentied to be in bounds. We define a point on the grid to be the top left corner of the grid squares
/// if a grid is drawn out. The top left of the grid is (0,0).
struct GridPoint {
    size_t row;
    size_t col;

private:
    GridPoint(size_t row, size_t col) : row(row), col(col) {}

    // Gridpoints must come from kartpoints or float points, to maintain their invariants.
    friend class KartPoint;

    friend class GridPointf;
};

/// A point in gridspace, not aligned to the grid nor bounds checked.
struct GridPointf {
    float row;
    float col;

    GridPointf(float row, float col) : row(row), col(col) {}

    /// Transforms this point into a grid point, if in bounds.
    [[nodiscard]] std::optional<GridPoint> into_gridpoint(const size_t grid_size) const noexcept {
        const auto [r, c] = *this;

        if (r < 0.0 || c < 0.0 || r >= (float) grid_size || c >= (float) grid_size) {
            return std::nullopt;
        } else {
            return GridPoint{(size_t) r, (size_t) c};
        }
    }

    /// Gets this point as a tuple.
    [[nodiscard]] std::tuple<float, float> raw() const noexcept {
        return std::tuple<float, float>{row, col};
    }
};

/// A point in the reference frame of the kart, where (0,0) is the location of the front range sensor.
///
/// This is in (x,y), where x is parallel to the kart and positive y is perpendicular to the right of the kart.
struct KartPoint {
    float x;
    float y;

    explicit KartPoint(float x, float y) : x(x), y(y) {}

    /// Creates a point in kart frame from a polar coordinate in kart frame.
    /// Note that 0 degrees is still pointing to the right.
    /// Theta is in degrees.
    static KartPoint from_polar(const float r, const float theta) noexcept {
        auto th = dtor(theta);

        auto x = r * sinf(th);
        auto y = r * cosf(th);

        return KartPoint{x, y};
    }

    /// Transforms a point from the frame of the kart to the frame of the grid. This function will not
    /// apply any bounds checking.
    [[nodiscard]] GridPointf transform_to_gridf(const std::size_t grid_size) const noexcept {
        const auto n = grid_size;

        // This is the grid scale
        const auto m = 10.0f / (float) n;

        const auto [r, c] = *this;

        // Linear transform as defined in docs
        const auto out_r = (float) n - (r / m);
        const auto out_c = (((float) n - 1.0f) / 2.0f) + (c / m);

        return GridPointf{out_r, out_c};
    }

    /// Transforms a point from the frame of the kart to the frame of the grid, if in bounds.
    [[nodiscard]] std::optional<GridPoint> transform_to_grid(const std::size_t grid_size) const noexcept {
        return this->transform_to_gridf(grid_size).into_gridpoint(grid_size);
    }
};

/// A cell in the occupancy grid.
enum class Cell {
    Unoccupied,
    Occupied
};

/// An nxn occupancy grid.
///
/// \tparam N the size of one size of the grid, must be odd and < f32::max.
template<std::size_t N>
struct Grid {
    std::array<std::array<Cell, N>, N> data{};

    explicit Grid() {
        // Assert N is valid
        static_assert(N % 2 != 0, "N must be odd!");
        static_assert(N < std::numeric_limits<float>::max(), "N must be odd!");
    }

    /// Marks a cell as occupied.
    void mark_occupied(const GridPoint idx) noexcept {
        data[idx.row][idx.col] = Cell::Occupied;
    }

    /// Checks if a cell is occupied.
    void is_occupied(const GridPoint idx) const noexcept {
        return data[idx.row][idx.col] == Cell::Occupied;
    }

    /// Gets size.
    [[nodiscard]] size_t get_size() const noexcept {
        return N;
    }

    /// Resets the occupancy grid to a fully free state.
    void reset() noexcept {
        data = std::array<std::array<Cell, N>, N>{};
    }

    //TODO draw and check polygon

    Cell &operator[](GridPoint idx) const noexcept {
        return data[idx.row][idx.col];
    }

    friend std::ostream &operator<<(std::ostream &os, const Grid &grid) {
        os << "data: " << grid.data;
        return os;
    }
};
