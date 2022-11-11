#pragma once

/**
 * Deterministic occupancy grid and associated utilities
 */

#include <cstddef>
#include <limits>
#include <array>
#include <cmath>
#include <tuple>
#include <optional>
#include <cstdio>
#include <cstdint>
#include "line_drawing.hpp"

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

namespace {
    /// A row registration used in rasterisation.
    struct RowReg {
        size_t start;
        std::optional<size_t> end;

        explicit RowReg(size_t start) : start(start) {
            end = std::nullopt;
        }

        explicit RowReg(size_t start, const std::optional<size_t> &anEnd) : start(start), end(anEnd) {}
    };
}

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

    /// Checks if a filled polygon overlaps with any occupied space.
    ///
    /// - Lines must form a closed polygon, else this function will fail.
    ///
    /// - This function will not fault if a point falls off the grid.
    ///
    /// - Be aware that this function will use usize*N stack space.
    template<typename Iter>
    bool polygon_collide(const Iter &lines) {
        bool collided = false;

        // Check for filled cells
        auto cb = [&collided](Cell &cell, size_t x, size_t y) mutable {
            collided |= cell == Cell::Occupied;
        };

        polygon_base(lines, cb);

        return collided;
    }

    /// Draws (Rasterizes) a filled polygon onto the grid. Lines must form a polygon, else this
    /// function will fail. This function will not fault if a point falls off the grid.
    ///
    /// Be aware that this function will use usize*N stack space.
    template<typename Iter>
    void draw_polygon(Iter &lines) {
        auto draw = [](Cell &cell, size_t x, size_t y) {
            cell = Cell::Occupied;
        };

        polygon_base(lines, draw);
    }

    Cell &operator[](GridPoint idx) const noexcept {
        return data[idx.row][idx.col];
    }

    /// Prints the grid
    void print() {
        auto m = 10.0f / (float) get_size();

        // Print numbers
        printf("     ");
        for (size_t col_n = 0; col_n < N; col_n++) {
            printf("%-4zu", col_n);
        }
        printf("\n");

        // Print data
        for (size_t i = 0; i < N; i++) {
            printf("%-4zu[ ", i);
            for (size_t j = 0; j < N; j++) {
                auto cell = data[i][j];

                if (cell == Cell::Occupied) {
                    printf("# ");
                } else {
                    printf("  ");
                }

                if (j + 1 != N) {
                    printf("| ");
                }
            }
            printf("]%4zu %4.2fm\n", i, 10.0f - (m * (float) i));

            // Add separating lines
            printf("    ");
            for (int r = 0; r < N; ++r) {
                printf("----");
            }
            printf("\n");
        }
    }

private:
    /// Basis for raster methods. Cb is a lambda of (&Cell, x, y) -> void called on each cell.
    template<typename Iter, typename Cb>
    void polygon_base(Iter &lines, Cb visitor) {
        // Array of the first and last points on each line. Row is index, col is in the reg
        std::array<std::optional<RowReg>, N> ends{};

        // Draw the lines, and record the ends of each row
        for (LineDrawing::Linef &line: lines) {
            for (LineDrawing::Point<int64_t> &p: LineDrawing::midpoint<float, int64_t>(line.p1, line.p2)) {
                // Handle points OOB as negitive
                auto x = (size_t) std::max(p.x, 0l);
                auto y = (size_t) std::max(p.y, 0l);

                // Register ends
                if (ends[x].has_value()) {
                    RowReg &reg = *ends[x];

                    if (reg.start == y) {
                        continue;
                    }

                    // If point is left-more than start, it is the starting point
                    if (y < reg.start) {
                        // Move old start to end if there is none (if some, then end will already be bigger)
                        if (!reg.end.has_value()) {
                            reg.end = reg.start;
                        }
                        reg.start = y;
                    }

                        // If there was no end, this point must be the biggest. If there is, then replace if new point is bigger
                    else if (!reg.end.has_value() || *reg.end < y) {
                        reg.end = y;
                    }
                } else if (x < N) {
                    ends[x] = RowReg{y, std::nullopt};
                }

                // Bounds check, then visit
                if (!(p.x < 0 || p.x >= N || p.y < 0 || p.y >= N)) {
                    visitor(data[x][y], x, y);
                }
            }
        }

        // Now fill in the inside of the polygon, bounded by the ends
        for (size_t x = 0; x < N; x++) {
            if (auto reg = ends[x]; reg) {
                // Ignore rows with only one cell filled
                if (reg->end) {
                    for (size_t y = reg->start; y < *reg->end; y++) {
                        // Bounds check
                        if (!(x >= N || y >= N)) {
                            visitor(data[x][y], x, y);
                        }
                    }
                }
            }
        }
    }
};
