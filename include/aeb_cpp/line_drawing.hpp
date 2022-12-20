#pragma once

/**
 * Contains line drawing algorithms, for use in rasterisation.
 */

#include <iterator>
#include <cmath>
#include <cassert>

namespace LineDrawing {

    /// A point in 2d space.
    template<typename T>
    struct Point {
        T x;
        T y;

        Point(T x, T y) : x(x), y(y) {}

        Point(const std::tuple<T, T> &t) { // NOLINT(google-explicit-constructor)
            auto [tx, ty] = t;
            x = tx;
            y = ty;
        }

        Point() = default;
    };

    typedef Point<float> Pointf;
    typedef Point<size_t> Pointl;

    /// A line in 2d space.
    template<typename T>
    struct Line {
        Point<T> p1;
        Point<T> p2;
    };

    typedef Line<float> Linef;
    typedef Line<size_t> Linel;

    class Octant {
        uint8_t value;

    public:
        explicit Octant() {
            value = 0;
        }

        /// Get the relevant octant from a start and end point.
        template<typename T>
        explicit Octant(const Point<T> &start, const Point<T> &end) {
            value = 0;
            auto dx = end.x - start.x;
            auto dy = end.y - start.y;

            if (dy < 0.0) {
                dx = -dx;
                dy = -dy;
                value += 4;
            }

            if (dx < 0.0) {
                auto temp = dx;
                dx = dy;
                dy = -temp;
                value += 2;
            }

            if (dx < dy) {
                value += 1;
            }
        }

        /// Convert a point to its position in the octant.
        template<typename T>
        [[nodiscard]] Point<T> to(const Point<T> &point) const noexcept {
            switch (value) {
                case 0:
                    return Point{point.x, point.y};

                case 1:
                    return Point{point.y, point.x};

                case 2:
                    return Point{point.y, -point.x};

                case 3:
                    return Point{-point.x, point.y};

                case 4:
                    return Point{-point.x, -point.y};

                case 5:
                    return Point{-point.y, -point.x};

                case 6:
                    return Point{-point.y, point.x};

                case 7:
                    return Point{point.x, -point.y};
            }
            assert("We cannot reach here!");
            return Point<T>{};
        }

        /// Convert a point from its position in the octant.
        template<typename T>
        [[nodiscard]] Point<T> from(const Point<T> &point) const noexcept {
            switch (value) {
                case 0:
                    return Point{point.x, point.y};

                case 1:
                    return Point{point.y, point.x};

                case 2:
                    return Point{-point.y, point.x};

                case 3:
                    return Point{-point.x, point.y};

                case 4:
                    return Point{-point.x, -point.y};

                case 5: // NOLINT(bugprone-branch-clone)
                    return Point{-point.y, -point.x};

                case 6:
                    return Point{point.y, -point.x};

                case 7:
                    return Point{point.x, -point.y};
            }
            assert("We cannot reach here!");
            return Point<T>{};
        }
    };

    /// Components for the midpoint algorithm.
    namespace MidPoint {
        /// Iterator that implements the midpoint algorithm.
        ///
        /// Adapted from https://github.com/andyblarblar/line_drawing/blob/master/src/midpoint.rs
        template<typename I, typename O>
        class MidpointIter {
            using iterator_category = std::input_iterator_tag;
            using value_type = Point<O>;
            using pointer = Point<O> *;  // or also value_type*
            using reference = Point<O> &;  // or also value_type&

            /// Marks if this iterator is complete.
            bool done{};

            Octant octant;
            /// The WIP point
            Point<O> point;
            /// The last produced point.
            Point<O> out_point;

            // State variables
            I a;
            I b;
            I k;
            O end_x;

        private:

            /// Increments the algorithm, storing the result in this->out_point.
            void next() noexcept {
                if (point.x <= end_x) {
                    out_point = octant.from(point);

                    // Take an N step
                    if (k <= (I) 0.0) {
                        k += b;
                        point.y += 1.0;
                    }

                    // Take an E step
                    k += a;
                    point.x += 1.0;
                } else {
                    done = true;
                }
            }

            /// Creates an finished midpoint iterator for comparisons.
            explicit MidpointIter() {
                this->done = true;
            }

        public:
            explicit MidpointIter(const Point<I> &start, const Point<I> &end) {
                // Get the octant to use
                octant = Octant{start, end};

                // Convert the points into the octant versions
                auto start_2 = octant.to(start);
                auto end_2 = octant.to(end);

                // Initialise the variables
                a = -(end_2.y - start_2.y);
                b = end_2.x - start_2.x;
                auto c = start_2.x * end_2.y - end_2.x * start_2.y;

                point = Point{(O) std::round(start_2.x), (O) std::round(start_2.y)};
                k = a * (std::round(start_2.x) + 1) + b * (std::round(start_2.y) + (I) 0.5) + c;
                end_x = (O) std::round(end_2.x);

                // Prime the iterator with the first value.
                next();
            }

            reference operator*() {
                return out_point;
            }

            pointer operator->() {
                return &out_point;
            }

            MidpointIter<I, O> &operator++() {
                next();
                return *this;
            }

            template<typename OtherI, typename OtherO>
            friend bool operator==(const MidpointIter &l, const MidpointIter<OtherI, OtherO> &r) {
                return l.done == r.done;
            }

            template<typename OtherI, typename OtherO>
            friend bool operator!=(const MidpointIter &l, const MidpointIter<OtherI, OtherO> &r) {
                return l.done != r.done;
            }

            /// Returns an iterator that is done, which can be compared to.
            static MidpointIter end() {
                return MidpointIter{};
            }
        };

        /// Midpoint iterator container.
        template<typename I, typename O>
        class Midpoint {
            MidpointIter<I, O> state;
            const MidpointIter<I, O> term = MidpointIter<I, O>::end();

        public:
            explicit Midpoint(const Point<I> &start, const Point<I> &end) : state(start, end) {}

            MidpointIter<I, O> &begin() {
                return state;
            }

            const MidpointIter<I, O> &end() {
                return term;
            }
        };
    }

    /// Midpoint line drawing algorithm.
    ///
    /// \tparam I Input number type. Can be floating.
    /// \tparam O Output number type. Must be signed, generally an integer as it can then be used as an index.
    /// \return An object implementing an input iterator over the points generated by the midpoint algorithm.
    template<typename I, typename O>
    MidPoint::Midpoint<I, O> midpoint(Point<I> &start, Point<I> &end) noexcept {
        // Assert O invariants
        {
            constexpr O sample = -10;
            static_assert(sample < 0, "O should be a signed number!");
        }
        return MidPoint::Midpoint<I, O>{start, end};
    }
}
