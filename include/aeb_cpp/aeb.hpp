#pragma once

#include <tuple>
#include <utility>
#include "grid.hpp"

/// The entrypoint to the automatic emergency braking algorithm.
template<size_t N>
class Aeb {
private:
    /// The occupancy grid
    Grid<N> grid{};
    /// Current velocity of the kart in m/s
    float volocity;
    /// Current angle of the virtual ackermann wheel, in degrees, positive is right.
    float steering_angle;
    /// The distance between axles, in meters
    float wheelbase;
    /// The collision box of the vehicle, defined by the top left point of the box relative to the center of the rear axel,
    /// and the bottom right point relative to the rear axel.
    std::tuple<LineDrawing::Pointf, LineDrawing::Pointf> collision_box;
    /// Rear axel to range sensor transform
    KartPoint axel_to_range;
    /// The minimum allowed time to collision.
    float min_ttc;
    /// ms timestep used in the AEB loop. Greatly increases complexity of the algorithm if set to a small value.
    size_t timestep;

    /// Rotates a point about another, by the angle in rad.
    [[nodiscard]] KartPoint
    rotate_point_about(const KartPoint &p, const KartPoint &origin, const float angle) const noexcept {
        auto [x1, y1] = p;
        auto [ox, oy] = origin;

        auto x2 = (x1 - ox) * cosf(angle) - (y1 - oy) * sinf(angle) + ox;
        auto y2 = (x1 - ox) * sinf(angle) + (y1 - oy) * cosf(angle) + oy;

        return KartPoint{x2, y2};
    }

public:
    /// Predicts the future position of the kart given the currently configured params, at some time.
    ///
    /// Returns final (position, heading), where heading is in rad and negative if left.
    ///
    /// \param t time in ms
    [[nodiscard]] std::tuple<KartPoint, float> predict_pos(const size_t time) const noexcept {
        const auto t = static_cast<float>(time) / 1000.0f;
        const auto phi = dtor(steering_angle);
        const auto s = volocity;
        const auto l = wheelbase;
        const auto [axel_x, axel_y] = axel_to_range;

        if (phi == 0.0) {
            const auto displacement = t * s;
            return std::tuple(KartPoint{displacement - axel_x, 0 - axel_y}, 0.0);
        }

        // Forward kinematics equations integrated over time. Based on: https://www.xarg.org/book/kinematics/ackerman-steering/
        const auto x = l * (1.0f / tanf(phi)) * sinf((s * t * tanf(phi)) / l);
        const auto y = l * (1.0f / tanf(phi)) * (-cosf((s * t * tanf(phi)) / l) + 1.0f);

        // Integrate heading separately for OBB
        const auto heading = (s * t * tanf(phi)) / l;

        // Translate the rear axel to its real location by pushing it back by its distance to the range sensor
        return std::tuple{
                KartPoint{x - axel_x, y - axel_y},
                heading
        };
    }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "ArgumentSelectionDefects"

    /// Creates the oriented bounding box given the karts position and yaw in rad.
    ///
    /// This box may lie outside the grid partially or totally.
    [[nodiscard]] std::array<LineDrawing::Linef, 4> create_obb(const KartPoint &pos, const float yaw) const noexcept {
        const auto n = grid.get_size();

        // Swap axis to make me sane
        const auto [pred_y, pred_x] = pos;
        const auto [tl, br] = collision_box;
        const auto [tlx, tly] = tl;
        const auto [brx, bry] = br;

        // Define box lines
        const auto tl_to_tr = LineDrawing::Linef{
                rotate_point_about(KartPoint{pred_y + tly, pred_x + tlx}, pos, yaw).transform_to_gridf(n).raw(),
                rotate_point_about(KartPoint{pred_y + bry, pred_x + tlx}, pos, yaw).transform_to_gridf(n).raw()
        };
        const auto tr_to_br = LineDrawing::Linef{
                rotate_point_about(KartPoint{pred_y + bry, pred_x + tlx}, pos, yaw).transform_to_gridf(n).raw(),
                rotate_point_about(KartPoint{pred_y + bry, pred_x + brx}, pos, yaw).transform_to_gridf(n).raw()
        };
        const auto br_to_bl = LineDrawing::Linef{
                rotate_point_about(KartPoint{pred_y + bry, pred_x + brx}, pos, yaw).transform_to_gridf(n).raw(),
                rotate_point_about(KartPoint{pred_y + tly, pred_x + brx}, pos, yaw).transform_to_gridf(n).raw()
        };
        const auto bl_to_tl = LineDrawing::Linef{
                rotate_point_about(KartPoint{pred_y + tly, pred_x + brx}, pos, yaw).transform_to_gridf(n).raw(),
                rotate_point_about(KartPoint{pred_y + tly, pred_x + tlx}, pos, yaw).transform_to_gridf(n).raw()
        };

        return std::array{tl_to_tr, tr_to_br, br_to_bl, bl_to_tl};
    }

#pragma clang diagnostic pop

    /// Updates the current velocity in m/s.
    void update_velocity(const float velocity) noexcept {
        this->velocity = volocity;
    }

    /// Updates the current ackermann steering angle, in degrees.
    void update_steering(const float steering) noexcept {
        this->steering_angle = steering;
    }

    /// Updates the minimum allowed time to collision, in seconds.
    ///
    /// The minimum time to collision will be the boundary that we will collision detect to, ie
    /// the minimum tolerated collision time.
    void update_ttc(const float ttc) noexcept {
        this->min_ttc = ttc;
    }

    /// Gets the currently configured timestep.
    [[nodiscard]] size_t get_timestep() const {
        return timestep;
    }

    /// Updates the timestep (in milliseconds) taken in the AEB loop. The precision of collision times is equal to this value,
    /// but collision detection itself will remain accurate. Setting the timestep to a small value
    /// will significantly increase the complexity of collision checking, so change with care.
    ///
    /// Values of 100-500 are reasonable, defaults to 500.
    void update_timestep(size_t timestepn) {
        this->timestep = timestepn;
    }

    /// Gets the currently configured steering angle.
    float get_steering() {
        return this->steering_angle;
    }

    /// Gets the currently configured velocity.
    float get_velocity() {
        return this->volocity;
    }

    /// Gets the currently configured time to collision.
    float get_ttc() {
        return this->min_ttc;
    }

    /// Adds points to the grid to be used during the next collision check.
    ///
    /// Points that are < 15cm from the data source will be filtered out.
    /// \param points_iter A begin end iterator pair over KartPoints.
    template<class Iter1, typename Iter2>
    void add_points(Iter1 &&points_iter_begin, Iter2 &&points_iter_end) noexcept {
        for (; points_iter_begin != points_iter_end; points_iter_begin++) {
            auto p = *points_iter_begin;
            // If point is less than 15cm from kart, filter out
            if (p.x < 0.15f) {
                continue;
            }

            // Transform point to grid, skipping if out of bounds
            auto gp = p.transform_to_grid(grid.get_size());
            if (gp) {
                grid.mark_occupied(*gp);
            }
        }
    }

    /// Runs the collision checking algorithm with the current parameters.
    /// After this call, the grid will be cleared.
    ///
    /// Returns true if the vehicle should brake, along with it's collision time in ms.
    std::tuple<bool, size_t> collision_check() noexcept {
        // Convert ttc to millis
        auto ttc = static_cast<size_t>(min_ttc * 1000.0f);

        // Collision check by integrating over our model
        for (size_t t = 0; t < ttc; t += this->timestep) {
            // Predict position at time with definite integral of forward kinematics
            auto [pos, heading] = predict_pos(t);

            auto obb = create_obb(pos, heading);

            // Actually collision check
            if (grid.polygon_collide(obb.begin(), obb.end())) {
                grid.reset();
                return std::tuple{true, t};
            }
        }

        grid.reset();
        return std::tuple{false, 0};
    }

    void print_grid() {
        grid.print();
    }

    /// Configures AEB.
    ///
    /// # Config
    ///
    /// - initial_vel: starting velocity, in m/s
    /// - initial_steering: starting ackermann wheel angle, in degrees, positive is right.
    /// - wheelbase: distance between axles, in meters.
    /// - collision_box: collision bounding box, defined as the top left and bottom right points relative
    /// to the center of the rear axel. This is in normal coordinates.
    /// - axel_to_sensor: The transform of the axel to the front range sensor, in the kart frame.
    /// - min_ttc: the minimum allowed time to collision.
    /// - timestep: The precision of the collision detection, in ms. Lower values have more precision, but greatly
    /// increase complexly.
    Aeb(float initial_velocity, float inital_steering, float wheelbase,
        std::tuple<LineDrawing::Pointf, LineDrawing::Pointf> collision_box, const KartPoint &axel_to_range,
        float min_ttc, size_t timestep = 50) noexcept:
            volocity(initial_velocity),
            steering_angle(
                    inital_steering),
            wheelbase(
                    wheelbase),
            collision_box(std::move(
                    collision_box)),
            axel_to_range(
                    axel_to_range),
            min_ttc(min_ttc), timestep(timestep) {}
};