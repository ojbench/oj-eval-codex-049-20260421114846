#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include math.h

class Monitor; // forward declaration

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    Vec rotate_vec(const Vec &v, double theta) const {
        double c = std::cos(theta);
        double s = std::sin(theta);
        // Clockwise rotation to match math.h rotate convention
        return Vec(v.x * c + v.y * s, v.y * c - v.x * s);
    }

    bool will_collide_with_any(const Vec &v_next) const {
        if (!monitor) return false;
        int n = monitor->get_robot_number();
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec pj = monitor->get_pos_cur(j);
            Vec vj = monitor->get_v_cur(j);
            double rj = monitor->get_r(j);

            Vec delta_pos = pos_cur - pj;
            Vec delta_v = v_next - vj;
            double dv_norm = delta_v.norm();
            double delta_r = r + rj;

            if (dv_norm < 1e-9) {
                if (delta_pos.norm_sqr() <= delta_r * delta_r - 1e-9) return true;
                continue;
            }

            double project = delta_pos.dot(delta_v);
            if (project >= 0) continue; // moving apart
            project /= -dv_norm;

            double min_dis_sqr;
            if (project < dv_norm * TIME_INTERVAL) {
                min_dis_sqr = delta_pos.norm_sqr() - project * project;
            } else {
                Vec end_delta = delta_pos + delta_v * TIME_INTERVAL;
                min_dis_sqr = end_delta.norm_sqr();
            }
            if (min_dis_sqr <= delta_r * delta_r - 1e-9) return true;
        }
        return false;
    }

public:
    Vec get_v_next() {
        Vec to_tar = pos_tar - pos_cur;
        double dist = to_tar.norm();
        if (dist <= EPSILON) return Vec();

        // Attraction toward target
        Vec attract = to_tar.normalize();

        // Repulsion from neighbors (yield to lower-id robots)
        Vec repel(0.0, 0.0);
        double crowd_factor = 0.0;
        if (monitor) {
            int n = monitor->get_robot_number();
            for (int j = 0; j < n; ++j) if (j != id) {
                Vec pj = monitor->get_pos_cur(j);
                double rj = monitor->get_r(j);
                Vec d = pos_cur - pj;
                double dlen = d.norm();
                double safe = r + rj + 0.2; // small buffer
                double pen = ((0.0) > (safe - dlen) ? 0.0 : (safe - dlen));
                double w = (pen > 0 ? 1.5 + (j < id ? 0.8 : 0.2) : 0.0);
                if (dlen > 1e-9) repel += d.normalize() * (w * (safe / (dlen + 1e-6)));
                crowd_factor += (((0.0) > (safe / (dlen + 1e-6) - 1.0)) ? 0.0 : (safe / (dlen + 1e-6) - 1.0));
            }
        }

        Vec desired_dir = (attract * 1.0) + (repel * 0.8);
        if (desired_dir.norm() < 1e-9) desired_dir = attract;
        desired_dir = desired_dir.normalize();

        double desired_speed = v_max;
        if (dist < v_max * TIME_INTERVAL) desired_speed = dist / TIME_INTERVAL;
        // Slow down if crowded
        desired_speed *= 1.0 / (1.0 + 0.5 * crowd_factor);

        static const double angles[] = {
            0.0,
            0.261799387, -0.261799387,  // 15 deg
            0.523598775, -0.523598775,  // 30 deg
            0.785398163, -0.785398163,  // 45 deg
            1.047197551, -1.047197551,  // 60 deg
            1.570796327, -1.570796327   // 90 deg
        };
        static const double scales[] = {1.0, 0.9, 0.75, 0.6, 0.45, 0.3, 0.15, 0.0};

        for (double sc : scales) {
            double sp = desired_speed * sc;
            for (double ang : angles) {
                Vec cand = rotate_vec(desired_dir, ang) * sp;
                if (!will_collide_with_any(cand)) return cand;
            }
        }
        // Fallback: stop
        return Vec();
    }
};

#endif //PPCA_SRC_HPP
