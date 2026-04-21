#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"

class Monitor; // declared in monitor.h

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
            if (project >= 0) continue;
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

        double desired_speed = v_max;
        if (dist < v_max * TIME_INTERVAL) desired_speed = dist / TIME_INTERVAL;
        Vec base_dir = to_tar.normalize();

        static const double angles[] = {0.0,
                                        0.34906585, -0.34906585,
                                        0.6981317, -0.6981317,
                                        1.04719755, -1.04719755,
                                        1.57079633, -1.57079633};
        static const double scales[] = {1.0, 0.8, 0.6, 0.4, 0.2, 0.0};

        if (monitor) {
            int n = monitor->get_robot_number();
            double min_gap = 1e18;
            for (int j = 0; j < n; ++j) if (j != id) {
                double rj = monitor->get_r(j);
                double gap = (monitor->get_pos_cur(j) - pos_cur).norm() - (r + rj);
                if (gap < min_gap) min_gap = gap;
            }
            if (min_gap < 0.5) desired_speed *= 0.0;
            else if (min_gap < 1.5) desired_speed *= 0.5;
            else if (min_gap < 3.0) desired_speed *= 0.8;
        }

        for (double sc : scales) {
            double sp = desired_speed * sc;
            for (double ang : angles) {
                Vec cand = rotate_vec(base_dir, ang) * sp;
                if (!will_collide_with_any(cand)) return cand;
            }
        }
        return Vec();
    }
};

#endif //PPCA_SRC_HPP
