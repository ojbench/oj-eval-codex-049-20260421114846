#pragma once

#include <bits/stdc++.h>
using std::vector; using std::min; using std::max; using std::sqrt; using std::cos; using std::sin; using std::fabs;

struct Vec {
    double x, y;
    Vec(double x_=0, double y_=0): x(x_), y(y_) {}
    Vec operator+(const Vec& o) const { return Vec(x+o.x, y+o.y); }
    Vec operator-(const Vec& o) const { return Vec(x-o.x, y-o.y); }
    Vec operator*(double k) const { return Vec(x*k, y*k); }
    Vec operator/(double k) const { return Vec(x/k, y/k); }
    Vec& operator+=(const Vec& o){ x+=o.x; y+=o.y; return *this; }
    Vec& operator-=(const Vec& o){ x-=o.x; y-=o.y; return *this; }
    double dot(const Vec& o) const { return x*o.x + y*o.y; }
    double cross(const Vec& o) const { return x*o.y - y*o.x; }
    double norm2() const { return x*x + y*y; }
    double norm() const { return std::sqrt(norm2()); }
    Vec unit() const { double n = norm(); if(n==0) return Vec(0,0); return *this / n; }
};

struct Monitor {
    virtual bool get_speeding(int id) const = 0;
    virtual std::vector<int> get_collision(int id) const = 0;
    virtual bool get_warning() const = 0;
    virtual Vec get_pos_cur(int id) const = 0;
    virtual Vec get_v_cur(int id) const = 0;
    virtual double get_r(int id) const = 0;
    virtual bool get_done() const = 0;
    virtual int get_robot_number() const = 0;
    virtual int get_test_id() const = 0;
    virtual ~Monitor() = default;
};

struct Controller {
    // Current robot state (read-only filled by framework)
    Vec pos_cur, v_cur, pos_tar;
    double r = 0.0, v_max = 0.0;
    int id = 0;
    const Monitor* monitor = nullptr;

    // User-implemented method to choose next velocity
    Vec get_v_next() const {
        // Simple rule: drive toward target with capped speed.
        Vec dir = pos_tar - pos_cur;
        double dist = dir.norm();
        if(dist < 1e-6) return Vec(0,0);
        Vec desire = dir.unit() * v_max;
        // Conservative slowing near neighbors: if too close, reduce speed.
        if(monitor){
            int n = monitor->get_robot_number();
            double min_gap = 1e18;
            for(int i=0;i<n;i++) if(i!=id){
                Vec pi = monitor->get_pos_cur(i);
                double ri = monitor->get_r(i);
                double gap = (pi - pos_cur).norm() - (ri + r);
                if(gap < min_gap) min_gap = gap;
            }
            if(min_gap < 0.5 * (r+1)){
                desire = desire * 0.0; // stop if very close
            } else if(min_gap < 2.0 * (r+1)){
                desire = desire * 0.3;
            } else if(min_gap < 4.0 * (r+1)){
                desire = desire * 0.6;
            }
        }
        return desire;
    }
};
