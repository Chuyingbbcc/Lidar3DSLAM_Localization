//
// Created by chuchu on 12/28/25.
//
#include "common.h"
#include  <Eigen/Core>

#include "../include/DataType.h"

namespace  math {
    void computeMeanAndCov(const std::vector<Vec3d>& pts, Vec3d& out_mu, Mat3d& out_sig) {
        double min_var = 1e-4f;
        out_mu.setZero();
        out_sig.setZero();
        const int n = pts.size();

        if(n == 0) return;
        for(const auto& pt : pts) out_mu += pt;
        out_mu /= static_cast<double>(n);

        if(n==1) {
            out_sig = min_var* Mat3d::Identity();
            return;
        }

        Mat3d S = Mat3d::Zero();
        for(const auto& pt : pts) {
           const Vec3d d = pt - out_mu;
           S.noalias() += d * d.transpose();
        }
        //Prevents floating-point asymmetry → stable eigensolvers
        out_sig = S / static_cast<double>(n);
        out_sig = 0.5f * (out_sig + out_sig.transpose());
        //“No voxel is allowed to be infinitely confident in any direction.”
        out_sig.diagonal().array() += min_var;
    }

    void updateMeanAndCov(const std::vector<Vec3d>&pts, const int old_size, const Vec3d& old_mu ,const Mat3d& old_sig, Vec3d& out_mu, Mat3d& out_sig) {
        const double n0 = static_cast<double>(old_size);
        const double n1 = static_cast<double>(pts.size());
        const double n  = n0 + n1;

        out_mu.setZero();
        out_sig.setZero();

        if (n <= 0.f) return;

        Vec3d in_mu = Vec3d::Zero();
        Mat3d in_sig = Mat3d::Zero();
        if (!pts.empty()) {
            computeMeanAndCov(pts, in_mu, in_sig); // MUST be MLE (/N) for NDT
        }

        if (old_size <= 0) { out_mu = in_mu; out_sig = in_sig; }
        else if (pts.empty()) { out_mu = old_mu; out_sig = old_sig; }
        else {
            out_mu = (n0 * old_mu + n1 * in_mu) / n;

            const Vec3d d0 = old_mu - out_mu;
            const Vec3d d1 = in_mu  - out_mu;

            Mat3d old_sig_change = old_sig + d0 * d0.transpose();
            Mat3d in_sig_change  = in_sig  + d1 * d1.transpose();

            out_sig = (n0 * old_sig_change + n1 * in_sig_change) / n;
        }

        // stable / invertible for NDT
        out_sig = 0.5f * (out_sig + out_sig.transpose());
        out_sig.diagonal().array() += 1e-4f;
    }
}
