//
// Created by chuchu on 12/28/25.
//
#include "common.h"
#include  <Eigen/Core>

#include "../include/DataType.h"

namespace  math {
    void computeMeanAndCov(const std::vector<Vec3f>& pts, Vec3f& out_mu, Mat3f& out_sig) {
        float min_var = 1e-4f;
        out_mu.setZero();
        out_sig.setZero();
        const int n = pts.size();

        if(n == 0) return;
        for(const auto& pt : pts) out_mu += pt;
        out_mu /= static_cast<float>(n);

        if(n==1) {
            out_sig = min_var* Mat3f::Identity();
            return;
        }

        Mat3f S = Mat3f::Zero();
        for(const auto& pt : pts) {
           const Vec3f d = pt - out_mu;
           S.noalias() += d * d.transpose();
        }
        //Prevents floating-point asymmetry → stable eigensolvers
        out_sig = S / static_cast<float>(n);
        out_sig = 0.5f * (out_sig + out_sig.transpose());
        //“No voxel is allowed to be infinitely confident in any direction.”
        out_sig.diagonal().array() += min_var;
    }

    void updateMeanAndCov(const std::vector<Vec3f>&pts, const int old_size, const Vec3f& old_mu ,const Mat3f& old_sig, Vec3f& out_mu, Mat3f& out_sig) {
        const float n0 = static_cast<float>(old_size);
        const float n1 = static_cast<float>(pts.size());
        const float n  = n0 + n1;

        out_mu.setZero();
        out_sig.setZero();

        if (n <= 0.f) return;

        Vec3f in_mu = Vec3f::Zero();
        Mat3f in_sig = Mat3f::Zero();
        if (!pts.empty()) {
            computeMeanAndCov(pts, in_mu, in_sig); // MUST be MLE (/N) for NDT
        }

        if (old_size <= 0) { out_mu = in_mu; out_sig = in_sig; }
        else if (pts.empty()) { out_mu = old_mu; out_sig = old_sig; }
        else {
            out_mu = (n0 * old_mu + n1 * in_mu) / n;

            const Vec3f d0 = old_mu - out_mu;
            const Vec3f d1 = in_mu  - out_mu;

            Mat3f old_sig_change = old_sig + d0 * d0.transpose();
            Mat3f in_sig_change  = in_sig  + d1 * d1.transpose();

            out_sig = (n0 * old_sig_change + n1 * in_sig_change) / n;
        }

        // stable / invertible for NDT
        out_sig = 0.5f * (out_sig + out_sig.transpose());
        out_sig.diagonal().array() += 1e-4f;
    }
}
