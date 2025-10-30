/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: mathematical basic operation.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef FASTMATH_H
#define FASTMATH_H

#include <cmath>
#include <memory>
#include <vector>
#include <numeric>
/*
Utilize quantization and look up table to accelate exp operation
*/
class FastMath {
public:
    FastMath()
    {
        for (auto i = 0; i < maskLen_; i++) {
            negCoef_[0][i] = std::exp(-float(i) / quantValue_);
            negCoef_[1][i] = std::exp(-float(i) * maskLen_ / quantValue_);
            posCoef_[0][i] = std::exp(float(i) / quantValue_);
            posCoef_[1][i] = std::exp(float(i) * maskLen_ / quantValue_);
        }
    }
    ~FastMath() {}
    inline float FExp(const float x)
    {
        int quantX = static_cast<int>(std::max(std::min(x, float(quantBound_)), -float(quantBound_)) * quantValue_);
        float expx;
        if (quantX & 0x80000000) {
            expx = negCoef_[0][((~quantX + 0x00000001)) & maskValue_] *
                    negCoef_[1][((~quantX + 0x00000001) >> maskBits_) & maskValue_];
        } else {
            expx = posCoef_[0][(quantX) & maskValue_] * posCoef_[1][(quantX >> maskBits_) & maskValue_];
        }
        return expx;
    }
    inline float Sigmoid(float x)
    {
        return 1.0f / (1.0f + FExp(-x));
    }
    void Softmax(std::vector<float>& digits)
    {
        std::vector<float> expDigits = {};
        for (size_t i = 0; i < digits.size(); i++) {
            expDigits.push_back(FExp(digits[i]));
        }
        float sum = 0.0;
        sum = std::accumulate(expDigits.begin(), expDigits.end(), sum);
        if (std::fabs(sum) < 1e-6) {
            expDigits.clear();
            return;
        }
        for (size_t i = 0; i < digits.size(); i++) {
            digits[i] = expDigits[i] / sum;
        }
        expDigits.clear();
    }
    inline float sign(float x)
    {
        if (x > 0.0f) {
            return 1.0f;
        } else if (x < 0.0f) {
            return -1.0f;
        } else {
            return 0.0f;
        }
    }

private:
    static const int maskBits_ = 12;
    static const int maskLen_ = (1 << maskBits_);
    static const int maskValue_ = maskLen_ - 1;
    static const int quantBits_ = 16;
    static const int quantValue_ = (1 << quantBits_);
    static const int quantBound_ = (1 << (2 * maskBits_ - quantBits_)) - 1;
    float negCoef_[2][maskLen_];
    float posCoef_[2][maskLen_];
};

namespace fastmath {
    static FastMath fastMath;
    inline float exp(const float x)
    {
        return fastMath.FExp(x);
    }
    inline float sigmoid(float x)
    {
        return fastMath.Sigmoid(x);
    }
    inline void softmax(std::vector<float>& digits)
    {
        fastMath.Softmax(digits);
    }
    inline float sign(float x)
    {
        return fastMath.sign(x);
    }
}

#endif