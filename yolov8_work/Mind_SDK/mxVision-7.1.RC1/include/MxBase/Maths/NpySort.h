/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2021. All rights reserved.
 * Description: The Sort Method of Python Implemented in C++.
 * Author: MindX SDK
 * Create: 2020
 * History: NA
 */

#ifndef MINDXSDK_NPYAQUICKSORT_H
#define MINDXSDK_NPYAQUICKSORT_H

#include <vector>
#include "MxBase/Log/Log.h"

const int SMALL_QUICKSORT = 15;
const int HEAP_SORT_TWO = 2;
const int MIN_N = -2;
const float EPSILON = 1e-6;

class NpySort {
public:
    explicit NpySort() = default;

    NpySort(std::vector<float> preSortVec, std::vector<int> sortIdx);

    ~NpySort() {};

    void NpyArgQuickSort(bool reverse);

    void NpyArgHeapSort(int tosort, int n);

    std::vector<int> GetSortIdx();

private:
    std::vector<float> preSortVec_;
    std::vector<int> sortIdx_;
    void SwapValueForHeapSort(int iV, int jV, int n, int tmp, std::vector<int> &sortIdxHeap);

    int Compare(float pa, float pb);

    int NpyGetMsb(uint unum);

    void QuickSort(int &pl, int &pr, std::vector<int> &sptr, std::vector<int> &psdepth, int &cdepth);

    void InsertSort(int &pl, int &pr);

    bool CheckVectors();
};


#endif // MINDXSDK_NPYAQUICKSORT_H
