//
// Created by runrunxin on 24-3-19.
//
///////////////////////////////////////////////////////////////////////////////
// Hungarian.h: Header file for Class HungarianAlgorithm.
//
// This is a C++ wrapper with slight modification of a hungarian algorithm implementation by Markus Buehren.
// The original implementation is a few mex-functions for use in MATLAB, found here:
// http://www.mathworks.com/matlabcentral/fileexchange/6543-functions-for-the-rectangular-assignment-problem
//
// Both this code and the orignal code are published under the BSD license.
// by Cong Ma, 2016
//

#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#include <iostream>
#include <vector>

using namespace std;


class HungarianAlgorithm
{
public:
    HungarianAlgorithm();
    ~HungarianAlgorithm();
    double Solve(vector <vector<double> > DistMatrix, vector<int>& Assignment);

private:
    void assignmentoptimal(int *assignment, double *cost, double *distMatrix, int nOfRows, int nOfColumns);
    void buildassignmentvector(int *assignment, bool *starMatrix, int nOfRows, int nOfColumns);
    void computeassignmentcost(int *assignment, double *cost, double *distMatrix, int nOfRows);
    void step2a(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step2b(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step3(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
    void step4(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim, int row, int col);
    void step5(int *assignment, double *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, int nOfRows, int nOfColumns, int minDim);
};


#include <vector>
#include <algorithm>
#include <limits>

class GreedyAssignmentAlgorithm {
public:
    double Solve(std::vector<std::vector<double>> DistMatrix, std::vector<int>& Assignment) {
        unsigned int nRows = DistMatrix.size();
        unsigned int nCols = DistMatrix[0].size();

        // 初始化分配结果和总成本
        Assignment = std::vector<int>(nRows, -1); // 初始化为-1，表示未分配
        double cost = 0.0;

        // 遍历每一行，寻找当前行的最小成本分配
        for (unsigned int j = 0; j < nCols; ++j) {
            
            double minCost = std::numeric_limits<double>::max();
            int min_i = -1;

            for(int i = 0; i < nRows; ++i) {
                if(DistMatrix[i][j] < minCost && Assignment[i] == -1) {
                    minCost = DistMatrix[i][j];
                    min_i = i;
                }
            }

            // 在未分配的行中寻找最小成本
            if(min_i != -1 && DistMatrix[min_i][j] < 1) {
                Assignment[min_i] = j;
                cost += minCost;
            }
        }

        return cost;
    }
};


double Solve(vector<vector<double>> DistMatrix, vector<int>& Assignment);


#endif
