#pragma once

#include <vector>
#include <utility>
#include <cstdint>

class AStarPlanner{
public:
    AStarPlanner(int width, int height);

    std::vector<std::pair<int, int>> findPath(const std::vector<int8_t>& grid, const   std::pair<int,int>& start_cell, const std::pair<int,int>& goal_cell);

private:
    int width_;
    int height_;

    bool isValidCell(int row, int col) const;

    bool isTraversable(const std::vector<int8_t>& grid, int row, int col) const;

    double traversalCost(const std::vector<int8_t>& grid, int row, int col) const;

    double calDistance(int row, int col, int goal_row, int goal_col) const;

    std::vector<std::pair<int, int>> reconstructPath(
        const std::vector<std::pair<int, int>>& parents,
        const std::pair<int,int>& start_cell, 
        const std::pair<int,int>& goal_cell
    ) const;
};

struct AStarNode{
    int row;
    int col;

    double g_cost; // cost from start to current cell
    double h_cost; // estimated cost from current cell to goal
};

struct CompareAStarNode{
    bool operator()(const AStarNode& a, const AStarNode& b) const;
};