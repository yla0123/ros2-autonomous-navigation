#pragma once

#include <vector>
#include <utility>
#include <cstdint>

class AStarPlanner{
public:
    AStarPlanner(int width, int height);

    std::vector<std::pair<int, int>> findPath(const std::vector<int8_t>& grid, 
    const int start_row, const int start_col, const int goal_row, const int goal_col);

private:
    int width_;
    int height_;

    bool isValidCell(int row, int col) const;

    bool isTraversable(const std::vector<int8_t>& grid, int row, int col) const;

    double calDistance(int row, int col, int goal_row, int goal_col) const;

    std::vector<std::pair<int, int>> reconstructPath(
        const std::vector<std::pair<int, int>>& parents,
        int start_row,
        int start_col,
        int goal_row,
        int goal_col
    ) const;
};

struct AStarNode{
    int row;
    int col;

    double cost; // cost from start to curr
    double est_cost; // estimated cost from curr to goal
};

struct CompareAStarNode{
    bool operator()(const AStarNode& a, const AStarNode& b) const;
};