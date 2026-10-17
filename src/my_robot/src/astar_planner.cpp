#include "my_robot/astar_planner.hpp"
#include <cmath>
#include <queue>
#include <limits>
#include <algorithm>

AStarPlanner::AStarPlanner(int width, int height)
    :width_(width), height_(height){
        // initialize width_ and height_
    }

std::vector<std::pair<int, int>> AStarPlanner::findPath(const std::vector<int8_t>& grid, 
    const std::pair<int,int>& start_cell, const std::pair<int,int>& goal_cell){
        std::vector<std::pair<int,int>> path;

        const int start_row = start_cell.first;
        const int start_col = start_cell.second;

        const int goal_row = goal_cell.first;
        const int goal_col = goal_cell.second;

        // Check if start and goal are valid
        if (!isTraversable(grid, start_row, start_col)|| 
            !isTraversable(grid, goal_row, goal_col)) return path;

        std::priority_queue<AStarNode, std::vector<AStarNode>, CompareAStarNode> open_set;
        // Create a priority queue that stores AStarNodes, internally uses a vector<AStarNode>, and orders them using CompareAStarNode.

        std::vector<double> cost_map(width_ * height_, std::numeric_limits<double>::infinity());
        std::vector<std::pair<int,int>> parents(width_ * height_, {-1, -1});
        
        int start_id = start_row * width_ + start_col;
        cost_map[start_id] = 0.0;
        
        // Add start to open_set
        AStarNode start;
        start.row = start_row;
        start.col = start_col;
        start.g_cost = 0;
        start.h_cost = calDistance(start_row, start_col, goal_row, goal_col);
        open_set.push(start);

        // 4 directions
        const int drow[4] = {-1, 1, 0, 0};
        const int dcol[4] = {0, 0, -1, 1};

        while(!open_set.empty()){
            AStarNode current = open_set.top();
            open_set.pop();

            if(current.row == goal_row && current.col == goal_col){
                return reconstructPath(parents, start_cell, goal_cell);
            }

            //check 4 directions
            for(int i = 0; i < 4; ++i){
                int next_row = current.row + drow[i];
                int next_col = current.col + dcol[i];

                if(!isTraversable(grid, next_row, next_col)){
                    continue;
                }
                double next_cost = current.g_cost + 1.0;
                int next_id = next_row * width_ + next_col;

                if(next_cost < cost_map[next_id]){
                    // a better route to 'next' is found
                    cost_map[next_id] = next_cost;
                    parents[next_id] = {current.row, current.col}; // record previous cell

                    // add to open set
                    AStarNode next;
                    next.row = next_row;
                    next.col = next_col;
                    next.g_cost = next_cost;
                    next.h_cost = calDistance(next_row, next_col, goal_row, goal_col);
                    open_set.push(next);
                }
            }
        }

        return path;
    }


std::vector<std::pair<int, int>> AStarPlanner::reconstructPath(
    const std::vector<std::pair<int, int>>& parents,
    const std::pair<int,int>& start_cell, const std::pair<int,int>& goal_cell)const{

        const int start_row = start_cell.first;
        const int start_col = start_cell.second;

        const int goal_row = goal_cell.first;
        const int goal_col = goal_cell.second;

        std::vector<std::pair<int,int>> path;
        std::pair<int, int> current = std::pair<int, int>{goal_row, goal_col};

        while (current != std::pair<int,int>{start_row, start_col}){
            path.push_back(current);
            int id = current.first * width_ + current.second;
            current = parents[id];
        }
        path.push_back(current);
        std::reverse(path.begin(), path.end());
        return path;
}

bool AStarPlanner::isValidCell(int row, int col) const{
    if(row < 0 || row >= height_) return false;
    if(col < 0 || col >= width_ ) return false;

    return true;
}

bool AStarPlanner::isTraversable(const std::vector<int8_t>& grid, int row, int col) const{
    if(!isValidCell(row, col)) return false;

    int id = row * width_ + col;

    return (grid[id] == 0);
}

double AStarPlanner::calDistance(int row, int col, int goal_row, int goal_col) const{
    return std::abs(row - goal_row) + std::abs(col - goal_col);
}

bool CompareAStarNode::operator()(const AStarNode& a, const AStarNode& b) const{
    return (a.g_cost+a.h_cost > b.g_cost+b.h_cost);
    // return true if a cost more than b
}

