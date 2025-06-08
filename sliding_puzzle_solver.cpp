#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <functional>


/*

Version: 1.5

This is an insanely fast sliding puzzle solver for 3x3, 4x4, and 5x5 puzzles written in C++.

After trying to find a fast sliding puzzle solver in Python, the LLM suggest to try a faster language like C++. And the difference is indeed huge. While 3x3 matrices can be solved with Python in about a second, it will try ages to solve a 4x4 matrix.

Testing with the input "14 4 13 5 3 12 9 6 1 15 7 2 11 10 8 0" the previous Python implementation was eventually aborted after 10 minutes with about 30 million states explored. The C++ implementation solves the same puzzle in 0.01 seconds with only 2133 states explored. Also, the solution only takes 66 moves, while https://slidingpuzzlesolver.org suggests a solution of 176 moves. So this script is not only incredibly fast, it also finds a much shorter solution than the other solver.


Usage:

./sliding_puzzle                                Run without any arguments for default behavior.
./sliding_puzzle --shortest                     Find the shortest solution for 4x4 and 5x5 puzzles (3x3 will always find the shortest solution). Timeouts after 90 seconds.
./sliding_puzzle --shortest --timeout 120       Same as above, but with a custom timeout.
./sliding_puzzle --weights 1.5                  Use a custom heuristic weight for the A* algorithm. Must be a positive floating-point number. Can't be used together with --shortest.



Compile this code using GCC or Clang:
g++ gemini_sliding_puzzle_solver.cpp -o sliding_puzzle -std=c++17 -O2

ARM64/Apple Silicon (M1, M2 etc.) users can use the following flags for better performance:
g++ gemini_sliding_puzzle_solver.cpp -o sliding_puzzle -std=c++17 -O3 -march=native -mtune=native -flto -DNDEBUG -ffast-math -funroll-loops -fomit-frame-pointer
clang++ gemini_sliding_puzzle_solver.cpp -o sliding_puzzle -std=c++17 -O3 -march=native -mtune=native -flto -DNDEBUG -ffast-math -funroll-loops

*/

// Forward declaration
class SlidingPuzzleSolver;

// Custom hasher for std::vector<int> to be used in unordered_map/unordered_set
struct VectorHasher {
    std::size_t operator()(const std::vector<int>& vec) const {
        std::size_t seed = vec.size();
        for (int i : vec) {
            // A common way to combine hash values.
            seed ^= std::hash<int>()(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

// Node structure for A* priority queue
struct AStarNode {
    double f_score;
    long long tie_breaker; // To ensure stable sort for elements with same f_score
    int g_score;
    std::vector<int> state;
    std::vector<std::string> path;

    // Custom comparator for min-priority queue
    bool operator>(const AStarNode& other) const {
        if (f_score != other.f_score) {
            return f_score > other.f_score;
        }
        return tie_breaker > other.tie_breaker; // Smaller tie_breaker has higher priority
    }
};

class SlidingPuzzleSolver {
private:
    struct TargetPosCache {
        int size = 0;
        std::unordered_map<int, std::pair<int, int>> map;
    };
    TargetPosCache _target_pos_map_cache;
    long long _astar_tie_breaker_counter = 0;


    int _get_size(const std::vector<int>& state_tuple) const {
        int length = state_tuple.size();
        if (length == 0) return 0;
        int size = static_cast<int>(std::sqrt(length));
        if (size * size != length) {
            throw std::runtime_error("Invalid puzzle state length. Must be a perfect square.");
        }
        return size;
    }

    std::pair<int, int> _get_blank_pos(const std::vector<int>& state_tuple, int size) const {
        auto it = std::find(state_tuple.begin(), state_tuple.end(), 0);
        if (it == state_tuple.end()) {
            throw std::runtime_error("Blank tile (0) not found in puzzle state.");
        }
        int idx = std::distance(state_tuple.begin(), it);
        return {idx / size, idx % size}; // row, col
    }

    bool _is_solvable(const std::vector<int>& initial_state_tuple) const {
        int size = _get_size(initial_state_tuple);
        if (size == 0) return false;

        std::vector<int> state_list_no_blank;
        for (int tile : initial_state_tuple) {
            if (tile != 0) {
                state_list_no_blank.push_back(tile);
            }
        }

        int inversions = 0;
        for (size_t i = 0; i < state_list_no_blank.size(); ++i) {
            for (size_t j = i + 1; j < state_list_no_blank.size(); ++j) {
                if (state_list_no_blank[i] > state_list_no_blank[j]) {
                    inversions++;
                }
            }
        }

        if (size % 2 == 1) { // Odd grid (e.g., 3x3)
            return inversions % 2 == 0;
        } else { // Even grid (e.g., 4x4)
            std::pair<int, int> blank_pos = _get_blank_pos(initial_state_tuple, size);
            int blank_row_from_top = blank_pos.first;
            int blank_row_from_bottom_0_indexed = (size - 1) - blank_row_from_top;
            return (inversions + blank_row_from_bottom_0_indexed) % 2 == 0;
        }
    }

    std::vector<std::pair<std::vector<int>, std::string>> _get_neighbors(const std::vector<int>& current_state_tuple) const {
        int size = _get_size(current_state_tuple);
        std::pair<int, int> blank_pos = _get_blank_pos(current_state_tuple, size);
        int blank_r = blank_pos.first;
        int blank_c = blank_pos.second;
        
        std::vector<std::pair<std::vector<int>, std::string>> neighbors;

        int dr[] = {-1, 1, 0, 0}; // Relative row changes for Up, Down
        int dc[] = {0, 0, -1, 1}; // Relative col changes for Left, Right
        std::string tile_move_dirs[] = {"Down", "Up", "Right", "Left"}; // How the TILE moves to fill the blank

        for (int i = 0; i < 4; ++i) {
            int tile_r = blank_r + dr[i]; // This is the position of the TILE that will move
            int tile_c = blank_c + dc[i];

            if (tile_r >= 0 && tile_r < size && tile_c >= 0 && tile_c < size) {
                std::vector<int> new_state_list = current_state_tuple;
                int tile_to_move_val = new_state_list[tile_r * size + tile_c];
                
                // Swap blank (0) with the tile
                new_state_list[blank_r * size + blank_c] = tile_to_move_val; // Blank spot gets tile value
                new_state_list[tile_r * size + tile_c] = 0;                   // Tile spot becomes blank
                
                std::string move_description = "Move tile " + std::to_string(tile_to_move_val) + " " + tile_move_dirs[i];
                neighbors.push_back({new_state_list, move_description});
            }
        }
        return neighbors;
    }

    std::vector<int> _get_target_state(int size) const {
        if (size == 0) return {};
        std::vector<int> target(size * size);
        for (int i = 0; i < size * size - 1; ++i) {
            target[i] = i + 1;
        }
        target[size * size - 1] = 0; // Blank at the end
        return target;
    }

    double _manhattan_distance(const std::vector<int>& state_tuple, const std::vector<int>& target_state_tuple, int size) {
        if (size == 0) return 0.0;

        if (_target_pos_map_cache.size != size) {
            _target_pos_map_cache.map.clear();
            _target_pos_map_cache.size = size;
            for (int r = 0; r < size; ++r) {
                for (int c = 0; c < size; ++c) {
                    int tile_val = target_state_tuple[r * size + c];
                    if (tile_val != 0) {
                        _target_pos_map_cache.map[tile_val] = {r, c};
                    }
                }
            }
        }
        
        double distance = 0;
        for (int r_curr = 0; r_curr < size; ++r_curr) {
            for (int c_curr = 0; c_curr < size; ++c_curr) {
                int tile_val = state_tuple[r_curr * size + c_curr];
                if (tile_val != 0) {
                    auto it = _target_pos_map_cache.map.find(tile_val);
                    if (it != _target_pos_map_cache.map.end()) {
                        std::pair<int, int> target_pos = it->second;
                        distance += std::abs(r_curr - target_pos.first) + std::abs(c_curr - target_pos.second);
                    }
                }
            }
        }
        return distance;
    }

    std::vector<std::string> _solve_bfs(const std::vector<int>& initial_state_tuple, const std::vector<int>& target_state_tuple, int size) {
        std::queue<std::pair<std::vector<int>, std::vector<std::string>>> q;
        std::set<std::vector<int>> visited; 

        q.push({initial_state_tuple, {}});
        visited.insert(initial_state_tuple);
        
        long long nodes_explored = 0;
        auto start_time_bfs = std::chrono::steady_clock::now();

        while (!q.empty()) {
            nodes_explored++;
            if (nodes_explored % 20000 == 0) {
                auto current_time_bfs = std::chrono::steady_clock::now();
                double elapsed_bfs = std::chrono::duration<double>(current_time_bfs - start_time_bfs).count();
                std::cout << "BFS: States explored: " << nodes_explored 
                          << ", Queue size: " << q.size() 
                          << ", Time: " << std::fixed << std::setprecision(2) << elapsed_bfs << "s\n";
            }

            std::pair<std::vector<int>, std::vector<std::string>> current_item = q.front();
            q.pop();
            std::vector<int> current_state = current_item.first;
            std::vector<std::string> current_path = current_item.second;

            if (current_state == target_state_tuple) {
                auto end_time_bfs = std::chrono::steady_clock::now();
                double elapsed_bfs = std::chrono::duration<double>(end_time_bfs - start_time_bfs).count();
                std::cout << "\nBFS Solution Found! Explored " << nodes_explored 
                          << " states. Time: " << std::fixed << std::setprecision(2) << elapsed_bfs << "s (" << std::fixed << std::setprecision(5) << elapsed_bfs << "s)\n";
                return current_path;
            }

            for (const auto& neighbor_pair : _get_neighbors(current_state)) {
                const std::vector<int>& next_state = neighbor_pair.first;
                if (visited.find(next_state) == visited.end()) {
                    visited.insert(next_state);
                    std::vector<std::string> new_path = current_path;
                    new_path.push_back(neighbor_pair.second);
                    q.push({next_state, new_path});
                }
            }
        }
        auto end_time_bfs = std::chrono::steady_clock::now();
        double elapsed_bfs = std::chrono::duration<double>(end_time_bfs - start_time_bfs).count();
        std::cout << "BFS: No solution found. Explored " << nodes_explored << " states. Time: " << std::fixed << std::setprecision(2) << elapsed_bfs << "s (" << std::fixed << std::setprecision(5) << elapsed_bfs << "s)\n";
        return {}; // Return empty path for no solution
    }

    std::vector<std::string> _solve_astar(const std::vector<int>& initial_state_tuple, const std::vector<int>& target_state_tuple, int size, double weight) {
        _astar_tie_breaker_counter = 0; // Reset for each call
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
        std::unordered_map<std::vector<int>, int, VectorHasher> g_scores;

        double initial_h_score = _manhattan_distance(initial_state_tuple, target_state_tuple, size);
        double initial_f_score = 0 + weight * initial_h_score; // g_score is 0

        open_set.push({initial_f_score, _astar_tie_breaker_counter++, 0, initial_state_tuple, {}});
        g_scores[initial_state_tuple] = 0;

        long long nodes_popped = 0;
        auto start_time_astar = std::chrono::steady_clock::now();
        auto last_progress_print_time = start_time_astar;
        auto last_matrix_print_time = start_time_astar;
        
        std::cout << "A* Started with Heuristic Weight: " << std::fixed << std::setprecision(1) << weight << std::endl;

        while (!open_set.empty()) {
            nodes_popped++;
            auto current_time_astar = std::chrono::steady_clock::now();

            if (nodes_popped % 10000 == 0 || std::chrono::duration<double>(current_time_astar - last_progress_print_time).count() > 10.0) {
                if (!open_set.empty()) {
                    const AStarNode& top_node = open_set.top();
                    double h_raw = (weight > 0 && std::abs(weight) > 1e-9) ? (top_node.f_score - top_node.g_score) / weight : (top_node.f_score - top_node.g_score);
                    double elapsed_astar = std::chrono::duration<double>(current_time_astar - start_time_astar).count();
                    std::cout << "A*: Nodes popped: " << nodes_popped
                              << ", Open set: " << open_set.size()
                              << ", Min f: " << std::fixed << std::setprecision(2) << top_node.f_score
                              << " (g:" << top_node.g_score << ", h_raw:" << h_raw << ")"
                              << ", Time: " << elapsed_astar << "s\n";
                }
                last_progress_print_time = current_time_astar;
            }

            AStarNode current_node = open_set.top();
            open_set.pop();
            
            double current_f_popped = current_node.f_score; // Store f_score before path modification
            int current_g_popped = current_node.g_score;

            if (std::chrono::duration<double>(current_time_astar - last_matrix_print_time).count() > 10.0 && nodes_popped > 1) { // Don't print for the very first pop
                double elapsed_astar_mat = std::chrono::duration<double>(current_time_astar - start_time_astar).count();
                 std::ostringstream msg_stream;
                 msg_stream << "A* Current state (after " << nodes_popped << " pops, "
                           << std::fixed << std::setprecision(1) << elapsed_astar_mat << "s, f="
                           << std::fixed << std::setprecision(2) << current_f_popped << ", g=" << current_g_popped << "):";
                print_puzzle(current_node.state, msg_stream.str());
                last_matrix_print_time = current_time_astar;
            }
            
            auto g_it = g_scores.find(current_node.state);
             // If state not in g_scores or current_node.g_score is higher, it means this path is not optimal or state already processed better.
            if (g_it == g_scores.end() || current_node.g_score > g_it->second) {
                continue; 
            }


            if (current_node.state == target_state_tuple) {
                auto end_time_astar = std::chrono::steady_clock::now();
                double elapsed_astar = std::chrono::duration<double>(end_time_astar - start_time_astar).count();
                std::string solution_type = (std::abs(weight - 1.0) < 1e-9) ? "Optimal A*" : ("Weighted A* (W=" + std::to_string(weight) + ")");
                std::cout << "\n" << solution_type << " Solution Found! Popped " << nodes_popped 
                          << " states. Time: " << std::fixed << std::setprecision(2) << elapsed_astar << "s (" << std::fixed << std::setprecision(5) << elapsed_astar << "s)\n";
                return current_node.path;
            }

            for (const auto& neighbor_pair : _get_neighbors(current_node.state)) {
                const std::vector<int>& next_state = neighbor_pair.first;
                const std::string& move_description = neighbor_pair.second;
                int tentative_g_score = current_node.g_score + 1;

                auto g_score_it = g_scores.find(next_state);
                if (g_score_it == g_scores.end() || tentative_g_score < g_score_it->second) {
                    g_scores[next_state] = tentative_g_score; // Update or insert g_score
                    double h_score = _manhattan_distance(next_state, target_state_tuple, size);
                    double f_score = tentative_g_score + weight * h_score;
                    
                    std::vector<std::string> new_path = current_node.path;
                    new_path.push_back(move_description);
                    open_set.push({f_score, _astar_tie_breaker_counter++, tentative_g_score, next_state, new_path});
                }
            }
        }
        auto end_time_astar = std::chrono::steady_clock::now();
        double elapsed_astar = std::chrono::duration<double>(end_time_astar - start_time_astar).count();
        std::cout << "A*: No solution found. Popped " << nodes_popped << " states. Time: " << std::fixed << std::setprecision(2) << elapsed_astar << "s (" << std::fixed << std::setprecision(5) << elapsed_astar << "s) (W=" << weight << ")\n";
        return {}; // Return empty path
    }

    std::vector<std::string> _solve_astar_with_timeout(const std::vector<int>& initial_state_tuple, const std::vector<int>& target_state_tuple, int size, double weight, int timeout_seconds) {
        _astar_tie_breaker_counter = 0;
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
        std::unordered_map<std::vector<int>, int, VectorHasher> g_scores;

        double initial_h_score = _manhattan_distance(initial_state_tuple, target_state_tuple, size);
        double initial_f_score = 0 + weight * initial_h_score;

        open_set.push({initial_f_score, _astar_tie_breaker_counter++, 0, initial_state_tuple, {}});
        g_scores[initial_state_tuple] = 0;

        long long nodes_popped = 0;
        auto start_time_astar = std::chrono::steady_clock::now();
        auto last_progress_print_time = start_time_astar;
        auto last_matrix_print_time = start_time_astar;
        
        std::cout << "A* Started with Heuristic Weight: " << std::fixed << std::setprecision(1) << weight << std::endl;

        while (!open_set.empty()) {
            nodes_popped++;
            auto current_time_astar = std::chrono::steady_clock::now();
            
            // Check timeout
            double elapsed = std::chrono::duration<double>(current_time_astar - start_time_astar).count();
            if (elapsed > timeout_seconds) {
                std::cout << "A*: Timeout reached (" << timeout_seconds << "s). Stopping search at weight " << weight << std::endl;
                return {}; // Return empty to indicate timeout
            }

            if (nodes_popped % 10000 == 0 || std::chrono::duration<double>(current_time_astar - last_progress_print_time).count() > 10.0) {
                if (!open_set.empty()) {
                    const AStarNode& top_node = open_set.top();
                    double h_raw = (weight > 0 && std::abs(weight) > 1e-9) ? (top_node.f_score - top_node.g_score) / weight : (top_node.f_score - top_node.g_score);
                    std::cout << "A*: Nodes popped: " << nodes_popped
                              << ", Open set: " << open_set.size()
                              << ", Min f: " << std::fixed << std::setprecision(2) << top_node.f_score
                              << " (g:" << top_node.g_score << ", h_raw:" << h_raw << ")"
                              << ", Time: " << elapsed << "s\n";
                }
                last_progress_print_time = current_time_astar;
            }

            AStarNode current_node = open_set.top();
            open_set.pop();
            
            double current_f_popped = current_node.f_score;
            int current_g_popped = current_node.g_score;

            if (std::chrono::duration<double>(current_time_astar - last_matrix_print_time).count() > 10.0 && nodes_popped > 1) {
                std::ostringstream msg_stream;
                msg_stream << "A* Current state (after " << nodes_popped << " pops, "
                          << std::fixed << std::setprecision(1) << elapsed << "s, f="
                          << std::fixed << std::setprecision(2) << current_f_popped << ", g=" << current_g_popped << "):";
                print_puzzle(current_node.state, msg_stream.str());
                last_matrix_print_time = current_time_astar;
            }
            
            auto g_it = g_scores.find(current_node.state);
            if (g_it == g_scores.end() || current_node.g_score > g_it->second) {
                continue; 
            }

            if (current_node.state == target_state_tuple) {
                auto end_time_astar = std::chrono::steady_clock::now();
                double elapsed_final = std::chrono::duration<double>(end_time_astar - start_time_astar).count();
                std::string solution_type = (std::abs(weight - 1.0) < 1e-9) ? "Optimal A*" : ("Weighted A* (W=" + std::to_string(weight) + ")");
                std::cout << "\n" << solution_type << " Solution Found! Popped " << nodes_popped 
                          << " states. Time: " << std::fixed << std::setprecision(2) << elapsed_final << "s (" << std::fixed << std::setprecision(5) << elapsed_final << "s)\n";
                return current_node.path;
            }

            for (const auto& neighbor_pair : _get_neighbors(current_node.state)) {
                const std::vector<int>& next_state = neighbor_pair.first;
                const std::string& move_description = neighbor_pair.second;
                int tentative_g_score = current_node.g_score + 1;

                auto g_score_it = g_scores.find(next_state);
                if (g_score_it == g_scores.end() || tentative_g_score < g_score_it->second) {
                    g_scores[next_state] = tentative_g_score;
                    double h_score = _manhattan_distance(next_state, target_state_tuple, size);
                    double f_score = tentative_g_score + weight * h_score;
                    
                    std::vector<std::string> new_path = current_node.path;
                    new_path.push_back(move_description);
                    open_set.push({f_score, _astar_tie_breaker_counter++, tentative_g_score, next_state, new_path});
                }
            }
        }
        auto end_time_astar = std::chrono::steady_clock::now();
        double elapsed_astar = std::chrono::duration<double>(end_time_astar - start_time_astar).count();
        std::cout << "A*: No solution found. Popped " << nodes_popped << " states. Time: " << std::fixed << std::setprecision(2) << elapsed_astar << "s (" << std::fixed << std::setprecision(5) << elapsed_astar << "s) (W=" << weight << ")\n";
        return {};
    }

public:
    SlidingPuzzleSolver() = default;

    void print_puzzle(const std::vector<int>& state_tuple, const std::string& message = "") const {
        int size;
        try {
            size = _get_size(state_tuple);
        } catch (const std::runtime_error& e) {
            std::cerr << message << " Invalid state for printing: " << e.what() << std::endl;
            return;
        }

        if (!message.empty()) {
            std::cout << message << std::endl;
        }
        if (size == 0) {
            std::cout << "Cannot print puzzle of size 0." << std::endl;
            return;
        }
            
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                int val = state_tuple[i * size + j];
                std::cout << std::setw(3) << (val == 0 ? "_" : std::to_string(val));
            }
            std::cout << std::endl;
        }
        // Adjust line width calculation
        int line_width = size * 3 + (size > 1 ? (size - 1) * 1 : 0) ; // 3 per tile, 1 for space between tiles
        if (size == 1) line_width = 3; // special case for 1x1
        if (size == 0) line_width = 0;

        for(int k = 0; k < line_width ; ++k) std::cout << "-";
        std::cout << std::endl;
    }

    std::vector<std::string> solve(const std::vector<int>& initial_state_list, bool find_shortest = false, int timeout_seconds = 90, double custom_weight = -1.0) {
        std::vector<int> initial_state_tuple = initial_state_list; 
        int size;
        try {
            size = _get_size(initial_state_tuple);
        } catch (const std::runtime_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return {};
        }
        
        if (size == 0) {
             std::cerr << "Error: Puzzle size cannot be zero (e.g. empty input)." << std::endl;
             return {};
        }

        // Validate tiles are 0 to N*N-1 exactly once
        std::vector<bool> present(size * size, false);
        if (initial_state_tuple.size() != (size_t)(size * size)) {
            std::cerr << "Error: Input tile count (" << initial_state_tuple.size() 
                      << ") does not match expected for " << size << "x" << size 
                      << " puzzle (" << size*size << ")." << std::endl;
            return {};
        }
        for (int tile : initial_state_tuple) {
            if (tile < 0 || tile >= size * size) {
                std::cerr << "Error: Tile " << tile << " is out of range for a " 
                          << size << "x" << size << " puzzle." << std::endl;
                return {};
            }
            if (present[tile]) {
                std::cerr << "Error: Tile " << tile << " is duplicated." << std::endl;
                return {};
            }
            present[tile] = true;
        }

        if (!_is_solvable(initial_state_tuple)) {
            std::cout << "This puzzle configuration is not solvable." << std::endl;
            print_puzzle(initial_state_tuple, "Initial (Unsolvable) State:");
            return {};
        }

        std::vector<int> target_state_tuple = _get_target_state(size);

        if (initial_state_tuple == target_state_tuple) {
            std::cout << "Puzzle is already solved!" << std::endl;
            return {};
        }

        print_puzzle(initial_state_tuple, "Initial State:");
        
        // Determine heuristic weight - use custom weight if provided, otherwise use defaults
        double weight_to_use;
        if (custom_weight > 0.0) {
            weight_to_use = custom_weight;
            std::cout << "Using custom heuristic weight: " << std::fixed << std::setprecision(1) << weight_to_use << std::endl;
        } else {
            // Use default weights based on size
            if (size == 3) {
                weight_to_use = 1.0;
            } else if (size == 4) {
                weight_to_use = 2.0;
            } else if (size == 5) {
                weight_to_use = 3.0;
            } else {
                std::cerr << "Solver currently supports 3x3, 4x4 and 5x5 puzzles. Size " 
                          << size << "x" << size << " is not supported." << std::endl;
                return {};
            }
        }

        // Reset cache if size changed
        if (_target_pos_map_cache.size != size) {
            _target_pos_map_cache.map.clear();
            _target_pos_map_cache.size = 0; 
        }

        // Solve with the determined weight
        std::vector<std::string> best_solution;
        if (custom_weight > 0.0) {
            // Custom weight mode
            std::string weight_description = (std::abs(weight_to_use - 1.0) < 1e-9) ? 
                "(guarantees shortest path)" : 
                "(custom weight, may not be shortest)";
            std::cout << "Solving a " << size << "x" << size << " puzzle using A* (W=" 
                      << std::fixed << std::setprecision(1) << weight_to_use << ") " 
                      << weight_description << "..." << std::endl;
            best_solution = _solve_astar(initial_state_tuple, target_state_tuple, size, weight_to_use);
        } else {
            // Default mode
            if (size == 3) {
                std::cout << "Solving a 3x3 puzzle using A* (W=" << std::fixed << std::setprecision(1) << weight_to_use 
                          << ") (guarantees shortest path)..." << std::endl;
                best_solution = _solve_astar(initial_state_tuple, target_state_tuple, size, weight_to_use);
            } else if (size == 4) {
                std::cout << "Solving a 4x4 puzzle using Weighted A* (W=" << std::fixed << std::setprecision(1) << weight_to_use 
                          << ") (aims for faster solution, may not be shortest)..." << std::endl;
                best_solution = _solve_astar(initial_state_tuple, target_state_tuple, size, weight_to_use);
            } else if (size == 5) {
                std::cout << "Solving a 5x5 puzzle using Weighted A* (W=" << std::fixed << std::setprecision(1) << weight_to_use 
                          << ") (aims for faster solution, may not be shortest)..." << std::endl;
                best_solution = _solve_astar(initial_state_tuple, target_state_tuple, size, weight_to_use);
            }
        }

        if (best_solution.empty()) {
            return {}; // No solution found
        }

        size_t best_moves = best_solution.size();
        std::cout << "\nSolution found with " << best_moves << " moves using weight " << std::fixed << std::setprecision(1) << weight_to_use << std::endl;

        // Print solution steps if using --shortest
        if (find_shortest) {
            std::cout << "\nSolution path (" << best_solution.size() << " moves):\n";
            for (size_t i = 0; i < best_solution.size(); ++i) {
                std::cout << (i + 1) << ". " << best_solution[i] << std::endl;
            }
        }

        // If --shortest flag is set and we're not already at weight 1.0, try to find shorter solutions
        if (find_shortest && weight_to_use > 1.0) {
            std::cout << "\n=== SEARCHING FOR SHORTER SOLUTIONS ===\n";
            std::cout << "Using timeout of " << timeout_seconds << " seconds per attempt.\n";
            
            double current_weight = weight_to_use - 0.1;
            double best_weight = weight_to_use;  // Track the weight that produced the best solution
            
            while (current_weight >= 1.0) {
                std::cout << "\nTrying weight " << std::fixed << std::setprecision(1) << current_weight << "..." << std::endl;
                
                std::vector<std::string> candidate_solution = _solve_astar_with_timeout(
                    initial_state_tuple, target_state_tuple, size, current_weight, timeout_seconds);
                
                if (candidate_solution.empty()) {
                    // Either no solution found or timeout reached
                    std::cout << "Stopping search due to timeout or no solution found." << std::endl;
                    break;
                }
                
                if (candidate_solution.size() < best_moves) {
                    best_solution = candidate_solution;
                    best_moves = candidate_solution.size();
                    best_weight = current_weight;  // Update the weight that gave us the best solution
                    std::cout << "*** BETTER SOLUTION FOUND! ***" << std::endl;
                    std::cout << "New best: " << best_moves << " moves with weight " << std::fixed << std::setprecision(1) << current_weight << std::endl;
                    
                    // Print the better solution steps
                    std::cout << "\nImproved solution path (" << candidate_solution.size() << " moves):\n";
                    for (size_t i = 0; i < candidate_solution.size(); ++i) {
                        std::cout << (i + 1) << ". " << candidate_solution[i] << std::endl;
                    }
                } else {
                    std::cout << "Solution found with " << candidate_solution.size() << " moves (not better than current best of " << best_moves << ")" << std::endl;
                }
                
                // Check if we've reached weight 1.0 (with small tolerance for floating point)
                if (current_weight <= 1.0 + 1e-9) {
                    std::cout << "Reached optimal weight 1.0. Search complete." << std::endl;
                    break;
                }
                
                current_weight -= 0.1;
                // Ensure we don't go below 1.0 due to floating point precision
                if (current_weight < 1.0) {
                    current_weight = 1.0;
                }
            }
            
            std::cout << "\n=== SHORTEST SOLUTION SEARCH COMPLETE ===\n";
            std::cout << "Final best solution: " << best_moves << " moves (found with weight " << std::fixed << std::setprecision(1) << best_weight << ")" << std::endl;
            
            // Repeat the best solution at the end
            std::cout << "\n=== BEST SOLUTION SUMMARY ===\n";
            std::cout << "Shortest path found (" << best_solution.size() << " moves, weight " << std::fixed << std::setprecision(1) << best_weight << "):\n";
            for (size_t i = 0; i < best_solution.size(); ++i) {
                std::cout << (i + 1) << ". " << best_solution[i] << std::endl;
            }
        }

        return best_solution;
    }
};

int main(int argc, char* argv[]) {
    // Check for --shortest flag, --timeout parameter, and --weights parameter
    bool find_shortest = false;
    int timeout_seconds = 90; // Default timeout
    double custom_weight = -1.0; // -1.0 indicates no custom weight specified
    
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--shortest") {
            find_shortest = true;
        } else if (std::string(argv[i]) == "--timeout" && i + 1 < argc) {
            try {
                timeout_seconds = std::stoi(argv[i + 1]);
                if (timeout_seconds <= 0) {
                    std::cerr << "Error: Timeout must be a positive integer." << std::endl;
                    return 1;
                }
                i++; // Skip the next argument since we used it as the timeout value
            } catch (const std::exception& e) {
                std::cerr << "Error: Invalid timeout value '" << argv[i + 1] << "'. Must be a positive integer." << std::endl;
                return 1;
            }
        } else if (std::string(argv[i]) == "--weights" && i + 1 < argc) {
            try {
                custom_weight = std::stod(argv[i + 1]);
                if (custom_weight <= 0.0) {
                    std::cerr << "Error: Weight must be a positive number." << std::endl;
                    return 1;
                }
                i++; // Skip the next argument since we used it as the weight value
            } catch (const std::exception& e) {
                std::cerr << "Error: Invalid weight value '" << argv[i + 1] << "'. Must be a positive number." << std::endl;
                return 1;
            }
        }
    }

    // Check for conflicting parameters
    if (find_shortest && custom_weight > 0.0) {
        std::cerr << "Error: Cannot use --shortest and --weights together. Please choose one." << std::endl;
        return 1;
    }

    SlidingPuzzleSolver solver;
    
    std::cout << "Enter puzzle tiles separated by spaces (e.g., 1 2 3 4 0 5 6 7 8 for 3x3, 16 tiles for 4x4, or 25 tiles for 5x5): \n";
    std::string line;
    std::getline(std::cin, line);
    
    std::vector<int> initial_puzzle_list;
    std::stringstream ss(line);
    int tile;
    while (ss >> tile) {
        initial_puzzle_list.push_back(tile);
    }

    if (initial_puzzle_list.empty()) {
        std::cerr << "Error: No puzzle input provided." << std::endl;
        return 1;
    }

    try {
        std::vector<std::string> solution_path = solver.solve(initial_puzzle_list, find_shortest, timeout_seconds, custom_weight);

        if (!solution_path.empty() && !find_shortest) { 
            // Only print final solution if not using --shortest (since --shortest prints solutions as they're found)
            std::cout << "\nFinal solution path (" << solution_path.size() << " moves):\n";
            for (size_t i = 0; i < solution_path.size(); ++i) {
                std::cout << (i + 1) << ". " << solution_path[i] << std::endl;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "An unexpected error occurred during solve: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}