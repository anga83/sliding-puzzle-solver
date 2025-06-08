# Sliding Puzzle Solver
An insanely fast sliding puzzle solver for 3x3, 4x4, and 5x5 puzzles written in C++.

![[Screenshot]](screenshot.jpg)


With the default settings a solution is usually found within 0.1 seconds or even faster. Use lower weights to find a shorter solution path (less moves) at the cost of longer calculation time.

### Usage

```
./sliding_puzzle                                Run without any arguments for default behavior.
./sliding_puzzle --shortest                     Find the shortest solution for 4x4 and 5x5 puzzles (3x3 will always find the shortest solution). Timeouts after 90 seconds.
./sliding_puzzle --shortest --timeout 120       Same as above, but with a custom timeout.
./sliding_puzzle --weights 1.5                  Use a custom heuristic weight for the A* algorithm. Must be a positive floating-point number. Can't be used together with --shortest.
```

In case you need a sliding puzzle to test the code, you might want to check out the daily [Bing Image Puzzle](https://www.bing.com/spotlight/imagepuzzle).


### Compilation

Comple the C++ using the following command:

```
g++ sliding_puzzle_solver.cpp -o sliding_puzzle -std=c++17 -O2
```

(Experimental) While the code is already very fast for the default weights, lower weights can take much longer to find a solution. ARM64/Apple Silicon (M1, M2 etc.) users may try to use Clang with the following flags for a bit better performance:

```
clang++ gemini_sliding_puzzle_solver.cpp -o sliding_puzzle -std=c++17 -O3 -march=native -mtune=native -flto -DNDEBUG -ffast-math -funroll-loops
```
