# sliding-puzzle-solver
An insanely fast sliding puzzle solver for 3x3, 4x4, and 5x5 puzzles written in C++.

### Usage

```
./sliding_puzzle                                Run without any arguments for default behavior.
./sliding_puzzle --shortest                     Find the shortest solution for 4x4 and 5x5 puzzles (3x3 will always find the shortest solution). Timeouts after 90 seconds.
./sliding_puzzle --shortest --timeout 120       Same as above, but with a custom timeout.
./sliding_puzzle --weights 1.5                  Use a custom heuristic weight for the A* algorithm. Must be a positive floating-point number. Can't be used together with --shortest.
```


### Compilation

Comple the C++ using the following command:

```
g++ sliding_puzzle_solver.cpp -o sliding_puzzle -std=c++17 -O2
```

While the code is already very fast for the default weights, lower weights can take much longer to find a solution. ARM64/Apple Silicon (M1, M2 etc.) users may try to use Clang with the following flags for a bit better performance:

```
clang++ gemini_sliding_puzzle_solver.cpp -o sliding_puzzle -std=c++17 -O3 -march=native -mtune=native -flto -DNDEBUG -ffast-math -funroll-loops
```
