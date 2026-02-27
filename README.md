# Rectangle Packing Solver

An interactive C++ application for solving the **2D rectangle bin-packing problem** — fitting a set of rectangles into as few fixed-size square bins as possible. The project includes multiple solver strategies, a real-time GUI visualizer built with ImGui/OpenGL, and a benchmarking framework.

---

## Features

- **Multiple Solver Strategies**
    - *Greedy* — Biggest First, Smallest First, Best Fit
    - *Local Search* — Geometry-Based Neighborhood, Rule-Based Neighborhood, Relaxed Geometry-Based (Simulated Annealing)
- **Interactive GUI** — real-time step-by-step visualization of any solver
- **Benchmarking** — lightweight and heavy benchmark modes with XML result export
- **Solution Persistence** — save/load packing solutions as XML files
- **UML Documentation** — Doxygen configuration included for generating class diagrams and call graphs



## Getting Started

### Prerequisites

- CMake ≥ 3.15
- C++20-compatible compiler (GCC, Clang)
- GLFW3 (`sudo apt install libglfw3-dev` on Ubuntu)
- OpenGL
- X11, pthread, dl (standard on Linux)

ImGui is fetched automatically via CMake's `FetchContent`.

### Build

```bash
git clone https://github.com/your-username/optimalgo.git
cd optimalgo
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Run

```bash
./optimalgo
```

---

## Usage

### GUI

The visualizer opens a 1920×1080 window with two control panels:

**Problem Configuration (left panel)**
- Set rectangle count, box size, and min/max dimensions
- Generate a new random problem instance
- Choose a solver type and strategy
- Step through the solver one move at a time, or run it fully
- Revert to the original unsolved instance

**Visualization & Results (right panel)**
- Switch between Single Box, All Boxes (Horizontal), or All Boxes (Fit Screen) views
- View utilization statistics
- Run a benchmark and export results to XML
- Load a previously saved solution from XML

### Benchmarks

From the GUI, click **Run Benchmark**. Two modes are available:

| Mode | Rectangles | Description |
|------|-----------|-------------|
| Fast | 1,000 | Quick comparison across all solvers |
| Heavy | 5,000–10,000 | Four configurations, larger instances |

Results are written to `benchmark_results.txt` / `benchmark_results_big.txt`, and individual solutions are saved as `solution_*.xml`.

---

## Project Structure

```
src/
├── problem/
│   ├── OptimizationProblem.h              # Abstract base template
│   ├── RectangleFittingProblem.{h,cpp}    # Problem definition & objective function
│   └── primitives/
│       ├── RectanglePlacement.{h,cpp}     # Rectangle data structure
│       └── InstanceGenerator.{h,cpp}      # Random & heuristic instance generation
├── solver/
│   ├── RectangleFittingProblemSolver.{h,cpp}  # Solver base class
│   ├── greedy/
│   │   └── GreedySolver.{h,cpp}           # Greedy strategies
│   └── local_search/
│       ├── LocalSearchSolver.h            # Local search base with subproblem decomposition
│       └── specialized_solvers/
│           ├── GeometryBasedNeighborhoodSolver.{h,cpp}
│           ├── RuleBasedNeighborhoodSolver.{h,cpp}
│           └── RelaxedGeometryBasedNeighborhoodSolver.{h,cpp}
├── gui/
│   ├── visualizer.{h,cpp}                 # ImGui/OpenGL visualizer
│   ├── main_vis.cpp                       # Entry point
│   ├── benchmark_loader.{h,cpp}           # XML solution loader
├── Benchmark.{h,cpp}                      # Benchmark runner
CMakeLists.txt
Doxyfile
```

---

## Solver Overview

### Greedy Solvers

Place rectangles one at a time (with optional rotation) using an occupancy grid:

- **Biggest First** — largest area rectangle placed first
- **Smallest First** — smallest area rectangle placed first
- **Best Fit** — scores placements by adjacency and corner proximity

### Local Search Solvers

All local search variants inherit from `LocalSearchSolver`, which handles subproblem decomposition, parallel batch solving, and solution merging.

- **GeometryBasedNeighborhoodSolver** — generates neighbors by moving rectangles to valid grid positions in other bins, using a bottom-left greedy placement on implicit coordinate splits
- **RuleBasedNeighborhoodSolver** — generates neighbors via random index swaps and re-applies greedy placement
- **RelaxedGeometryBasedNeighborhoodSolver** — extends the geometry solver with simulated annealing; allows temporary overlaps proportional to the current temperature

### Objective Function

The objective rewards:
- High bin utilization (squared, to penalize imbalance)
- Edge-touching / adjacency between rectangles and bin walls
- Penalizes the number of bins used (hard penalty)
- Penalizes overlaps with a temperature-scaled weight (zero when `T=0`)

---

## Generating Documentation (Doxygen + UML)

A `Doxyfile` is included. To generate HTML docs with UML class and call graphs:

```bash
# Install Doxygen and Graphviz
sudo apt install doxygen graphviz

# Run from the project root
doxygen Doxyfile
```

Output will be in `html/`. Open `html/index.html` in a browser.

The configuration enables:
- UML-style class diagrams (`UML_LOOK = YES`)
- Collaboration, include, call, and caller graphs
- Interactive SVGs

---

## Configuration Reference

Key parameters available in the GUI and benchmark code:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rect_count` | 1000 | Number of rectangles |
| `box_size` | 80 | Side length of each square bin |
| `min/max_width` | 5–20 | Rectangle width range |
| `min/max_height` | 5–10 | Rectangle height range |
| `T` (temperature) | 1000 | Initial temperature for simulated annealing |
| `num_reruns` | 3 | Local search reruns per solve call |
| `max_rectangle_in_subproblem` | 100 | Max rectangles per parallel batch |

---

## Dependencies

| Library | Purpose | How obtained |
|---------|---------|--------------|
| [ImGui](https://github.com/ocornut/imgui) v1.90.8 | GUI rendering | FetchContent (auto) |
| GLFW3 | Window & input | System package |
| OpenGL | Rendering backend | System |

---