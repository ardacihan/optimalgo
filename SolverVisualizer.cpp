#include "SolverVisualizer.h"
#include <iostream>

SolverVisualizer::SolverVisualizer() 
    : solver(std::make_unique<GeometryBasedNeighborhoodSolver>()) {}

std::vector<RectanglePlacement> SolverVisualizer::solve_with_visualization(
    RectangleFittingProblem& problem, int max_steps) {
    
    clear_history();
    
    auto current_solution = problem.get_current_solution();
    int current_obj = problem.objective(current_solution);
    
    // Store initial state
    search_steps.push_back(current_solution);
    objective_values.push_back(current_obj);
    step_descriptions.push_back("Initial solution");
    
    std::cout << "Starting visualization solver with " << max_steps << " steps" << std::endl;
    
    for (int step = 0; step < max_steps; ++step) {
        std::cout << "Step " << step + 1 << "/" << max_steps << std::endl;
        
        // Generate neighbors
        auto neighbors = solver->construct_neighbors(problem);
        std::cout << "Generated " << neighbors.size() << " neighbors" << std::endl;
        
        if (neighbors.empty()) {
            std::cout << "No neighbors generated, stopping early." << std::endl;
            break;
        }
        
        // Evaluate all neighbors
        int best_neighbor_obj = current_obj;
        std::vector<RectanglePlacement> best_neighbor = current_solution;
        int best_neighbor_index = -1;
        
        for (size_t i = 0; i < neighbors.size(); ++i) {
            int neighbor_obj = problem.objective(neighbors[i]);
            
            // Store each neighbor for visualization
            search_steps.push_back(neighbors[i]);
            objective_values.push_back(neighbor_obj);
            step_descriptions.push_back("Neighbor " + std::to_string(i + 1) + 
                                       " (Δ=" + std::to_string(neighbor_obj - current_obj) + ")");
            
            if (neighbor_obj > best_neighbor_obj) {
                best_neighbor_obj = neighbor_obj;
                best_neighbor = neighbors[i];
                best_neighbor_index = i;
            }
        }
        
        // Check if we found improvement
        if (best_neighbor_obj > current_obj) {
            // Store the selected best neighbor
            search_steps.push_back(best_neighbor);
            objective_values.push_back(best_neighbor_obj);
            step_descriptions.push_back("SELECTED: Best neighbor " + 
                                       std::to_string(best_neighbor_index + 1) + 
                                       " (Gain: +" + 
                                       std::to_string(best_neighbor_obj - current_obj) + ")");
            
            // Update current solution
            current_solution = best_neighbor;
            current_obj = best_neighbor_obj;
            problem.set_current_solution(current_solution);
            
            std::cout << "Improved to objective: " << current_obj 
                      << " (gain: +" << (best_neighbor_obj - current_obj) << ")" << std::endl;
        } else {
            std::cout << "No improvement found. Best neighbor: " << best_neighbor_obj 
                      << " (current: " << current_obj << ")" << std::endl;
            
            // Store current state to show no improvement
            search_steps.push_back(current_solution);
            objective_values.push_back(current_obj);
            step_descriptions.push_back("No improvement - keeping current solution");
            
            break; // No improvement, stop
        }
    }
    
    std::cout << "Final objective: " << current_obj << std::endl;
    return current_solution;
}

void SolverVisualizer::clear_history() {
    search_steps.clear();
    objective_values.clear();
    step_descriptions.clear();
}