#include "Benchmark.h"
#include "solver/local_search/GeometryBasedNeighborhoodSolver.h"
#include "solver/local_search/RuleBasedNeighborhoodSolver.h"
#include "solver/local_search/RelaxedGeometryBasedNeighborhoodSolver.h"
#include "solver/greedy/GreedySolver.h"

#include <iostream>
#include <string>

void printUsage() {
    std::cout << "Enhanced Test Environment for Rectangle Packing Algorithms\n";
    std::cout << "===========================================================\n";
    std::cout << "Usage:\n";
    std::cout << "  ./enhanced_test quick     - Run quick test (2-5 minutes)\n";
    std::cout << "  ./enhanced_test full      - Run comprehensive test (30-60 minutes)\n";
    std::cout << "  ./enhanced_test params    - Run parameter sensitivity analysis\n";
    std::cout << "  ./enhanced_test custom    - Run with custom test cases\n";
    std::cout << "\nOutput files:\n";
    std::cout << "  - enhanced_test_results.csv: Detailed results for all runs\n";
    std::cout << "  - enhanced_test_summary.txt: Comprehensive analysis\n";
    std::cout << "  - detailed_analysis.csv: Parameter sensitivity data\n";
    std::cout << "\n";
}

int main4(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage();
        return 1;
    }
    
    std::string mode = argv[1];
    
    try {
        EnhancedTestEnvironment env;
        
        // Register all solvers
        std::cout << "Registering solvers...\n";
        env.addSolver("GeometryBased", std::make_unique<GeometryBasedNeighborhoodSolver>());
        env.addSolver("RuleBased", std::make_unique<RuleBasedNeighborhoodSolver>());
        env.addSolver("RelaxedRuleBased", std::make_unique<RelaxedGeometryBasedNeighborhoodSolver>());
        env.addSolver("FirstFitGreedy", std::make_unique<GreedySolver>());
        env.addSolver("BestFitGreedy", std::make_unique<GreedySolver>());
        env.addSolver("WorstFitGreedy", std::make_unique<GreedySolver>());
        std::cout << "Registered 6 solvers\n\n";
        
        if (mode == "quick") {
            std::cout << "Setting up QUICK test environment...\n";
            env.setupQuickTest();
            env.runTests("quick_enhanced_results.csv");
            
        } else if (mode == "full") {
            std::cout << "Setting up COMPREHENSIVE test environment...\n";
            env.setupComprehensiveTest();
            env.runTests("full_enhanced_results.csv");
            
        } else if (mode == "params") {
            std::cout << "Setting up PARAMETER SENSITIVITY analysis...\n";
            env.setupParameterAnalysis();
            env.runTests("param_analysis_results.csv");
            
        } else if (mode == "custom") {
            // Custom test cases
            std::cout << "Setting up CUSTOM test environment...\n";
            
            EnhancedTestEnvironment customEnv;
            
            // Add your custom test cases here
            customEnv.addTestCase(EnhancedTestCase(3, 100, 2, 2, 20, 20, 25, "Custom1", 100, 3, 20));
            customEnv.addTestCase(EnhancedTestCase(2, 200, 3, 3, 30, 30, 40, "Custom2", 150, 4, 30));
            customEnv.addTestCase(EnhancedTestCase(1, 500, 5, 5, 40, 40, 60, "Custom3", 200, 5, 50));
            
            // Register solvers to custom environment

            
            customEnv.runTests("custom_enhanced_results.csv");
            
        } else {
            std::cerr << "Unknown mode: " << mode << "\n";
            printUsage();
            return 1;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}