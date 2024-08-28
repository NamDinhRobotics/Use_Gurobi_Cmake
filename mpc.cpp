#include "gurobi_c++.h"
#include <iostream>
#include <vector>

int main() {
    try {
        // Create a Gurobi environment
        GRBEnv env = GRBEnv(true);
        env.start();

        // Define the problem parameters
        int n = 2; // Number of state variables
        int m = 1; // Number of control inputs
        int N = 20; // Prediction horizon

        // Define system matrices
        std::vector<std::vector<double>> A = {{1, 1}, {0, 1}};
        std::vector<std::vector<double>> B = {{0.5}, {1}};
        std::vector<std::vector<double>> Q = {{1, 0}, {0, 1}};
        std::vector<std::vector<double>> R = {{1}};
        std::vector<std::vector<double>> Q_terminal = {{10, 0}, {0, 10}}; // Penalize deviation at the final state

        // Define the initial state A and target state B
        std::vector<double> x0 = {0, 0};  // Initial state A
        std::vector<double> x_target = {10, 0};  // Target state B

        // State and input constraints
        std::vector<double> x_min = {-10, -10};
        std::vector<double> x_max = {10, 10};
        std::vector<double> u_min = {-1};
        std::vector<double> u_max = {1};

        // Create an empty model
        GRBModel model = GRBModel(env);

        // Define variables
        std::vector<GRBVar> x_vars(N * n);
        std::vector<GRBVar> u_vars((N - 1) * m);

        // Add state variables x_k
        for (int k = 0; k < N; ++k) {
            for (int i = 0; i < n; ++i) {
                x_vars[k * n + i] = model.addVar(x_min[i], x_max[i], 0, GRB_CONTINUOUS, "x_" + std::to_string(k) + "_" + std::to_string(i));
            }
        }

        // Add control variables u_k
        for (int k = 0; k < N - 1; ++k) {
            for (int j = 0; j < m; ++j) {
                u_vars[k * m + j] = model.addVar(u_min[j], u_max[j], 0, GRB_CONTINUOUS, "u_" + std::to_string(k) + "_" + std::to_string(j));
            }
        }

        // Set quadratic objective function
        GRBQuadExpr obj = 0;

        // Quadratic cost for states: x_k^T Q x_k
        for (int k = 0; k < N; ++k) {
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    obj += x_vars[k * n + i] * Q[i][j] * x_vars[k * n + j];
                }
            }
        }

        // Quadratic cost for controls: u_k^T R u_k
        for (int k = 0; k < N - 1; ++k) {
            for (int i = 0; i < m; ++i) {
                for (int j = 0; j < m; ++j) {
                    obj += u_vars[k * m + i] * R[i][j] * u_vars[k * m + j];
                }
            }
        }

        // Terminal cost: penalize deviation from target state x_target
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                obj += (x_vars[(N-1) * n + i] - x_target[i]) * Q_terminal[i][j] * (x_vars[(N-1) * n + j] - x_target[j]);
            }
        }

        model.setObjective(obj, GRB_MINIMIZE);

        // Add system dynamics constraints
        for (int k = 0; k < N - 1; ++k) {
            for (int i = 0; i < n; ++i) {
                GRBLinExpr lhs = 0;
                for (int j = 0; j < n; ++j) {
                    lhs += A[i][j] * x_vars[k * n + j];
                }
                for (int j = 0; j < m; ++j) {
                    lhs += B[i][j] * u_vars[k * m + j];
                }
                model.addConstr(x_vars[(k + 1) * n + i] == lhs, "dyn_" + std::to_string(k) + "_" + std::to_string(i));
            }
        }

        // Set initial state constraint: x_0 = A
        for (int i = 0; i < n; ++i) {
            model.addConstr(x_vars[i] == x0[i], "initial_" + std::to_string(i));
        }

        // Optimize the model
        model.optimize();

        // Output the results
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            std::cout << "Optimal solution found!" << std::endl;
            for (int k = 0; k < N; ++k) {
                std::cout << "x_" << k << " = ";
                for (int i = 0; i < n; ++i) {
                    std::cout << x_vars[k * n + i].get(GRB_DoubleAttr_X) << " ";
                }
                std::cout << std::endl;
            }
            for (int k = 0; k < N - 1; ++k) {
                std::cout << "u_" << k << " = ";
                for (int j = 0; j < m; ++j) {
                    std::cout << u_vars[k * m + j].get(GRB_DoubleAttr_X) << " ";
                }
                std::cout << std::endl;
            }
            std::cout << "Objective value: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
        } else {
            std::cout << "No optimal solution found." << std::endl;
        }
    } catch (GRBException& e) {
        std::cerr << "Error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
    } catch (...) {
        std::cerr << "Exception during optimization" << std::endl;
    }

    return 0;
}
