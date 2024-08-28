#include "gurobi_c++.h"
#include <iostream>

int main() {
    try {
        // Create a Gurobi environment
        GRBEnv env = GRBEnv(true);
        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);

        // Define the number of variables
        int n = 3; // Example size
        std::vector<GRBVar> vars(n);

        // Define the quadratic objective function
        // Q matrix (symmetric)
        std::vector<std::vector<double>> Q = { {1.0, 0.5, 0.0},
                                               {0.5, 1.0, 0.0},
                                               {0.0, 0.0, 1.0} };

        // c vector
        std::vector<double> c = {1.0, 1.0, 1.0};

        // Create variables
        for (int i = 0; i < n; ++i) {
            vars[i] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "x" + std::to_string(i));
        }

        // Set objective function: 0.5 * x^T Q x + c^T x
        GRBQuadExpr obj = 0;
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j <= i; ++j) {
                obj += 0.5 * Q[i][j] * vars[i] * vars[j];
            }
            obj += c[i] * vars[i];
        }
        model.setObjective(obj, GRB_MINIMIZE);

        // Define constraints
        int m = 2; // Example number of constraints
        std::vector<std::vector<double>> A = { {1.0, 1.0, 0.0},
                                               {0.0, 1.0, 1.0} };
        std::vector<double> b = {1.0, 1.0};

        for (int i = 0; i < m; ++i) {
            GRBLinExpr constraint_expr = 0;
            for (int j = 0; j < n; ++j) {
                constraint_expr += A[i][j] * vars[j];
            }
            model.addConstr(constraint_expr <= b[i], "c" + std::to_string(i));
        }

        // Solve the model
        model.optimize();

        // Output the results
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            std::cout << "Optimal solution found!" << std::endl;
            for (int i = 0; i < n; ++i) {
                std::cout << "x_" << i << " = " << vars[i].get(GRB_DoubleAttr_X) << std::endl;
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
