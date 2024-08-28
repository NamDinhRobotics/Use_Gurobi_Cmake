#include "gurobi_c++.h"
#include <iostream>
#include <vector>
#include <cmath>

int main() {
    try {
        GRBEnv env = GRBEnv(true);
        env.start();

        int N = 10; // Prediction horizon
        double T = 0.1; // Time step
        double L = 1.5; // Wheelbase of the vehicle

        // Weights for the cost function
        std::vector<std::vector<double>> Q = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0.1, 0}, {0, 0, 0, 0.1}};
        std::vector<std::vector<double>> R = {{0.1, 0}, {0, 0.1}};
        std::vector<std::vector<double>> Q_f = {{10, 0, 0, 0}, {0, 10, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

        // Define initial state and target state
        std::vector<double> x_init = {0, 0, 0, 0};  // Initial state (x, y, theta, v)
        std::vector<double> x_target = {5, 5, 0, 0};  // Target state (x, y, theta, v)

        // Control limits
        double steer_max = M_PI / 4;
        double steer_min = -M_PI / 4;
        double a_max = 3.0;
        double a_min = -3.0;

        GRBModel model = GRBModel(env);

        // Define state and control variables
        std::vector<GRBVar> x_vars(N+1), y_vars(N+1), theta_vars(N+1), v_vars(N+1);
        std::vector<GRBVar> steer_vars(N), a_vars(N);
        std::vector<GRBVar> cos_theta_vars(N), sin_theta_vars(N), tan_steer_vars(N);

        // Create state variables
        for (int k = 0; k <= N; ++k) {
            x_vars[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "x_" + std::to_string(k));
            y_vars[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "y_" + std::to_string(k));
            theta_vars[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "theta_" + std::to_string(k));
            v_vars[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "v_" + std::to_string(k));
        }

        // Create control variables
        for (int k = 0; k < N; ++k) {
            steer_vars[k] = model.addVar(steer_min, steer_max, 0, GRB_CONTINUOUS, "steer_" + std::to_string(k));
            a_vars[k] = model.addVar(a_min, a_max, 0, GRB_CONTINUOUS, "a_" + std::to_string(k));

            // Create variables for cos(theta_k), sin(theta_k), and tan(steer_k)
            cos_theta_vars[k] = model.addVar(-1, 1, 0, GRB_CONTINUOUS, "cos_theta_" + std::to_string(k));
            sin_theta_vars[k] = model.addVar(-1, 1, 0, GRB_CONTINUOUS, "sin_theta_" + std::to_string(k));
            tan_steer_vars[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "tan_steer_" + std::to_string(k));
        }

        // Set initial state constraint
        model.addConstr(x_vars[0] == x_init[0]);
        model.addConstr(y_vars[0] == x_init[1]);
        model.addConstr(theta_vars[0] == x_init[2]);
        model.addConstr(v_vars[0] == x_init[3]);

        // Set up dynamics constraints, trigonometric constraints, and cost function
        GRBQuadExpr obj = 0;

        for (int k = 0; k < N; ++k) {
            // Add general constraints for cos(theta_k), sin(theta_k), and tan(steer_k)
            model.addGenConstrCos(theta_vars[k], cos_theta_vars[k], "cos_theta_" + std::to_string(k));
            model.addGenConstrSin(theta_vars[k], sin_theta_vars[k], "sin_theta_" + std::to_string(k));
            model.addGenConstrTan(steer_vars[k], tan_steer_vars[k], "tan_steer_" + std::to_string(k));

            // Dynamics constraints using sin, cos, and tan variables
            model.addQConstr(x_vars[k+1] == x_vars[k] + T * v_vars[k] * cos_theta_vars[k]);
            model.addQConstr(y_vars[k+1] == y_vars[k] + T * v_vars[k] * sin_theta_vars[k]);
            model.addQConstr(theta_vars[k+1] == theta_vars[k] + T * (v_vars[k] / L) * tan_steer_vars[k]);
            model.addConstr(v_vars[k+1] == v_vars[k] + T * a_vars[k]);

            // Quadratic cost function for states and controls
            obj += x_vars[k] * x_vars[k] * Q[0][0] + y_vars[k] * y_vars[k] * Q[1][1] +
                   theta_vars[k] * theta_vars[k] * Q[2][2] + v_vars[k] * v_vars[k] * Q[3][3] +
                   steer_vars[k] * steer_vars[k] * R[0][0] + a_vars[k] * a_vars[k] * R[1][1];
        }

        // Terminal cost
        obj += (x_vars[N] - x_target[0]) * (x_vars[N] - x_target[0]) * Q_f[0][0] +
               (y_vars[N] - x_target[1]) * (y_vars[N] - x_target[1]) * Q_f[1][1] +
               (theta_vars[N] - x_target[2]) * (theta_vars[N] - x_target[2]) * Q_f[2][2] +
               (v_vars[N] - x_target[3]) * (v_vars[N] - x_target[3]) * Q_f[3][3];

        model.setObjective(obj, GRB_MINIMIZE);

        // Optimize the model
        model.optimize();

        // Output the results
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            std::cout << "Optimal solution found!" << std::endl;
            for (int k = 0; k <= N; ++k) {
                std::cout << "State at step " << k << ": ("
                          << x_vars[k].get(GRB_DoubleAttr_X) << ", "
                          << y_vars[k].get(GRB_DoubleAttr_X) << ", "
                          << theta_vars[k].get(GRB_DoubleAttr_X) << ", "
                          << v_vars[k].get(GRB_DoubleAttr_X) << ")" << std::endl;
            }
        } else {
            std::cout << "No optimal solution found." << std::endl;
        }
    } catch (GRBException& e) {
        std::cerr << "Error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
    } catch (...) {
        std::cerr << "Exception during optimization." << std::endl;
    }
    return 0;
}
