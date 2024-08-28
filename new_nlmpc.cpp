#include "gurobi_c++.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>

struct Obstacle {
    double x, y, radius;
};

int main() {
    try {
        GRBEnv env = GRBEnv(true);
        env.start();

        // Gurobi parameters
        env.set("MIPFocus", "1");
        env.set("MIPGap", "0.01");
        env.set("TimeLimit", "600");
        env.set("Threads", "0");
        env.set("PreSolve", "2");
        env.set("Cuts", "2");

        const int N = 20; // Prediction horizon
        double T = 0.1; // Time step
        double L = 2.0; // Wheelbase of the vehicle

        // Weights for the cost function
        double Q[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 0.1, 0}, {0, 0, 0, 0.1}};
        double R[2][2] = {{0.1, 0}, {0, 0.1}};
        double Q_f[4][4] = {{10, 0, 0, 0}, {0, 10, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

        // Define start and goal states
        double x_start[4] = {0, 0, M_PI_4, 0};  // Start state (x, y, theta, v)
        double x_goal[4] = {5, 5, M_PI_4, 0};  // Goal state (x, y, theta, v)

        // Control limits
        double steer_max = M_PI / 4;
        double steer_min = -M_PI / 4;
        double a_max = 2.0;
        double a_min = -2.0;

        // Define obstacles
        std::vector<Obstacle> obstacles = {
                {2, 2, 1}
        };

        GRBModel model = GRBModel(env);

        // Define state and control variables
        GRBVar x_vars[N+1], y_vars[N+1], theta_vars[N+1], v_vars[N+1];
        GRBVar steer_vars[N], a_vars[N];
        GRBVar cos_theta_vars[N], sin_theta_vars[N], tan_steer_vars[N];

        // Create state and control variables
        for (int k = 0; k <= N; ++k) {
            x_vars[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "x_" + std::to_string(k));
            y_vars[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "y_" + std::to_string(k));
            theta_vars[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "theta_" + std::to_string(k));
            v_vars[k] = model.addVar(0, 10, 0, GRB_CONTINUOUS, "v_" + std::to_string(k));
        }

        for (int k = 0; k < N; ++k) {
            steer_vars[k] = model.addVar(steer_min, steer_max, 0, GRB_CONTINUOUS, "steer_" + std::to_string(k));
            a_vars[k] = model.addVar(a_min, a_max, 0, GRB_CONTINUOUS, "a_" + std::to_string(k));
            cos_theta_vars[k] = model.addVar(-1, 1, 0, GRB_CONTINUOUS, "cos_theta_" + std::to_string(k));
            sin_theta_vars[k] = model.addVar(-1, 1, 0, GRB_CONTINUOUS, "sin_theta_" + std::to_string(k));
            tan_steer_vars[k] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0, GRB_CONTINUOUS, "tan_steer_" + std::to_string(k));
        }

        model.update();

        // Set initial state constraint
        model.addConstr(x_vars[0] == x_start[0]);
        model.addConstr(y_vars[0] == x_start[1]);
        model.addConstr(theta_vars[0] == x_start[2]);
        model.addConstr(v_vars[0] == x_start[3]);

        // Set up dynamics constraints and cost function
        GRBQuadExpr obj = 0;

        for (int k = 0; k < N; ++k) {
            // Trigonometric constraints using Gurobi's built-in functions
            model.addGenConstrCos(theta_vars[k], cos_theta_vars[k], "cos_theta_" + std::to_string(k));
            model.addGenConstrSin(theta_vars[k], sin_theta_vars[k], "sin_theta_" + std::to_string(k));
            model.addGenConstrTan(steer_vars[k], tan_steer_vars[k], "tan_steer_" + std::to_string(k));

            // Dynamics constraints
            model.addQConstr(x_vars[k+1] == x_vars[k] + T * v_vars[k] * cos_theta_vars[k]);
            model.addQConstr(y_vars[k+1] == y_vars[k] + T * v_vars[k] * sin_theta_vars[k]);
            model.addQConstr(theta_vars[k+1] == theta_vars[k] + T * (v_vars[k] / L) * tan_steer_vars[k]);
            model.addConstr(v_vars[k+1] == v_vars[k] + T * a_vars[k]);

            // Obstacle avoidance constraints
            for (const auto& obstacle : obstacles) {
                model.addQConstr((x_vars[k] - obstacle.x) * (x_vars[k] - obstacle.x) +
                                 (y_vars[k] - obstacle.y) * (y_vars[k] - obstacle.y) >=
                                 (obstacle.radius + 1) * (obstacle.radius + 1));
            }

            // Cost function for states and controls
            obj += (x_vars[k] - x_goal[0]) * (x_vars[k] - x_goal[0]) * Q[0][0] +
                   (y_vars[k] - x_goal[1]) * (y_vars[k] - x_goal[1]) * Q[1][1] +
                   theta_vars[k] * theta_vars[k] * Q[2][2] +
                   v_vars[k] * v_vars[k] * Q[3][3] +
                   steer_vars[k] * steer_vars[k] * R[0][0] +
                   a_vars[k] * a_vars[k] * R[1][1];
        }

        // Terminal cost
        obj += (x_vars[N] - x_goal[0]) * (x_vars[N] - x_goal[0]) * Q_f[0][0] +
               (y_vars[N] - x_goal[1]) * (y_vars[N] - x_goal[1]) * Q_f[1][1] +
               (theta_vars[N] - x_goal[2]) * (theta_vars[N] - x_goal[2]) * Q_f[2][2] +
               (v_vars[N] - x_goal[3]) * (v_vars[N] - x_goal[3]) * Q_f[3][3];

        model.setObjective(obj, GRB_MINIMIZE);

        // Set NonConvex parameter for non-convex quadratic optimization
        // model.set(GRB_IntParam_NonConvex, 2);
        model.set(GRB_IntParam_FuncNonlinear, 1);

        auto start = std::chrono::high_resolution_clock::now();

        // Optimize the model
        model.optimize();

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        std::cout << "Optimization time: " << elapsed.count() << " seconds" << std::endl;

        // Output the results
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            std::cout << "Optimal path found!" << std::endl;
            for (int k = 0; k <= N; ++k) {
                std::cout << "State at step " << k << ": ("
                          << x_vars[k].get(GRB_DoubleAttr_X) << ", "
                          << y_vars[k].get(GRB_DoubleAttr_X) << ", "
                          << theta_vars[k].get(GRB_DoubleAttr_X) << ", "
                          << v_vars[k].get(GRB_DoubleAttr_X) << ")" << std::endl;
            }
            //control
            for (int k = 0; k < N; ++k) {
                std::cout << "Control at step " << k << ": ("
                          << steer_vars[k].get(GRB_DoubleAttr_X) << ", "
                          << a_vars[k].get(GRB_DoubleAttr_X) << ")" << std::endl;
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