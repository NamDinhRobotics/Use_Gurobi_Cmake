#include "gurobi_c++.h"

int main() {
    try {
        // Create environment
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "mip_qcp.log");
        env.start();

        // Create an empty model
        GRBModel model = GRBModel(env);

        // Create variables
        GRBVar x = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "x");
        GRBVar y = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y");
        GRBVar z = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "z");

        // Set objective: x^2 + y^2 + z
        GRBQuadExpr objective = x*x + y*y + z;
        model.setObjective(objective, GRB_MINIMIZE);

        // Add linear constraint: x + y + z >= 1
        model.addConstr(x + y + z >= 1, "c1");

        // Add quadratic constraint: x^2 + y^2 <= z
        GRBQuadExpr qc = x*x + y*y;
        model.addQConstr(qc <= z, "qc1");

        // Optimize the model
        model.optimize();

        // Print results
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            std::cout << "Optimal objective: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
            std::cout << "x = " << x.get(GRB_DoubleAttr_X) << std::endl;
            std::cout << "y = " << y.get(GRB_DoubleAttr_X) << std::endl;
            std::cout << "z = " << z.get(GRB_DoubleAttr_X) << std::endl;
        } else {
            std::cout << "No optimal solution found" << std::endl;
        }

    } catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch (...) {
        std::cout << "Exception during optimization" << std::endl;
    }

    return 0;
}
