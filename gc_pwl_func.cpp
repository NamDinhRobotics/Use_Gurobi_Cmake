//
// Created by dinhnambkhn on 28/08/2024.
//

#include "gurobi_c++.h"
#include <cmath>
using namespace std;
static double f(double u) { return exp(u); }
static double g(double u) { return sqrt(u); }

static void
printsol(GRBModel& m, GRBVar& x, GRBVar& y, GRBVar& u, GRBVar& v)
{
    cout << "x = " << x.get(GRB_DoubleAttr_X) << ", u = " << u.get(GRB_DoubleAttr_X) << endl;
    cout << "y = " << y.get(GRB_DoubleAttr_X) << ", v = " << v.get(GRB_DoubleAttr_X) << endl;
    cout << "Obj = " << m.get(GRB_DoubleAttr_ObjVal) << endl;

    // Calculate violation of exp(x) + 4 sqrt(y) <= 9
    double vio = f(x.get(GRB_DoubleAttr_X)) + 4 * g(y.get(GRB_DoubleAttr_X)) - 9;
    if (vio < 0.0) vio = 0.0;
    cout << "Vio = " << vio << endl;
}

int
main(int argc, char* argv[])
{
    try {

        // Create environment

        GRBEnv env = GRBEnv();

        // Create a new model

        GRBModel m = GRBModel(env);

        // Create variables

        double lb = 0.0, ub = GRB_INFINITY;

        GRBVar x = m.addVar(lb, ub, 0.0, GRB_CONTINUOUS, "x");
        GRBVar y = m.addVar(lb, ub, 0.0, GRB_CONTINUOUS, "y");
        GRBVar u = m.addVar(lb, ub, 0.0, GRB_CONTINUOUS, "u");
        GRBVar v = m.addVar(lb, ub, 0.0, GRB_CONTINUOUS, "v");

        // Set objective

        m.setObjective(2*x + y, GRB_MAXIMIZE);

        // Add linear constraint

        m.addConstr(u + 4*v <= 9, "l1");

        m.addGenConstrExp(x, u, "gcf1");
        m.addGenConstrPow(y, v, 0.5, "gcf2");

        // Use the equal piece length approach with the length = 1e-3

        m.set(GRB_IntParam_FuncPieces, 1);
        m.set(GRB_DoubleParam_FuncPieceLength, 1e-3);

        // Optimize the model and print solution

        m.optimize();
        printsol(m, x, y, u, v);

        // Zoom in, use optimal solution to reduce the ranges and use a smaller
        // pclen=1e-5 to solve it

        double xval = x.get(GRB_DoubleAttr_X);
        double yval = y.get(GRB_DoubleAttr_X);

        x.set(GRB_DoubleAttr_LB, max(x.get(GRB_DoubleAttr_LB), xval-0.01));
        x.set(GRB_DoubleAttr_UB, min(x.get(GRB_DoubleAttr_UB), xval+0.01));
        y.set(GRB_DoubleAttr_LB, max(y.get(GRB_DoubleAttr_LB), yval-0.01));
        y.set(GRB_DoubleAttr_UB, min(y.get(GRB_DoubleAttr_UB), yval+0.01));
        m.update();
        m.reset();

        m.set(GRB_DoubleParam_FuncPieceLength, 1e-5);

        // Optimize the model and print solution

        m.optimize();
        printsol(m, x, y, u, v);

    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }

}