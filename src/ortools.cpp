#include "../include/ortools.h"

namespace operations_research {
 auto LinearProgrammingExample() -> void {

    MPSolver solver("linear_programming_examples",
        MPSolver::CPLEX_LINEAR_PROGRAMMING);

    const double infinity = solver.infinity();

    MPVariable* const x = 
      solver.MakeNumVar(0.0, infinity, "x");
    MPVariable* const y = 
      solver.MakeNumVar(0.0, infinity, "y");

    LOG(INFO) << "Number of variables = " << solver.NumVariables();

    MPConstraint* const c0 = solver.MakeRowConstraint(-infinity, 14.0);
    c0->SetCoefficient(x, 1);
    c0->SetCoefficient(y, 2);

    // 3*x - y >= 0.
    MPConstraint* const c1 = solver.MakeRowConstraint(0.0, infinity);
    c1->SetCoefficient(x, 3);
    c1->SetCoefficient(y, -1);

    // x - y <= 2.
    MPConstraint* const c2 = solver.MakeRowConstraint(-infinity, 2.0);
    c2->SetCoefficient(x, 1);
    c2->SetCoefficient(y, -1);
    LOG(INFO) << "Number of constraints = " << solver.NumConstraints();
    // [END constraints]

    // [START objective]
    MPObjective* const objective = solver.MutableObjective();
    objective->SetCoefficient(x, 3);
    objective->SetCoefficient(y, 4);
    objective->SetMaximization();

    const MPSolver::ResultStatus result_status = solver.Solve();
    if (result_status != MPSolver::OPTIMAL) {
      LOG(FATAL) << "The problem does not have an optimal solution!";
    }

    LOG(INFO) << "Solution:";
    LOG(INFO) << "Optimal objective value = " << objective->Value();
    LOG(INFO) << x->name() << " = " << x->solution_value();
    LOG(INFO) << y->name() << " = " << y->solution_value();
}
}
