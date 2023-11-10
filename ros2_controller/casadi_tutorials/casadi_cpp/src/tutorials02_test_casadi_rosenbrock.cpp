#include <iostream>
#include <casadi/casadi.hpp>

using namespace casadi;


// https://github.com/casadi/casadi/blob/main/docs/examples/cplusplus/rosenbrock.cpp
/**
Solve the Rosenbrock problem, formulated as the NLP:

minimize     x^2 + 100*z^2
subject to   z+(1-x)^2-y == 0

Joel Andersson, 2015-2016
*/

namespace casadi_cpp {
namespace {

void Run()
{
    // Declare variables
    SX x = SX::sym("x");
    SX y = SX::sym("y");
    SX z = SX::sym("z");

    // Formulate the NLP
    SX f = pow(x,2) + 100*pow(z,2);
    SX g = z + pow(1-x, 2) - y;
    SXDict nlp = {{"x", SX::vertcat({x,y,z})},
                    {"f", f},
                    {"g", g}};

    // Create an NLP solver
    Function solver = nlpsol("solver", "ipopt", nlp);

    // Solve the Rosenbrock problem
    DMDict arg;
    arg["x0"] = std::vector<double>{2.5,3.0,0.75};
    arg["lbg"] = arg["ubg"] = 0;
    DMDict res = solver(arg);

    //  Print solution
    std::cout << "Optimal cost:                     " << double(res.at("f")) << std::endl;
    std::cout << "Primal solution:                  " << std::vector<double>(res.at("x")) << std::endl;
    std::cout << "Dual solution (simple bounds):    " << std::vector<double>(res.at("lam_x")) << std::endl;
    std::cout << "Dual solution (nonlinear bounds): " << std::vector<double>(res.at("lam_g")) << std::endl;
}

}  // namespace
}  // namespace casadi_cpp



int main(int argc, char **argv)
{
    casadi_cpp::Run();
    return 0;
}