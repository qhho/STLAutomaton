/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/config.h>
#include <iostream>
#include <vector>

#include "src/STLAutomaton.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;

void generate_automaton()
{


    std::cout << "This is the automaton for the formula F[0,10]1" << std::endl;
    oc::AutomatonPtr eventually = oc::Automaton::STLtoAutomaton("F[0,10]1", 2);
    eventually->print(std::cout);
    std::cout << "--------------------------------------------------" << std::endl;


    std::cout << "This is the automaton for the formula G[0,10]1" << std::endl;
    oc::AutomatonPtr globally = oc::Automaton::STLtoAutomaton("G[0,10]1", 2);
    globally->print(std::cout);

    std::cout << "--------------------------------------------------" << std::endl;

    std::cout << "This is the automaton for the formula F[0,10]1&G[0, 10]2&F[0,10]3" << std::endl;
    oc::AutomatonPtr eventually_globally = oc::Automaton::STLtoAutomaton("F[0,10]1&G[0, 10]2&F[0,10]3", 2);
    eventually_globally->print(std::cout);

    std::cout << "--------------------------------------------------" << std::endl;

    std::cout << "Here is an example of syntactic separation into DNF formulas" << std::endl;

    std::cout << "We separate G[0, 6]2&F[0,18]1 at time {6}" << std::endl;

    unsigned int numberOfPredicates = 2; //define number of propositions
    std::vector<double > separationTimes = {6};
    //Seperation assumes that the formula timings are in sequence (this is a limitation of the current implementation)
    std::vector<std::string> disjunctFormulas = oc::Automaton::STLFormulaSeparation("G[0, 6]2&F[0,18]1", separationTimes);

    std::cout << "This gives us the following disjunct formulas, which each have the following automaton:" << std::endl;
    for (auto & disjunctFormula : disjunctFormulas)
    {
        std::cout << disjunctFormula << std::endl;
        oc::AutomatonPtr disjunctAutomaton = oc::Automaton::STLtoAutomaton(disjunctFormula, 2);
        disjunctAutomaton->print(std::cout);
        std::cout << std::endl;
    }
}

int main(int /*argc*/, char ** /*argv*/)
{
    generate_automaton();
    return 0;
}