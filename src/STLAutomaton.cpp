#include "STLAutomaton.hpp"
#include "ompl/control/planners/ltl/World.h"
#include <boost/range/irange.hpp>
#include <unordered_map>
#include <unordered_set>
#include <boost/dynamic_bitset.hpp>
#include <ostream>
#include <limits>
#include <queue>
#include <vector>
#include <iostream>

std::vector<int > ompl::control::Automaton::TransitionMap::eval(const World &w) const
{
    std::vector< int> states;
    const auto d = entries.find(w);
    if (d != entries.end())
        return d->second;
    for (const auto &entry : entries)
    {
        if (w.satisfies(entry.first))
        {
            // Since w satisfies another world that leads to d->second,
            // we can add an edge directly from w to d->second.
            entries[w] = entry.second;
            return entry.second;
        }
    }
    states.push_back(-1);
    return states;
}

ompl::control::Automaton::Automaton(unsigned int numProps, unsigned int numStates)
  : numProps_(numProps)
  , numStates_(numStates)
  , accepting_(numStates_, false)
  , transitions_(numStates_)
  , distances_(numStates_, std::numeric_limits<unsigned int>::max())
  , lowerbound_(numStates_, 0.0)
  , upperbound_(numStates_, std::numeric_limits<double>::max())
{
}

unsigned int ompl::control::Automaton::addState(bool accepting)
{
    ++numStates_;
    accepting_.resize(numStates_);
    accepting_[numStates_ - 1] = accepting;
    transitions_.resize(numStates_);
    distances_.resize(numStates_, std::numeric_limits<unsigned int>::max());
    lowerbound_.resize(numStates_);
    lowerbound_[numStates_ - 1] = 0.0;
    upperbound_.resize(numStates_);
    upperbound_[numStates_ -1] = 100.0;
    return numStates_ - 1;
}

unsigned int ompl::control::Automaton::addState(bool accepting, double lowerbound, double upperbound)
{
    ++numStates_;
    accepting_.resize(numStates_);
    accepting_[numStates_ - 1] = accepting;
    transitions_.resize(numStates_);
    distances_.resize(numStates_, std::numeric_limits<unsigned int>::max());
    lowerbound_.resize(numStates_);
    lowerbound_[numStates_ - 1] = lowerbound;
    upperbound_.resize(numStates_);
    upperbound_[numStates_ -1] = upperbound;
    return numStates_ - 1;
}

void ompl::control::Automaton::setAccepting(unsigned int s, bool a)
{
    accepting_[s] = a;
}

bool ompl::control::Automaton::isAccepting(unsigned int s) const
{
    return accepting_[s];
}

void ompl::control::Automaton::setStartState(unsigned int s)
{
    startState_ = s;
    lowerbound_[0] = 0.0;
    upperbound_[0] = 100.0;
}

int ompl::control::Automaton::getStartState() const
{
    return startState_;
}

void ompl::control::Automaton::setTimeBound(unsigned int s, double lowerbound, double upperbound)
{
    lowerbound_[s] = lowerbound;
    upperbound_[s] = upperbound;
}

std::pair<double, double> ompl::control::Automaton::getTimeBound(int s)
{
    return std::make_pair(lowerbound_[s], upperbound_[s]);
}

void ompl::control::Automaton::addTransition(unsigned int src, const World &w, unsigned int dest)
{
    TransitionMap &map = transitions_[src];
    map.entries[w].push_back(dest);
}

bool ompl::control::Automaton::run(const std::vector<World> &trace) const
{
    int current = startState_;
    for (const auto &w : trace)
    {
        current = step(current, w)[0];
        if (current == -1)
            return false;
    }
    return true;
}

std::vector<int > ompl::control::Automaton::step(int state, const World &w) const
{   
    std::vector< int> states;
    if (state == -1){
        states.push_back(-1);
        return states;
    }
    return transitions_[state].eval(w);
}

ompl::control::Automaton::TransitionMap &ompl::control::Automaton::getTransitions(unsigned int src)
{
    return transitions_[src];
}

unsigned int ompl::control::Automaton::numStates() const
{
    return numStates_;
}

unsigned int ompl::control::Automaton::numTransitions() const
{
    unsigned int ntrans = 0;
    for (const auto &transition : transitions_)
        ntrans += transition.entries.size();
    return ntrans;
}

unsigned int ompl::control::Automaton::numProps() const
{
    return numProps_;
}

void ompl::control::Automaton::print(std::ostream &out) const
{
    out << "digraph automaton {" << std::endl;
    out << "rankdir=LR" << std::endl;
    for (unsigned int i = 0; i < numStates_; ++i)
    {
        out << i << R"( [label=")" << i << R"(",shape=)";
        out << (accepting_[i] ? "doublecircle" : "circle");
        out << "lower bound: "  << lowerbound_[i] <<  ", upper bound: " << upperbound_[i] << "]" << std::endl;

        for (const auto &e : transitions_[i].entries)
        {
            const World &w = e.first;
            std::vector<int> dests = e.second;
            for (auto dest : dests)
            {
            // unsigned int dest = e.second[0]; //FIX THIS
            const std::string formula = w.formula();
            out << i << " -> " << dest << R"( [label=")" << formula << R"("])" << std::endl;
            }
        }
    }
    out << "}" << std::endl;
}

unsigned int ompl::control::Automaton::distFromAccepting(unsigned int s) const
{
    if (distances_[s] < std::numeric_limits<unsigned int>::max())
        return distances_[s];
    if (accepting_[s])
        return 0;
    std::queue<unsigned int> q;
    std::unordered_set<unsigned int> processed;
    std::unordered_map<unsigned int, unsigned int> distance;

    q.push(s);
    distance[s] = 0;
    processed.insert(s);

    while (!q.empty())
    {
        unsigned int current = q.front();
        q.pop();
        if (accepting_[current])
        {
            distances_[s] = distance[current];
            return distance[current];
        }
        for (const auto &e : transitions_[current].entries)
        {
            unsigned int neighbor = e.second[0]; 
            if (processed.count(neighbor) > 0)
                continue;
            q.push(neighbor);
            processed.insert(neighbor);
            distance[neighbor] = distance[current] + 1;
        }
    }
    return std::numeric_limits<unsigned int>::max();
}

ompl::control::AutomatonPtr ompl::control::Automaton::NodeEventually(AutomatonPtr phi, unsigned int numProps, std::vector<int> props, bool final_accepting,
                                                                        double lowerbound, double upperbound)
{

    // phi->addState(false, lowerbound, upperbound);
    int numPrevStates = phi->numStates();
    int num_new_states = 1 << props.size();

    for (unsigned int src = 0; src < num_new_states; ++src)
    {

        phi->addState(false, lowerbound, upperbound);
        const boost::dynamic_bitset<> state(props.size(), src);
        World loop(numProps);
        // each value of p is an index of a proposition in covProps
        for (unsigned int p = 0; p < props.size(); ++p)
        {
            // if proposition covProps[p] has already been covered at state src, skip it
            if (state[p])
                continue;
            // for each proposition covProps[p] that has not yet been
            // covered at state src, construct a transition from src to (src|p)
            // on formula (covProps[p]==true)
            boost::dynamic_bitset<> target(state);
            target[p] = true;
            World nextProp(numProps);
            nextProp[props[p]] = true;
            phi->lowerbound_[src + numPrevStates] = lowerbound;
            phi->upperbound_[src + numPrevStates] = upperbound;
            phi->addTransition(src + numPrevStates, nextProp, target.to_ulong()+ numPrevStates);
            // also build a loop from src to src on formula with conjunct (covProps[p]==false)
            loop[props[p]] = false;
        }
        // now we add a loop from src to src on conjunction of (covProps[p]==false)
        // for every p such that the pth bit of src is 1
        phi->lowerbound_[src + numPrevStates] = lowerbound;
        phi->upperbound_[src + numPrevStates] = upperbound;
        phi->addTransition(src + numPrevStates, loop, src + numPrevStates);
    }

    World trivialtransition(numProps);
    phi->addTransition(numPrevStates - 1, trivialtransition, numPrevStates);
    if (final_accepting)
        phi->setAccepting(phi->numStates() - 1, true);
    return phi;
}

ompl::control::AutomatonPtr ompl::control::Automaton::NodeGlobally(AutomatonPtr phi, unsigned int numProps, std::vector<int> props, bool final_accepting,
                                                                        const double lowerbound, double upperbound)
{

    phi->addState(false, lowerbound, upperbound);
    
    int state = phi->numStates() - 1;
    World trivialtransition(numProps);
    phi->addTransition(state-1, trivialtransition, state);

    World loop(numProps);
    for (int i = 0; i < props.size(); ++i)
    {
        loop[props[i]] = true;
    }
    phi->lowerbound_[state] = lowerbound;
    phi->upperbound_[state] = upperbound;
    phi->addTransition(state, loop, state);

    if (final_accepting)
    {
        phi->addState(true, upperbound, std::numeric_limits<double>::max());
        World progress(numProps);
        phi->addTransition(state, progress, state+1);
    }

    return phi;
}

ompl::control::AutomatonPtr ompl::control::Automaton::NodeEventuallyGlobally(AutomatonPtr phi, unsigned int numProps, std::vector<int> props_eventually, std::vector<int> props_globally, bool final_accepting,
                                                                        const double lowerbound, double upperbound)
{


    //TODO: fix shifting of bitshift

    // phi->addState(false, lowerbound, upperbound);
    
    int numPrevStates = phi->numStates();

    int num_new_states = 1 << props_eventually.size();

    for (unsigned int src = 0; src < num_new_states; ++src)
    {
        phi->addState(false, lowerbound, upperbound);


        const boost::dynamic_bitset<> state(props_eventually.size(), src);
        World loop(numProps);
        // each value of p is an index of a proposition in covProps
        for (unsigned int p = 0; p < props_eventually.size(); ++p)
        {
            // if proposition covProps[p] has already been covered at state src, skip it
            if (state[p])
                continue;
            // for each proposition covProps[p] that has not yet been
            // covered at state src, construct a transition from src to (src|p)
            // on formula (covProps[p]==true)
            boost::dynamic_bitset<> target(state);
            target[p] = true;
            World nextProp(numProps);
            nextProp[props_eventually[p]] = true;
            for (int i = 0; i < props_globally.size(); ++i)
            {
                nextProp[props_globally[i]] = true;
            }
            phi->lowerbound_[src + numPrevStates] = lowerbound;
            phi->upperbound_[src + numPrevStates] = upperbound;
            phi->addTransition(src + numPrevStates, nextProp, target.to_ulong() + numPrevStates);
            // also build a loop from src to src on formula with conjunct (covProps[p]==false)
            loop[props_eventually[p]] = false;
        }
        for (int i = 0; i < props_globally.size(); ++i)
        {
            loop[props_globally[i]] = true;
        }
        // now we add a loop from src to src on conjunction of (covProps[p]==false)
        // for every p such that the pth bit of src is 1
        phi->lowerbound_[src + numPrevStates] = lowerbound;
        phi->upperbound_[src + numPrevStates] = upperbound;
        phi->addTransition(src + numPrevStates, loop, src + numPrevStates);
    }

    World trivialtransition(numProps);
    phi->addTransition(numPrevStates - 1, trivialtransition, numPrevStates);

    if (final_accepting)
    {
        int state = phi->numStates() - 1;
        phi->addState(true, upperbound, std::numeric_limits<double>::max());
        World progress(numProps);
        phi->addTransition(state, progress, state+1);
    }
    return phi;
}

struct ParseTreeNode
{
    double lowerbound;
    double upperbound;
    std::vector<int> formula_type; //F is 0, G is 1, U is 2
    std::vector<int> formula_props;
};


ompl::control::AutomatonPtr ompl::control::Automaton::STLtoAutomaton(std::string strFormula, unsigned int numProps)
{
    strFormula.erase(std::remove(strFormula.begin(), strFormula.end(), ' '), strFormula.end());

    std::vector<ParseTreeNode> parseTree;

    //Add nodes to parse tree
    while (strFormula.length() > 0)
    {
        ParseTreeNode newNode;
        newNode.lowerbound = -1;
        newNode.upperbound = -1;
        newNode.formula_type = std::vector<int>();
        newNode.formula_props = std::vector<int>();

        if (strFormula[0] == '&')
        {
            strFormula = strFormula.substr(1);
        }

        // if the first character is a temporal operator
        if (strFormula[0] == 'F' || strFormula[0] == 'G')
        {
            int formula_type = -1;
            if (strFormula[0] == 'F')
            {
                formula_type = 0;
            }
            else
            {
                formula_type = 1;
            }

            // identify and save time bounds and proposition
            assert(("Temporal operators must have time bounds, e.g. F[0,2]", strFormula[1] == '['));
            int charindex = strFormula.find(',');
            assert(("temporal operators must have time bounds separated by commas, e.g. F[0,2]", charindex != std::string::npos));
            std::string leftstring = strFormula.substr(2, (charindex));
            std::string rightstring = strFormula.substr(charindex + 1);
            double lowerbound = std::stod(leftstring);
            charindex = rightstring.find(']');
            assert(("must close interval with a right bracket, ex: F[0,2]", charindex != std::string::npos));
            leftstring = rightstring.substr(0, (charindex));
            rightstring = rightstring.substr(charindex + 1);
            double upperbound = std::stod(leftstring);
            int proposition = std::stoi(rightstring);
            strFormula = rightstring.substr(1);

            if (parseTree.size() == 0)
            {
                //add the node to the tree
                newNode.lowerbound = lowerbound;
                newNode.upperbound = upperbound;
                newNode.formula_type.push_back(formula_type);
                newNode.formula_props.push_back(proposition);
                parseTree.push_back(newNode);
            }
            else
            {
                bool nodefound = false;
                for (ParseTreeNode &node : parseTree)
                {
                    if (lowerbound == node.lowerbound && upperbound == node.upperbound)
                    {
                        node.formula_type.push_back(formula_type);
                        node.formula_props.push_back(proposition);
                        nodefound = true;
                    }
                }
                if (!nodefound)
                {
                    newNode.lowerbound = lowerbound;
                    newNode.upperbound = upperbound;
                    newNode.formula_type.push_back(formula_type);
                    newNode.formula_props.push_back(proposition);
                    parseTree.push_back(newNode);
                }
            }
        }
    }

    auto automaton(std::make_shared<Automaton>(numProps, 1));
    automaton->setStartState(0);

    // print parse tree details
    // for (ParseTreeNode node : parseTree)
    // {
    //     std::cout << "lowerbound: " << node.lowerbound << std::endl;
    //     std::cout << "upperbound: " << node.upperbound << std::endl;
    //     std::cout << "formula_type: ";
    //     for (int i = 0; i < node.formula_type.size(); ++i)
    //     {
    //         std::cout << node.formula_type[i] << " ";
    //     }
    //     std::cout << std::endl;
    //     std::cout << "formula_props: ";
    //     for (int i = 0; i < node.formula_props.size(); ++i)
    //     {
    //         std::cout << node.formula_props[i] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    int i = 0;
    for (ParseTreeNode node : parseTree)
    {
        ++i;
        bool final = false;
        if (i == parseTree.size())
            final = true;

        std::vector<int> eventuallys;
        std::vector<int> globallys;
        
        for (int i = 0; i < node.formula_type.size(); ++i)
        {
            if (node.formula_type[i] == 0)
            {
                eventuallys.push_back(node.formula_props[i]);
            }
            else if (node.formula_type[i] == 1)
            {
                globallys.push_back(node.formula_props[i]);
            }
        }

        if (eventuallys.size() == 0)
        {
            // make a globally automaton node
            automaton = NodeGlobally(automaton, numProps, globallys, final, node.lowerbound, node.upperbound);
        }

        else if (globallys.size() == 0)
        {
            // make an eventually automaton node
            automaton = NodeEventually(automaton, numProps, eventuallys, final, node.lowerbound, node.upperbound);
        }
        else
        {
            // make an eventually globally automaton
            automaton = NodeEventuallyGlobally(automaton, numProps, eventuallys, globallys, final, node.lowerbound, node.upperbound);
        }
    }

    return automaton;
}

std::vector<std::string> ompl::control::Automaton::STLFormulaSeparation(std::string strFormula, std::vector<double > separationTimes)
{

    strFormula.erase(std::remove(strFormula.begin(), strFormula.end(), ' '), strFormula.end());

    std::vector<std::string> formulas;

    std::vector<std::vector<ParseTreeNode>> EventuallyNodes;
    std::vector<ParseTreeNode> conjunctNodes;
    
    while (strFormula.length() > 0)
    {
        if (strFormula[0] == '&')
        {
            strFormula = strFormula.substr(1);
        }

        if (strFormula[0] == 'F' || strFormula[0] == 'G')
        {            
            int formula_type = -1;
            if (strFormula[0] == 'F')
            {
                formula_type = 0;
            }
            else
            {
                formula_type = 1;
            }

            // identify time bounds and proposition
            assert(("Temporal operators must have time bounds, e.g. F[0,2]", strFormula[1] == '['));
            int charindex = strFormula.find(',');
            assert(("temporal operators must have time bounds separated by commas, e.g. F[0,2]", charindex != std::string::npos));
            std::string leftstring = strFormula.substr(2, (charindex));
            std::string rightstring = strFormula.substr(charindex + 1);
            double lowerbound = std::stod(leftstring); //
            charindex = rightstring.find(']');
            assert(("must close interval with a right bracket, ex: F[0,2]", charindex != std::string::npos));
            leftstring = rightstring.substr(0, (charindex));
            rightstring = rightstring.substr(charindex + 1);
            double upperbound = std::stod(leftstring); 
            int proposition = std::stoi(rightstring); 
            strFormula = rightstring.substr(1);


            std::vector<double> times;
            times.push_back(lowerbound);
            for (int i = 0; i < separationTimes.size(); ++i)
            {
                if (lowerbound < separationTimes[i] && upperbound > separationTimes[i])
                {
                    double newtime = separationTimes[i];
                    times.push_back(newtime);
                }
            }
            times.push_back(upperbound);

            std::vector<ParseTreeNode> disjunctNodes;
            
            for (int i = 0; i < times.size() - 1; ++i)
            {
                ParseTreeNode newNode;
                newNode.formula_type = std::vector<int>();
                newNode.formula_props = std::vector<int>();

                if (formula_type == 0)
                {
                    newNode.lowerbound = times[i];
                    newNode.upperbound = times[i+1];
                    newNode.formula_type.push_back(formula_type);
                    newNode.formula_props.push_back(proposition);
                    disjunctNodes.push_back(newNode);
                }
                else
                {
                    newNode.lowerbound = times[i];
                    newNode.upperbound = times[i+1];
                    newNode.formula_type.push_back(formula_type);
                    newNode.formula_props.push_back(proposition);
                    conjunctNodes.push_back(newNode);
                }
            }

            EventuallyNodes.push_back(disjunctNodes);
        }
    }

    for (int i = 0; i < EventuallyNodes.size(); ++i)
    {
        for (auto disjunctNode : EventuallyNodes[i])
        {
            std::string formula = "F[" + std::to_string(disjunctNode.lowerbound) + "," + std::to_string(disjunctNode.upperbound) + "]" + std::to_string(disjunctNode.formula_props[0]);

            for (auto conjunctNode : conjunctNodes)
            {
                if (conjunctNode.lowerbound < disjunctNode.lowerbound && conjunctNode.upperbound < disjunctNode.upperbound)
                {
                    formula = "&G[" + std::to_string(conjunctNode.lowerbound) + "," + std::to_string(conjunctNode.upperbound) + "]" + std::to_string(conjunctNode.formula_props[0]) + "&" + formula;
                }
                else
                {
                    formula = formula + "&G[" + std::to_string(conjunctNode.lowerbound) + "," + std::to_string(conjunctNode.upperbound) + "]" + std::to_string(conjunctNode.formula_props[0]);
                }
            }

            for (int j = 0; j < EventuallyNodes.size(); ++j)
            {
                if (j != i)
                {
                    for (auto disjunctNode2 : EventuallyNodes[j])
                    {
                        if (disjunctNode2.lowerbound < disjunctNode.lowerbound && disjunctNode2.upperbound < disjunctNode.upperbound)
                        {
                            formula = "&F[" + std::to_string(disjunctNode2.lowerbound) + "," + std::to_string(disjunctNode2.upperbound) + "]" + std::to_string(disjunctNode2.formula_props[0]) + formula;
                        }
                        else
                        {
                            formula = formula + "&F[" + std::to_string(disjunctNode2.lowerbound) + "," + std::to_string(disjunctNode2.upperbound) + "]" + std::to_string(disjunctNode2.formula_props[0]);
                        }
                    }
                }
            }
            if (formula[0] == '&')
            {
                formula = formula.substr(1);
            }
            formulas.push_back(formula);
        }
    }
    return formulas;
}