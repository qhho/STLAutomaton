/* Author: Qi Heng Ho, adapted from code in the OMPL LTL extension by Matt Maly, Keliang He */

#ifndef OMPL_CONTROL_PLANNERS_STL_AUTOMATON_
#define OMPL_CONTROL_PLANNERS_STL_AUTOMATON_

#include "ompl/control/planners/ltl/World.h"
#include "ompl/util/ClassForward.h"
#include "ompl/config.h"
#include <unordered_map>
#include <limits>
#include <ostream>
#include <vector>

namespace ompl
{
    namespace control
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::Automaton */
        OMPL_CLASS_FORWARD(Automaton);
        /// @endcond

        /** \class ompl::control::AutomatonPtr
            \brief A shared pointer wrapper for ompl::control::Automaton */

        /** \brief A class to represent a deterministic finite automaton,
            each edge of which corresponds to a World.
            A system trajectory, by way of project() and worldAtRegion()
            in PropositionalDecomposition, determines a sequence of Worlds,
            which are read by an Automaton to determine whether a trajectory
            satisfies a given specification.

            An automaton is meant to be run in a read-only fashion, i.e.,
            it does not keep track of an internal state and can be thought of
            as a lookup table. */
        class Automaton
        {
        public:
            /** \brief Each automaton state has a transition map, which maps from a
                World to another automaton state.
                A set \f$P\f$ of true propositions correponds to the formula
                \f$\bigwedge_{p\in P} p\f$. */
            struct TransitionMap
            {
                /** \brief Returns the automaton state corresponding to a given
                    World in this transition map.
                    Returns -1 if no such transition exists. */
                // int eval(const World &w) const;

                // int eval(const World &w, double time) const;

                std::vector<int> eval(const World &w) const;

                TransitionMap &operator=(const TransitionMap &tm) = default;

                // mutable std::unordered_map<World, unsigned int> entries;
                mutable std::unordered_map<World, std::vector<int>> entries;
            };

            /** \brief Creates an automaton with a given number of propositions and states. */
            Automaton(unsigned int numProps, unsigned int numStates = 0);

            /** \brief Adds a new state to the automaton and returns an ID for it. */
            unsigned int addState(bool accepting = false);

            /** \brief Adds a new state to the automaton and returns an ID for it. */
            unsigned int addState(bool accepting = false, double lowerbound = 0.0, double upperbound = 100.0);

            /** \brief Sets the accepting status of a given state. */
            void setAccepting(unsigned int s, bool a);

            /** \brief Returns whether a given state of the automaton is accepting. */
            bool isAccepting(unsigned int s) const;

            /** \brief Sets the start state of the automaton. */
            void setStartState(unsigned int s);

            /** \brief Returns the start state of the automaton.
                Returns -1 if no start state has been set. */
            int getStartState() const;

            /** \brief Sets the time boudns of the automaton state. */
            void setTimeBound(unsigned int s, double lowerbound, double upperbound);

            std::pair<double, double> getTimeBound(int s);

            /** \brief Adds a given transition to the automaton. */
            void addTransition(unsigned int src, const World &w, unsigned int dest);

            /** \brief Runs the automaton from its start state, using
                the values of propositions from a given sequence of Worlds.
                Returns false if and only if the result is a nonexistent state
                (i.e., if and only if there does not exist an extension to trace
                that will lead it to an accepting state). */
            bool run(const std::vector<World> &trace) const;

            /** \brief Runs the automaton for one step from the given state,
                using the values of propositions from a given World.
                Returns the resulting state, or -1 if the result is a nonexistent state. */
            // int step(int state, const World &w) const;

            std::vector<int > step(int state, const World &w) const;

            /** \brief Returns the outgoing transition map for a given automaton state. */
            TransitionMap &getTransitions(unsigned int src);

            /** \brief Returns the number of states in this automaton. */
            unsigned int numStates() const;

            /** \brief Returns the number of transitions in this automaton. */
            unsigned int numTransitions() const;

            /** \brief Returns the number of propositions used by this automaton. */
            unsigned int numProps() const;

            /** \brief Prints the automaton to a given output stream, in Graphviz dot format. */
            void print(std::ostream &out) const;

            /** \brief Returns the shortest number of transitions from a given state to
                an accepting state. */
            unsigned int distFromAccepting(unsigned int s) const;

            static AutomatonPtr NodeEventually(AutomatonPtr automaton, unsigned int numProps, std::vector<int> props, bool final_accepting,
                                                                        double lowerbound, double upperbound);
            static AutomatonPtr NodeGlobally(AutomatonPtr automaton, unsigned int numProps, std::vector<int> props, bool final_accepting,
                                                                        const double lowerbound, double upperbound);
            static AutomatonPtr NodeEventuallyGlobally(AutomatonPtr automaton, unsigned int numProps, std::vector<int> props_eventually, std::vector<int> props_globally, bool final_accepting,
                                                                        const double lowerbound, double upperbound);


            /** \brief Algorithm takes in a time separated formula in the form of "F[a,b]]1&G[c,d]2&..." and converts it to an Automaton-like structure
             *  Currently, it only handles F and G operators.
             *  The formula is assumed to be in the form of "F[a,b]1&G[c,d]2&..."
             *  The formula is assumed to be in sequence, i.e. a <= c, b <= d, etc.
             *  Returns an nondeterministic automaton with time bounds in each node.
             * . */
            static AutomatonPtr STLtoAutomaton(std::string strFormula, unsigned int numProps);


            /** \brief Algorithm takes in a formula in the form of "F[a,b]]1&G[c,d]2&..." and converts it into disjunctions of formulas
             *  Currently, it only handles F and G operators.
             * Returns a vector of syntactically separated formulas to be used in STLtoAutomaton function.
             * . */
            static std::vector<std::string> STLFormulaSeparation(std::string strFormula, std::vector<double > separationTimes);
        protected:
            unsigned int numProps_;
            unsigned int numStates_;
            int startState_{-1};
            std::vector<bool> accepting_;
            std::vector<double> lowerbound_;
            std::vector<double> upperbound_;
            std::vector<TransitionMap> transitions_;
            mutable std::vector<unsigned int> distances_;
        };
    }
}
#endif