# Overview

This repository provides functionality to convert simple STL formulas into timed automata structures.

## Functionality

The STLAutomaton library includes the following main functions:

- `STLtoAutomaton()`: Returns a timed automaton from a given formula.
- `STLFormulaSeparation()`: Returns a vector of formulas (in Disjunctive Normal Form)
- `print()`: Prints timed automaton.

There are some limitations in the implementation:

1. It works directly with propositions instead of predicates.
1. Currently, it only handles eventually and globally operators. Until and negation are work in progress.
2. When doing syntactic separation, it outputs multiple formulas as "disjunctive normal form". So, one gets multiple automata that are disjunctions of each other. The combined automaton is a work in progress.
3. It currently assumes that propositions are disjoint (there is no overlap between them).

## Usage

cmake .
make
./main.a