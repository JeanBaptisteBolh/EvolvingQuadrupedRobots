//TO MAKE .o FILES:  g++ -c main.cpp Robot.cpp Robot.h -std=c++17
//TO RUN:  g++ -o SimpleGA main.o Robot.o -std=c++17

/**
 * Pass by value f(int n) --> Compiler creates a copy in memory (can be a waste of memory)
 * Pass by reference f(int& n) --> Eliminates the copy in memory, but allows you to change the value
 * Constant reference f(const int& n) --> Read only, we cannot change the value of n
 */

//PROBLEM: AFPO STALLS WHEN NOTHING LIES ON THE NON-DOMINATED FRONT

#include <iostream>
#include <vector>
#include <random>
#include <string>

#include "Robot.h"
#include "Population.h"


Population afpo(Population p, int numGens, int maxPSize, int runNumber){

    for(int i=0; i<numGens; i++){
        std::cout << "Generation: " << i << std::endl;
        p.printBestFitness();
        std::cout << p << std::endl;

        //Expand (Use Tournament selection to add new children)
        p.expandPop();

        //Inject New Random Bot
        p.injectNewIndividual();

        //Add 1 to age of every robot
        p.agePop();

        //Evaluate
        p.evaluateFitnesses();

        //Contract (Reduces the population size)
        p.contractPop();

        //Write stats to file
        p.writeGenerationStatsForRun(i, runNumber);
    }
    return p;
}

//Runs GA on a population and returns it
Population simpleGA(Population p, int numGens, int runNumber){

    //For a number of evaluations, run genetic algorithm
    for(int i=0; i<numGens; i++){
        std::cout << "Generation: " << i << std::endl;

        //Copy parents into children vector and mutate
        Population children = p.makeChildren();

        //Evaluate Fitness
        children.evaluateFitnesses();

        //If children are better than their parents, replace them, and perform tournament selection
        p.replaceParentsSimpleGA(children);

        //Print robots
        std::cout << p << std::endl;

        p.writeGenerationStatsForRun(i, runNumber);
    }

    return p;
}

int main() {
    int popSize = 100;
    int gens = 100;

    /*
    for(int i = 0; i<5; i++){
        Population parents(popSize);
        parents.evaluateFitnesses();
        parents = afpo(parents, gens, popSize, i);
    }*/

    for(int i = 0; i<5; i++){
        Population parents(popSize);
        parents.evaluateFitnesses();
        parents = simpleGA(parents, gens, i);
    }


    return 0;
}


