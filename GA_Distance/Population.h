//
// Created by Jean-Baptiste Bolh on 11/14/19.
//

#ifndef GA_POPULATION_H
#define GA_POPULATION_H

#include "Robot.h"

class Population {

private:
    int popSize;
    std::vector<Robot> individuals;

public:
    //CONSTRUCTORS
    explicit Population(int size);
    Population(int size, std::vector<Robot> robots);

    //GETTERS
    int getPopSize() const;
    std::vector<Robot> getIndividuals() const;

    //SETTERS
    void setIndividuals(std::vector<Robot> robots);

    //GA FUNCTIONS
    void createIndividuals(int size); //Initializes a population of Robots of length equal to the size parameter
    Population makeChildren(); //Returns a population of mutated individuals derived from the population
    std::vector<Robot> replaceWithIfBetter(Population &children);
    std::vector<Robot> tournamentSelection();
    std::vector<Robot> replaceParentsSimpleGA(Population &children);

    //AFPO FUNCTIONS
    //Doubles the size of the population using tournament selection
    void expandPop(); //Doubles the size of the population using tournament selection
    void agePop(); //Add one to the age of every robot in the population
    void injectNewIndividual(); //Injects a random new individual into the population
    void evaluateFitnesses(); //Evaluates the fitness of every robot in the population
    void contractPop(); //Eliminates dominated robots from the population
    bool aggressorDominates(int agressorIdx, int defenderIdx); //Used in contractPop
    bool dominatedExist();

    //UTILITY FUNCTIONS
    void addBestFitnessToFileForGeneration(); //Add the fitness of the best individual in the population to file "bestFitnessEachGen.txt"
    void writeGenerationStatsForRun(int genNum, int runNum);
    void writeWholePopulationToFile(int genNum); //Writes the whole population to file
    void evaluateAndShowBest(); //Shows the animation for the best robot in the population
    void printBestFitness(); //Shows the best fitness in the population
    double getBestFitness();
    double getAvgFitness();
    std::string getBestCPGParameters();
    std::string getBestMorphParameters();
    void setIndivCPG(int i, std::vector<double> params);
    void setIndivMorph(int i, std::vector<double> params);
    void setIndivParams(int i, std::vector<double> params);
    int nextAvailableID();

    //Overloaded operators
    friend std::ostream &operator << (std::ostream &out, const Population &p);

};


#endif //GA_POPULATION_H
