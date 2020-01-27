//
// Created by Jean-Baptiste Bolh on 11/14/19.
//
#include <random>
#include <iostream>
#include <fstream>

#include "Population.h"
#include <unistd.h>

//Constructors
Population::Population(int size){
    popSize = size;
    createIndividuals(popSize);
}

Population::Population(int size, std::vector<Robot> robots){
    popSize = size;
    individuals = robots;
}

//GETTERS
int Population::getPopSize() const {
    return popSize;
}
std::vector<Robot> Population::getIndividuals() const {
    return individuals;
}

//SETTERS
void Population::setIndividuals(std::vector<Robot> robots){
    individuals = robots;
}

/** GENETIC ALGORITHM METHODS **/
//Creates a population of random robots
void Population::createIndividuals(int size){
    for(int i=0; i<size; i++){
        Robot r(i);
        individuals.push_back(r);
    }
}
//Copies parent vector and mutates each individual in it
Population Population::makeChildren() {
    //Create Children and mutate
    std::vector<Robot> childrenIndividuals = individuals;

    for(int i = 0; i < size(childrenIndividuals); i++){
        childrenIndividuals[i].mutate();
        //Add one to the age of the child
        int newChildAge = childrenIndividuals[i].getAge() + 1;
        childrenIndividuals[i].setAge(newChildAge);
    }

    Population children = Population(int(size(childrenIndividuals)), childrenIndividuals);
    return children;
}
std::vector<Robot> Population::replaceWithIfBetter(Population &children){
    for(int i=0; i < popSize; i++) {
        if (children.getIndividuals()[i].getFitness() < individuals[i].getFitness()) {
            individuals[i] = children.getIndividuals()[i];
        }
    }
    return individuals;
}
std::vector<Robot> Population::tournamentSelection(){
    //Perform tournament selection
    int seed = int(std::chrono::system_clock::now().time_since_epoch().count());
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<> distribution(0, popSize-1);

    for(int i=0; i < size(individuals); i++) {
        //Randomly select 2 parents
        int p1 = distribution(generator);
        int p2 = distribution(generator);

        //If both parents are the same reroll selection of parents
        while (p1 == p2) {
            p1 = distribution(generator);
        }

        //Compare fitnesses and select appropriately
        if (individuals[p1].getFitness() < individuals[p2].getFitness()) {
            individuals[p2] = individuals[p1];
        } else if (individuals[p2].getFitness() < individuals[p1].getFitness()) {
            individuals[p1] = individuals[p2];
        }
    }
    return individuals;
}
//Replaces parents with child if child is better (for every parent/child pair)
std::vector<Robot> Population::replaceParentsSimpleGA(Population &children){

    //If child is better than parent replace
    individuals = replaceWithIfBetter(children);

    //Perform tournament selection
    individuals = tournamentSelection();

    return individuals;
}
/** END GENETIC ALGORITHM METHODS **/

/** AFPO **/
//Doubles the size of the population
void Population::expandPop(){
    popSize = int(size(individuals));
    for(int i = popSize; i<2*popSize-1; i++){
        //Pick a robot to have a baby
        int seed = int(std::chrono::system_clock::now().time_since_epoch().count());
        std::default_random_engine generator (seed);
        std::uniform_int_distribution<> distribution(0, popSize-1);
        //Get the index of the parent
        int parentIndex = distribution(generator);

        //Copy, Mutate, SetID (CHILD MAYBE DOESN'T EXIST OUTSIDE OF THIS FUNCTION??)
        Robot child = Robot(individuals[parentIndex]);
        child.setFitness(1000000);
        child.setID(nextAvailableID());
        child.mutate();
        individuals.push_back(child);
    }
}

void Population::agePop() {
    for (int i = 0; i < size(individuals); i++){
        int newAgeOfIndividual = individuals[i].getAge() + 1;
        individuals[i].setAge(newAgeOfIndividual);
    }
}
void Population::injectNewIndividual(){
    //We don't update the population size, that will happen in the contract phase of the algorithm
    Robot newRobot = Robot(nextAvailableID());
    newRobot.setFitness(1000000);
    individuals.push_back(newRobot);
}
//Evaluate the fitness of every individual in the population
void Population::evaluateFitnesses(){
    for(int i=0; i<size(individuals); i++){
        individuals[i].evaluateFitness(false);
    }
}
void Population::contractPop(){
    while(size(individuals) > popSize){
        bool aggressorDominatesDefender = false;
        int aggressorIndex;
        int defenderIndex;
        bool dominatedRobotExists = dominatedExist();
        if (dominatedRobotExists){
            while (!aggressorDominatesDefender){
                //Pick a random robot
                int seed = int(std::chrono::system_clock::now().time_since_epoch().count());
                std::default_random_engine generator (seed);
                std::uniform_int_distribution<> distribution(0, size(individuals)-1);
                //Get the index of the parent
                aggressorIndex = distribution(generator);
                defenderIndex = distribution(generator);
                while(aggressorIndex == defenderIndex){
                    defenderIndex = distribution(generator);
                }
                //Check pareto dominance for aggressor/defender
                aggressorDominatesDefender = aggressorDominates(aggressorIndex, defenderIndex);
            }
            for(int i = defenderIndex; i < size(individuals)-1; i++){
                individuals[i] = individuals[i+1];
            }
            //Remove the last thing in individuals
            individuals.pop_back();
        }
        //If no dominated robots exist, get rid of the robot with the lowest fitness (Lowest age)
        else{
            individuals.pop_back(); //Last robot added to individuals will be the youngest/have the lowest fitness
        }
    }
}
bool Population::aggressorDominates(int aggressorIdx, int defenderIdx){
    if(individuals[aggressorIdx].getFitness() <= individuals[defenderIdx].getFitness() && individuals[aggressorIdx].getAge() <= individuals[defenderIdx].getAge()){
        return true;
    }
    else {
        return false;
    }
}
bool Population::dominatedExist(){
    bool dominatedExists = false;
    for(int i = 0; i < size(individuals)-1; i++){
        for(int j=i+1; j<size(individuals); j++){
            if(individuals[i].getFitness() <= individuals[j].getFitness() && individuals[i].getAge() <= individuals[j].getAge()){
                dominatedExists = true;
            }
        }
    }
    return dominatedExists;
}
/** END AFPO **/

/** UTILITY FUNCTIONS **/
void Population::addBestFitnessToFileForGeneration(){
    double bestFitness = 100000;
    for(int i=0; i<popSize; i++){
        if(individuals[i].getFitness() < bestFitness){
            bestFitness = individuals[i].getFitness();
        }
    }

    std::ofstream myfile;
    myfile.open ("bestFitnessEachGen.txt", std::ios::app);
    myfile << std::to_string(bestFitness) << std::endl;
    myfile.close();
}
void Population::writeGenerationStatsForRun(int genNum, int runNum){
    std::ofstream myfile;
    std::string name = "/Users/JeanBaptisteBolh/Documents/School/CS352/Final/GA_COT/Outputs/Run_" + std::to_string(runNum) + ".txt";
    myfile.open (name, std::ios::app);
    myfile << std::to_string(genNum) << " " << std::to_string(getBestFitness()) << " " << std::to_string(getAvgFitness()) << " " << getBestCPGParameters() << " " << getBestMorphParameters() << std::endl;
    myfile.close();
}
void Population::writeWholePopulationToFile(int genNum){
    std::ofstream myfile;
    std::string name = "/Users/JeanBaptisteBolh/Documents/School/CS352/Final/GA_COT/Outputs/Gen_" + std::to_string(genNum) + ".txt";
    myfile.open (name, std::ios::app);
    for(int i=0; i<popSize; i++){
        myfile << genNum << " " << std::to_string(individuals[i].getID()) << " " << std::to_string(individuals[i].getFitness()) << " " << std::to_string(individuals[i].getAge()) << std::endl;
    }
    myfile.close();
}
void Population::evaluateAndShowBest(){
    int idxOfBest = 0;
    //double fitOfBest = std::numeric_limits<int>::max();
    double fitOfBest = 1000000;

    for(int i = 0; i < size(individuals); i++){
        if(individuals[i].getFitness() < fitOfBest){
            idxOfBest = i;
            fitOfBest = individuals[i].getFitness();
        }
    }

    std::cout << "BEST INDIVIDUAL " << individuals[idxOfBest] << std::endl;

    individuals[idxOfBest].evaluateFitness(true);

}
void Population::printBestFitness(){
    int idxOfBest = 0;
    //double fitOfBest = std::numeric_limits<int>::max();
    double fitOfBest = 1000000;
    for(int i = 0; i < size(individuals); i++){
        if(individuals[i].getFitness() < fitOfBest){
            idxOfBest = i;
            fitOfBest = individuals[i].getFitness();
        }
    }
    std::cout << "BEST FIT: " << individuals[idxOfBest].getFitness() << std::endl;
}
int Population::nextAvailableID(){
    //Get the id of the last parent in the population (largest ID)
    return individuals.back().getID() + 1;
}
double Population::getBestFitness(){
    int idxOfBest = 0;
    //double fitOfBest = std::numeric_limits<int>::max();
    double fitOfBest = 1000000;
    for(int i = 0; i < size(individuals); i++){
        if(individuals[i].getFitness() < fitOfBest){
            idxOfBest = i;
            fitOfBest = individuals[i].getFitness();
        }
    }
    return individuals[idxOfBest].getFitness();
}

double Population::getAvgFitness(){
    double totalFitness = 0;
    for(int i = 0; i < size(individuals); i++){
        totalFitness += individuals[i].getFitness();
    }
    double sizeOfIndividuals = double(size(individuals));
    return totalFitness/sizeOfIndividuals;
}

std::string Population::getBestCPGParameters(){
    int idxOfBest = 0;
    //double fitOfBest = std::numeric_limits<int>::max();
    double fitOfBest = 1000000;
    for(int i = 0; i < size(individuals); i++){
        if(individuals[i].getFitness() < fitOfBest){
            idxOfBest = i;
            fitOfBest = individuals[i].getFitness();
        }
    }
    std::vector<double> bestCPGParameters = individuals[idxOfBest].getCPGParameters();
    std::string cpgParameterString;

    for(int i = 0; i<size(bestCPGParameters); i++){
        cpgParameterString += std::to_string(bestCPGParameters[i]) + " ";
    }

    return cpgParameterString;

}
std::string Population::getBestMorphParameters(){
    int idxOfBest = 0;
    //double fitOfBest = std::numeric_limits<int>::max();
    double fitOfBest = 1000000;
    for(int i = 0; i < size(individuals); i++){
        if(individuals[i].getFitness() < fitOfBest){
            idxOfBest = i;
            fitOfBest = individuals[i].getFitness();
        }
    }
    std::vector<double> bestMorphParameters = individuals[idxOfBest].getMorphParameters();
    std::string morphParameterString;

    for(int i = 0; i<size(bestMorphParameters); i++){
        morphParameterString += std::to_string(bestMorphParameters[i]) + " ";
    }

    return morphParameterString;
}


/** END UTILITY FUNCTIONS **/


//Overloaded output operator
std::ostream &operator << (std::ostream &out, const Population &p){
    for(int i=0; i<size(p.individuals); i++){
        out << p.individuals[i] << std::endl;
    }
    return out;
}