//Need to ./configure ode directory then make
//For cost of transportation we need to change functions to minimize fitness rather than maximize
//File paths are relative and need to be changed

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

        //Add 1 to age of every robot
        p.agePop();

        //Inject New Random Bot
        p.injectNewIndividual();

        //Evaluate
        p.evaluateFitnesses();

        //Contract (Reduces the population size)
        p.contractPop();

        //Write stats to file
        p.writeGenerationStatsForRun(i, runNumber);
    }
    p.evaluateAndShowBest();
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

/** Differential Evolution **/
Population differentialEvolution(Population p, int numGens, int runNumber){
    //random number generator
    int seed = int(std::chrono::system_clock::now().time_since_epoch().count());
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<> realdist(0, 1);
    std::uniform_int_distribution<> intdist(0, p.getPopSize()-1);
    std::uniform_int_distribution<> intdist2(0, 2);
    //crossover rate, [0, 1]
    double CR = 0.3; //0.2 // decrease it exponentially with time?
    //real and constant vector which controls the amplification of the differential variation, [0, 2]
    double F = 1.2;

    std::vector<double> bounds_min(15);
    std::vector<double> bounds_max(15);
    bounds_min = {0.01, 2.0, 0.05, 0.1, 10, -3.14/3, 0.1, 10, -3.14/3, 0.1, 10, -3.14/3, 0.1, 10, -3.14/3};
    bounds_max = {1.0, 15*3.14, 2.0, 0.5, 100, 3.14/3, 0.5, 100, 3.14/3, 0.5, 100, 3.14/3, 0.5, 100, 3.14/3};
    std::cout<<"run " << runNumber << std::endl;
    for (int count=0; count<numGens; count++){
        std::cout<<"gen " << count << std::endl;
        int NP = p.getPopSize();
        //number of the parameters
        int D = 15;
        Population newPopulation = Population(NP);
        for (int i=0; i<NP; i++){
            //std::cout<<"pop " << i << std::endl;
            // pick three unique random vectors
            int a = intdist(generator);
            while (a == i){
                a = intdist(generator);
            }
            int  b = intdist(generator);
            while ((b == i) || (b == a)){
                b = intdist(generator);
            }
            int c = intdist(generator);
            while ((c == i) || (c == a) || (c == b)){
                c = intdist(generator);
            }
            //randomly pick the first parameter
            //int j = intdist2(generator);
            std::vector<double> params(D);
            for (int k=0; k<D; k++){
                double randNum = realdist(generator);
                if (randNum < CR){ // || (k == D-1)
                    double diff = bounds_max[k] - bounds_min[k];
                    //computing the trial vector, making sure that vectors are normalized
                    if (k<3){ //CPG params
                        params[k] = (p.getIndividuals()[c].getCPGParameters()[k]/diff)+F*(p.getIndividuals()[a].getCPGParameters()[k]/diff - p.getIndividuals()[b].getCPGParameters()[k]/diff);
                    }
                    else{ //Morph params
                        params[k] = (p.getIndividuals()[c].getMorphParameters2()[k]/diff)+F*(p.getIndividuals()[a].getMorphParameters2()[k]/diff - p.getIndividuals()[b].getMorphParameters2()[k]/diff);
                    }
                    //clipping the params or divide it by 3?
                    if (params[k] < 0){
                        params[k] = 0;
                    }
                    else if (params[k] > 1){
                        params[k] = params[k]/3; //1;
                    }
                    //std::cout<<"index " << j << std::endl;
                    //std::cout<<"before " << params[j] << std::endl;
                    params[k] = bounds_min[k] + params[k] * (bounds_max[k] - bounds_min[k]);
                    //std::cout<<"after " << params[j] << std::endl;
                }
                else{
                    if (k<3){
                        params[k] = p.getIndividuals()[i].getCPGParameters()[k];
                    }
                    else{
                        params[k] = p.getIndividuals()[i].getMorphParameters2()[k];
                    }
                }
                //get the next parameter
                //j = (j+1) % D;
            }
            //evaluate the new robot
            Robot newRobot = Robot(); //Robot(p.nextAvailableID());
            newRobot.setParameters(params);
            newRobot.evaluateFitness(false);
            if (newRobot.getFitness() >= p.getIndividuals()[i].getFitness()){
                //std::cout<<"happened" <<std::endl;
                //std::cout<< params[0] <<std::endl;
                //std::cout<< newRobot.getCPGParameters()[0] <<std::endl;
                newPopulation.setIndivParams(i, params);
                //newPopulation.getIndividuals()[i].setCPGParameters(params);//= newRobot;
                //std::cout<< newPopulation.getIndividuals()[i].getCPGParameters()[0] <<std::endl;
            }
            else{
                //newPopulation.getIndividuals()[i] = p.getIndividuals()[i];
                newPopulation.setIndivCPG(i, p.getIndividuals()[i].getCPGParameters());
                newPopulation.setIndivMorph(i, p.getIndividuals()[i].getMorphParameters());
                //std::cout<< newPopulation.getIndividuals()[i].getCPGParameters()[0] <<std::endl;
                //std::cout<< p.getIndividuals()[i].getCPGParameters()[0] <<std::endl;
            }

        }
        //newPopulation.evaluateFitnesses();
        //std::cout<<"newPop "<<newPopulation<<std::endl;
        //std::cout<<"oldPop "<<p<<std::endl;
        p.setIndividuals(newPopulation.getIndividuals());
        p.evaluateFitnesses();
        //std::cout<<"gen: "<<count<<std::endl;
        //std::cout<<p<<std::endl;
        p.writeGenerationStatsForRun(count, runNumber);
    }
    return p;
}
/** Differential Evolution **/

int main() {
    int popSize = 100;
    int gens = 500;

    for(int i = 0; i<1; i++){
        Population parents(popSize);
        parents.evaluateFitnesses();
        parents = afpo(parents, gens, popSize, i);
    }

    /**
    for(int i = 0; i<1; i++){
        Population parents(popSize);
        parents.evaluateFitnesses();
        parents = simpleGA(parents, gens, i);
    }
    */

    /*
    for(int i = 0; i<5; i++){
        Population parents(popSize);
        parents.evaluateFitnesses();
        parents = differentialEvolution(parents, gens, i);
    }*/


    return 0;
}


