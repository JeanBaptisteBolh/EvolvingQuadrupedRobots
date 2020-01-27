//
// Created by Jean-Baptiste Bolh on 11/4/19.
//

#ifndef GA_ROBOT_H
#define GA_ROBOT_H

#include <vector>
#include <iostream>

class Robot {

private:
    int id;
    double fitness;
    int age;
    //(Num body segments, bodseg length, bodseg mass, bodseg spring k, bodseg angle, num front leg segs, flseg radius, length, mass, spring k, angle, num back leg segs, bl seg radius, length, mass, spring k, angle)
    std::vector<double> morphParams;
    //(ctrl amplitude, ctrl swing, ctrl stance)
    std::vector<double> cpgParams;

public:
    ////////// Constructors //////////
    // Sets an ID for the robot, along with random parameters, fitness set to MAX
    Robot();
    // Sets an ID for the robot, along with random parameters, and fitness set to MAX
    explicit Robot(int idNum);
    // Sets an ID for the robot, along with random CPG parameters, and fitness set to MAX
    Robot(int idNum, std::vector<double> mParams);
    // Sets an ID for the robot, along with parameters, and age
    Robot(int idNum, double fit, std::vector<double> mParams, std::vector<double> cParams);
    //COPY CONSTRUCTOR
    Robot(const Robot &r);


    ////////// GETTERS //////////
    int getID() const;
    double getFitness() const;
    int getAge() const;
    std::vector<double> getMorphParameters() const;
    std::vector<double> getMorphParameters2();
    std::vector<double> getCPGParameters() const;

    ////////// SETTERS //////////
    void setID(int idNum);
    void setFitness(double fit);
    void setAge(int newAge);
    void setMorphParameters(std::vector<double> mParams);
    void setCPGParameters(std::vector<double> cParams);
    void setParameters(std::vector<double> Params);
    ////////// Other Functions //////////

    //Create a random set of parameters for the morphology
    std::vector<double> setRandomMorphParams();
    //Create a random set of parameters for the cpg
    std::vector<double> setRandomCPGParams();

    //Mutations
    void mutate();
    void mutateCPGRobotParam();
    void mutateMorphRobotParam();

    //Sums the parameters (Eventually will be the result of the simulation fitness function)
    void evaluateFitness(bool show);

    //Overloaded operators
    friend std::ostream &operator << (std::ostream &out, const Robot &r);
};


#endif //GA_ROBOT_H
