//
// Created by Jean-Baptiste Bolh on 11/4/19.
//

#include "Robot.h"

#include <iostream>
#include <random>
#include <limits>
#include <fstream>

#define PI 3.14159

//Constructors
Robot::Robot() {
    id = 1;
    fitness = 1000000;
    age = 1;
    morphParams = setRandomMorphParams();
    cpgParams = setRandomCPGParams();
}
Robot::Robot(int idNum){
    id = idNum;
    fitness = 1000000;
    age = 1;
    morphParams = setRandomMorphParams();
    cpgParams = setRandomCPGParams();
}
Robot::Robot(int idNum, std::vector<double> mParams){
    id = idNum;
    fitness = 1000000;
    age = 1;
    morphParams = mParams;
    cpgParams = setRandomCPGParams();
}
Robot::Robot(int idNum, double fit, std::vector<double> mParams, std::vector<double> cParams) {
    id = idNum;
    fitness = fit;
    age = 1;
    morphParams = mParams;
    cpgParams = cParams;
}
//Copy Constructor
Robot::Robot(const Robot &r){
    id = r.getID();
    fitness = r.getFitness();
    age = r.getAge();
    morphParams = r.getMorphParameters();
    cpgParams = r.getCPGParameters();
}


//GETTERS
int Robot::getID() const {
    return id;
}
double Robot::getFitness() const {
    return fitness;
}
int Robot::getAge() const {
    return age;
}
std::vector<double> Robot::getMorphParameters() const {
    return morphParams;
}
std::vector<double> Robot::getCPGParameters() const {
    return cpgParams;
}

//SETTERS
void Robot::setID(int idNum){
    id = idNum;
}
void Robot::setFitness(double fit){
    fitness = fit;
}
void Robot::setAge(int newAge){
    age = newAge;
}
void Robot::setMorphParameters(std::vector<double> mParams){
    morphParams = mParams;
}
void Robot::setCPGParameters(std::vector<double> cParams){
    cpgParams = cParams;
}

//Functions


//Sets parameters (17 of them)
std::vector<double> Robot::setRandomMorphParams() {
    //Generate seed and random number generator
    int seed = int(std::chrono::system_clock::now().time_since_epoch().count());
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> legLengthDistribution(0.1,0.5);
    std::uniform_real_distribution<double> springDistribution(10,100);
    std::uniform_real_distribution<double> angleDistribution(-PI/3.0,PI/3.0);

    /** Num body segments, bodseg length, bodseg mass, bodseg spring k, bodseg angle **/
    std::vector<double>bodyParams {1, 0.5, 10, 1000, 0};

    /** num front leg segs, flseg radius, length, mass, spring k, angle {2, 0.03, 0.14, 1, 0, 0, 0.03, 0.14, 1, 100, -0.523} **/
    std::vector<double>frontLegParams = {2,0.03};

    double segmentLegLength = legLengthDistribution(generator);
    frontLegParams.push_back(segmentLegLength);

    frontLegParams.push_back(1); //Mass of segments set to 1

    double segmentSpringConst = springDistribution(generator);
    frontLegParams.push_back(segmentSpringConst);

    double angleOfSeg = angleDistribution(generator);
    frontLegParams.push_back(angleOfSeg);

    frontLegParams.push_back(0.03);

    segmentLegLength = legLengthDistribution(generator);
    frontLegParams.push_back(segmentLegLength);

    frontLegParams.push_back(1); //Mass of segments set to 1

    segmentSpringConst = springDistribution(generator);
    frontLegParams.push_back(segmentSpringConst);

    angleOfSeg = angleDistribution(generator);
    frontLegParams.push_back(angleOfSeg);

    /** num back leg segs, bl seg radius, length, mass, spring k, angle {2, 0.03, 0.14, 1, 0, 0, 0.03, 0.14, 1, 100, -0.5233} **/
    std::vector<double>backLegParams {2, 0.03};

    segmentLegLength = legLengthDistribution(generator);
    backLegParams.push_back(segmentLegLength);

    backLegParams.push_back(1); //Mass of segments set to 1

    segmentSpringConst = springDistribution(generator);
    backLegParams.push_back(segmentSpringConst);

    angleOfSeg = angleDistribution(generator);
    backLegParams.push_back(angleOfSeg);

    backLegParams.push_back(0.03);

    segmentLegLength = legLengthDistribution(generator);
    backLegParams.push_back(segmentLegLength);

    backLegParams.push_back(1); //Mass of segments set to 1

    segmentSpringConst = springDistribution(generator);
    backLegParams.push_back(segmentSpringConst);

    angleOfSeg = angleDistribution(generator);
    backLegParams.push_back(angleOfSeg);

    std::vector<double>params;
    params.reserve( bodyParams.size() + frontLegParams.size() + backLegParams.size() ); // preallocate memory
    params.insert( params.end(), bodyParams.begin(), bodyParams.end() );
    params.insert( params.end(), frontLegParams.begin(), frontLegParams.end() );
    params.insert( params.end(), backLegParams.begin(), backLegParams.end() );

    return params;

}

//Sets random CPG Parameters
std::vector<double> Robot::setRandomCPGParams() {
    int seed = int(std::chrono::system_clock::now().time_since_epoch().count());
    std::default_random_engine generator (seed);
    std::vector<double> params(3);
    for(int i = 0; i < size(params); i++){
        //Set amplitude
        if (i == 0){
            std::uniform_real_distribution<double> amplitudeDistribution(0.01,1.0);
            params[i] = amplitudeDistribution(generator);
        }
        else if(i == 1){
            std::uniform_real_distribution<double> swingDistribution(2.0,15.0*PI);
            params[i] = swingDistribution(generator);
        }
        else{
            std::uniform_real_distribution<double> stanceDistribution(0.05,2);
            params[i] = stanceDistribution(generator);
        }
    }

    return params;
}

void Robot::mutate(){
    int seed = int(std::chrono::system_clock::now().time_since_epoch().count());
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> brainBodyDist(0,9);

    int mutateBrainOrBody = brainBodyDist(generator);

    //1/10 chance of mutating the body
    if(mutateBrainOrBody == 0){
        mutateMorphRobotParam();
    }
    else{
        mutateCPGRobotParam();
    }
}

//Each parameter is mutated with a std deviation of 10% of parameter range
void Robot::mutateCPGRobotParam(){
    int seed = int(std::chrono::system_clock::now().time_since_epoch().count());
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> distribution(0,2);

    int whichParamToMutate = distribution(generator);

    std::vector<double> params = getCPGParameters();

    //Amplitude
    if(whichParamToMutate == 0){
        std::normal_distribution<double> amplitudeDistribution(cpgParams[0],1.0/10.0);
        params[0] = amplitudeDistribution(generator);
        while(params[0] <= 0 || params[0] > 1){
            params[0] = amplitudeDistribution(generator);
        }
    }
    //Swing
    else if(whichParamToMutate == 1){
        std::normal_distribution<double> swingDistribution(cpgParams[1],(15.0*PI)/10.0);
        params[1] = swingDistribution(generator);
    }
    //Stance
    else {
        std::normal_distribution<double> stanceDistribution(cpgParams[2], 2.0/10.0);
        params[2] = stanceDistribution(generator);
    }

    setCPGParameters(params);

}

void Robot::mutateMorphRobotParam(){
    int seed = int(std::chrono::system_clock::now().time_since_epoch().count());
    std::default_random_engine generator (seed);
    std::uniform_int_distribution<int> distribution(0,5);

    int whichParamToMutate = distribution(generator);

    std::vector<double> params = getMorphParameters();
    //Front Leg Length 0.1 -> 0.9 params[7], params[12]
    if (whichParamToMutate == 0){
        std::uniform_int_distribution<int> whichSeg(0,1);
        int whichSegmentToChangeLength = whichSeg(generator);
        if(whichSegmentToChangeLength == 0){
            std::normal_distribution<double> lengthDistribution(params[7],0.4/10.0);
            params[7] = lengthDistribution(generator);
            while(params[7] < 0.1 || params[7] > 0.5){
                params[7] = lengthDistribution(generator);
            }
        }
        else{
            std::normal_distribution<double> lengthDistribution(params[12],0.4/10.0);
            params[12] = lengthDistribution(generator);
            while(params[12] < 0.1 || params[12] > 0.5){
                params[12] = lengthDistribution(generator);
            }
        }
    }
    //Front Leg Spring Coefficient 10 -> 1000  params[9], params[14]
    else if (whichParamToMutate == 1){
        std::uniform_int_distribution<int> whichSpring(0,1);
        int whichSpringToChange = whichSpring(generator);
        if(whichSpringToChange == 0){
            std::normal_distribution<double> springDistribution(params[9],90.0/10.0);
            params[9] = springDistribution(generator);
            while(params[9] < 10 || params[9] > 100){
                params[9] = springDistribution(generator);
            }
        }
        else{
            std::normal_distribution<double> swingDistribution(params[14],90.0/10.0);
            params[14] = swingDistribution(generator);
            while(params[14] < 10 || params[14] > 100){
                params[14] = swingDistribution(generator);
            }
        }
    }
    //Front Leg Angle -pi/3 -> pi/3 params[10], params[15]
    else if (whichParamToMutate == 2){
        std::uniform_int_distribution<int> jointDistribution(0,1);
        int whichJointToChange = jointDistribution(generator);
        if(whichJointToChange == 0){
            std::normal_distribution<double> angleDistribution(params[10],(2.0*PI/3.0)/10.0);
            params[10] = angleDistribution(generator);
            while(params[10] < -PI/3.0 || params[10] > PI/3.0){
                params[10] = angleDistribution(generator);
            }
        }
        else{
            std::normal_distribution<double> angleDistribution(params[15],(2.0*PI/3.0)/10.0);
            params[15] = angleDistribution(generator);
            while(params[15] < -PI/3.0 || params[15] > PI/3.0){
                params[15] = angleDistribution(generator);
            }
        }
    }
    //Back Leg Length 0.1 -> 0.9 params[18] and params[23]
    else if (whichParamToMutate == 3){
        std::uniform_int_distribution<int> whichSeg(0,1);
        int whichSegmentToChangeLength = whichSeg(generator);
        if(whichSegmentToChangeLength == 0){
            std::normal_distribution<double> lengthDistribution(params[18],0.4/10.0);
            params[18] = lengthDistribution(generator);
            while(params[18] < 0.1 || params[18] > 0.5){
                params[18] = lengthDistribution(generator);
            }
        }
        else{
            std::normal_distribution<double> lengthDistribution(params[23],0.4/10.0);
            params[23] = lengthDistribution(generator);
            while(params[23] < 0.1 || params[23] > 0.5){
                params[23] = lengthDistribution(generator);
            }
        }
    }
    //Back Leg Spring Coefficient 10 -> 1000 params[20], params[25]
    else if (whichParamToMutate == 4){
        std::uniform_int_distribution<int> whichSpring(0,1);
        int whichSpringToChange = whichSpring(generator);
        if(whichSpringToChange == 0){
            std::normal_distribution<double> springDistribution(params[20],90.0/10.0);
            params[20] = springDistribution(generator);
            while(params[20] < 10 || params[20] > 100){
                params[20] = springDistribution(generator);
            }
        }
        else{
            std::normal_distribution<double> springDistribution(params[25],90.0/10.0);
            params[25] = springDistribution(generator);
            while(params[25] < 10 || params[25] > 100){
                params[25] = springDistribution(generator);
            }
        }
    }
    //Back Leg Angle -pi/3 -> pi/3 params[21], params[26]
    else{
        std::uniform_int_distribution<int> jointDistribution(0,1);
        int whichJointToChange = jointDistribution(generator);
        if(whichJointToChange == 0){
            std::normal_distribution<double> angleDistribution(params[21],(2.0*PI/3.0)/10.0);
            params[21] = angleDistribution(generator);
            while(params[21] < -PI/3.0 || params[21] > PI/3.0){
                params[21] = angleDistribution(generator);
            }
        }
        else{
            std::normal_distribution<double> angleDistribution(params[26],(2.0*PI/3.0)/10.0);
            params[26] = angleDistribution(generator);
            while(params[26] < -PI/3.0 || params[26] > PI/3.0){
                params[26] = angleDistribution(generator);
            }
        }
    }

    setMorphParameters(params);
}

void Robot::evaluateFitness(bool show){
    std::string filePathString = "/Users/JeanBaptisteBolh/Documents/School/CS352/Final/ode-0.12-drawstuff-master/ode/demo/demo_buggy";

    //Create Morphological parameter string
    std::string morphParamString = "";
    for(int i = 0; i < size(morphParams); i++){
        if(i != size(morphParams)-1){
            morphParamString += std::to_string(morphParams[i]) + " ";
        }
        else{
            morphParamString += std::to_string(morphParams[i]);
        }
    }

    //Create CPG parameter string
    std::string cpgParamString = std::to_string(cpgParams[0]) + " " + std::to_string(cpgParams[1]) + " " + std::to_string(cpgParams[2]);

    //Create Camera/Window parameter string (draw or not (0, 1), window pos x, window pos y)
    std::string cameraWindowParams;
    if(show){
        cameraWindowParams += "1 200 300";
    }
    else{
        cameraWindowParams += "0 200 300";
    }

    //Combine all strings
    std::string execString = filePathString + " " + morphParamString + " " + cpgParamString + " " + cameraWindowParams;

    //Read in fitness from file
    system(execString.c_str());

    std::ifstream myfile("outputParams.txt");
    myfile>>fitness;

    //std::cout << "Robo ID: " << id << " Fitness:" << fitness << std::endl;
}

//Overloaded output operator
std::ostream &operator << (std::ostream &out, const Robot &r){
    out << "ID: " << r.getID() << "  Fitness: " << r.getFitness() << "  Age: " << r.getAge() << "  CPGParams: ";
    for(int i=0; i<size(r.getCPGParameters()); i++){
        out << r.getCPGParameters()[i] << " ";
    }

    out << " MorphParams: ";
    for(int i=0; i<size(r.getMorphParameters()); i++){
        out << r.getMorphParameters()[i] << " ";
    }

    return out;
}