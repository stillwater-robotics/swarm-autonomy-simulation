#include "common.h"
#include "controller.h"
#include "point_generator.h"
#include "trajectory_generator.h"

/**
 * @brief Simulation agent, calling Update moves the simulation forward.
 * 
 */
class Agent
{
private:
    State currentState;

    PointGenerator* pointGenerator;
    TrajectoryGenerator* trajectoryGenerator;
    Controller* controller;
    
public:
    /**
    * @brief Constructs an agent in a given state
    * 
    * @param initialState starting state of the robot
    * @param initialSwarmState starting state of the robot within the swarm
    */
    Agent(State initialState);

    /**
     * @brief Updates the states of the agent, contains code that will be
     * used on the robot.
     * 
     * @param states States the neighboring agents
     */
    void Update(std::vector<State> states); 

    // returns the current state of the agent
    State ReadState();
};