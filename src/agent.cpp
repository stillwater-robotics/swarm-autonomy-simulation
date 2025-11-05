#include "agent.h"

Agent::Agent(State initialState){
    // TODO expose
    pointGenerator = &PointGenerator(1.0f, 1.0f, 1.0f);
    trajectoryGenerator = &TrajectoryGenerator();
    controller = &Controller();

    currentState = initialState;
}

void Agent::Update(std::vector<State> states){
    //TODO
    return;
}

State Agent::ReadState(){
    return currentState;
}