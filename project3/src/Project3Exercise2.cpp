///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <cmath>
#include <functional>

// The collision checker routines
#include "CollisionChecking.h"

#include <ompl/geometric/SimpleSetup.h>
// Except for the state space definitions and any planners
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

// Your random tree planner
#include "RTP.h"

void planPoint(const std::vector<Rectangle> &obstacles)
{
    // TODO: Use your implementation of RTP to plan for a point robot.
    // Step 1) Create the state (configuration) space for your system
    // For a robot that can translate in the plane, we can use R^2 directly
    // We also need to set bounds on R^2
    ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(2));

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-2); // x and y have a minimum of -2
    bounds.setHigh(2); // x and y have a maximum of 2

    // Cast the r2 pointer to the derived type, then set the bounds
    r2->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    // Step 2) Create the SimpleSetup container for the motion planning problem.
    // this container ensures that everything is initialized properly before
    // trying to solve the motion planning problem.
    // ALWAYS USE SIMPLE SETUP!  There is no loss of functionality when using
    // this class.
    ompl::geometric::SimpleSetup ss(r2);

    // Step 3) Setup the StateValidityChecker
    // This is a function that takes a state and returns whether the state is a
    // valid configuration of the system or not.  For geometric planning, this
    // is a collision checker.
    // Note, we are "binding" the obstacles to the state validity checker. The
    // _1 notation is from std::placeholders and indicates "the first argument"
    // to the function pointer.
    ss.setStateValidityChecker(std::bind(isValidStatePoint, std::placeholders::_1, obstacles));

    // Step 4) Specify the start and goal states
    // ScopedState creates the correct state instance for the state space
    ompl::base::ScopedState<> start(r2);
    start[0] = -0.9;
    start[1] = -0.9;

    ompl::base::ScopedState<> goal(r2);
    goal[0] = 0.9;
    goal[1] = 0.9;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Step 5) (optional) Specify a planning algorithm to use
    //ompl::base::PlannerPtr planner(new ompl::geometric::PRM(ss.getSpaceInformation()));
    ompl::base::PlannerPtr planner(new ompl::geometric::RTP(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(5.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
        ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric& path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path1.txt");
        if (fout.is_open()) {
            // Perform file operations here
            fout << "R2" << std::endl;
            path.printAsMatrix(fout);
            fout.close();
            // Remember to close the file stream when you are done
        } 
        else {
            std::cerr << "Error: Unable to open the file." << std::endl;
        }

    }
    else
        std::cout << "No solution found" << std::endl;
    
}

void planBox(const std::vector<Rectangle> &obstacles)
{
    // TODO: Use your implementation of RTP to plan for a rotating square robot.
    ompl::base::StateSpacePtr se2;

    ompl::base::StateSpacePtr r2(new ompl::base::RealVectorStateSpace(2));
    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-2); // x and y have a minimum of -2
    bounds.setHigh(2); // x and y have a maximum of 2

    // Cast the r2 pointer to the derived type, then set the bounds
    r2->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    ompl::base::StateSpacePtr so2(new ompl::base::SO2StateSpace());

    se2 = r2 + so2;

    // Step 2) Create the SimpleSetup container for the motion planning problem.
    // this container ensures that everything is initialized properly before
    // trying to solve the motion planning problem using OMPL.
    // ALWAYS USE SIMPLE SETUP!  There is no loss of functionality when using
    // this class.
    ompl::geometric::SimpleSetup ss(se2);

    // Step 3) Setup the StateValidityChecker
    // This is a function that takes a state and returns whether the state is a
    // valid configuration of the system or not.  For geometric planning, this
    // is a collision checker

    // Note, we are "binding" the side length, 0.1, and the obstacles to the
    // state validity checker. The _1 notation is from std::placeholders and
    // indicates "the first argument" to the function pointer.
    ss.setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, 0.1, obstacles));

    // Step 4) Specify the start and goal states
    // ScopedState creates the correct state instance for the state space
    // You can index into the components of the state easily with ScopedState
    // The indexes correspond to the order that the StateSpace components were
    // added into the StateSpace
    ompl::base::ScopedState<> start(se2);
    start[0] = -0.9;
    start[1] = -0.9;
    start[2] = 0.0;

    ompl::base::ScopedState<> goal(se2);
    goal[0] = 0.9;
    goal[1] = 0.9;
    goal[2] = 0.0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // Step 5) (optional) Specify a planning algorithm to use
    ompl::base::PlannerPtr planner(new ompl::geometric::RTP(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(5.0);

    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
        ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric& path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path2.txt");
        if (fout.is_open()) {
            // Perform file operations here
            fout << "R2" << std::endl;
            path.printAsMatrix(fout);
            fout.close();
            // Remember to close the file stream when you are done
        }
        else {
            std::cerr << "Error: Unable to open the file." << std::endl;
        }
    }
    else
        std::cout << "No solution found" << std::endl;
}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your first environment.
    Rectangle obstacle;
    obstacle.x = -0.7;
    obstacle.y = 0.0;
    obstacle.width = 0.1;
    obstacle.height = 2.0;
    obstacles.push_back(obstacle);

    Rectangle obstacle2;
    obstacle2.x = -0.3;
    obstacle2.y = -2.0;
    obstacle2.width = 0.1;
    obstacle2.height = 2.0;
    obstacles.push_back(obstacle2);

    Rectangle obstacle3;
    obstacle3.x = 0.1;
    obstacle3.y = 0.0;
    obstacle3.width = 0.1;
    obstacle3.height = 2.0;
    obstacles.push_back(obstacle3);

    Rectangle obstacle4;
    obstacle4.x = 0.5;
    obstacle4.y = -2.0;
    obstacle4.width = 0.1;
    obstacle4.height = 2.0;
    obstacles.push_back(obstacle4);

}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your second environment.
    Rectangle obstacle;
    obstacle.x = -2.0;
    obstacle.y = -1.3;
    obstacle.width = 2.9;
    obstacle.height = 0.1;
    obstacles.push_back(obstacle);

    Rectangle obstacle2;
    obstacle2.x = 0.8;
    obstacle2.y = -1.2;
    obstacle2.width = 0.1;
    obstacle2.height = 1.5;
    obstacles.push_back(obstacle2);

    Rectangle obstacle3;
    obstacle3.x = 0.0;
    obstacle3.y = 0.3;
    obstacle3.width = 0.9;
    obstacle3.height = 0.1;
    obstacles.push_back(obstacle3);

    Rectangle obstacle4;
    obstacle4.x = -0.9;
    obstacle4.y = 1.2;
    obstacle4.width = 2.9;
    obstacle4.height = 0.1;
    obstacles.push_back(obstacle4);

    Rectangle obstacle5;
    obstacle5.x = -0.9;
    obstacle5.y = -0.3;
    obstacle5.width = 0.1;
    obstacle5.height = 1.5;
    obstacles.push_back(obstacle5);

    Rectangle obstacle6;
    obstacle6.x = -0.9;
    obstacle6.y = -0.4;
    obstacle6.width = 0.9;
    obstacle6.height = 0.1;
    obstacles.push_back(obstacle6);

}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) TODO" << std::endl;
        std::cout << " (2) TODO" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
