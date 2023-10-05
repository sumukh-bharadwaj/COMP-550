///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

// Your random tree planner
#include "RTP.h"

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>

#include <ompl/base/samplers/UniformValidStateSampler.h>

using namespace ompl;

void benchmarkHome(std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    benchmark_name = std::string("Home");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Home_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(252.95);
    start->setY(-214.95);
    start->setZ(46.19);
    start->rotation().setAxisAngle(0., 0., -1., 3.12413936107);

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(-320.00);
    goal->setY(75.05);
    goal->setZ(46.19);
    goal->rotation().setAxisAngle(0., 0., -1., 3.12413936107);

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 325.00);
    bounds.setHigh(1, 337.89);
    bounds.setHigh(2, 146.19);
    bounds.setLow(0, -383.80);
    bounds.setLow(1, -371.47);
    bounds.setLow(2, -0.20);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 30.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 50;
}

void benchmarkApartment(std::string &benchmark_name, app::SE3RigidBodyPlanning &setup, double &runtime_limit,
                double &memory_limit, int &run_count)
{
    benchmark_name = std::string("Apartment");
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Apartment_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(270.);
    start->setY(160.);
    start->setZ(-200.);
    start->rotation().setIdentity();

    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(270.);
    goal->setY(160.);
    goal->setZ(-400.);
    goal->rotation().setIdentity();

    base::RealVectorBounds bounds(3);
    bounds.setHigh(0, 350.);
    bounds.setHigh(1, 250.);
    bounds.setHigh(2, -150.);
    bounds.setLow(0, 200.);
    bounds.setLow(1, 75.);
    bounds.setLow(2, -450.);
    setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    setup.setStartAndGoalStates(start, goal);
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    runtime_limit = 120.0;
    memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    run_count = 50;
}

int main()
{
    app::SE3RigidBodyPlanning setup;
    std::string benchmark_name;
    double runtime_limit, memory_limit;
    int run_count;

    int environment;

    do
    {
        std::cout << "Benchmark for: " << std::endl;
        std::cout << " (1) Apartment" << std::endl;
        std::cout << " (2) Home" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            benchmarkApartment(benchmark_name, setup, runtime_limit, memory_limit, run_count);
            break;
        case 2:
            benchmarkHome(benchmark_name, setup, runtime_limit, memory_limit, run_count);
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    // create the benchmark object and add all the planners we'd like to run
    tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    tools::Benchmark b(setup, benchmark_name);

    auto rrt3 = std::make_shared<geometric::RRT>(setup.getSpaceInformation());
    rrt3->setName("RRT Range 50.");
    rrt3->setRange(50.);

    auto rrt2 = std::make_shared<geometric::RRT>(setup.getSpaceInformation());
    rrt2->setName("RRT Range 5.");
    rrt2->setRange(5.);

    b.addPlanner(std::make_shared<geometric::EST>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::PRM>(setup.getSpaceInformation()));
    b.addPlanner(std::make_shared<geometric::RTP>(setup.getSpaceInformation()));
    b.addPlanner(rrt2);
    b.addPlanner(rrt3);

    b.benchmark(request);
    b.saveResultsToFile();

    return 0;
}
