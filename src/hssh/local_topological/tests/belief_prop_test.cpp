/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "hssh/local_topological/area_detection/labeling/factor_graph.h"
#include "hssh/local_topological/area_detection/labeling/belief_prop.h"

using namespace vulcan::hssh;


int main(int argc, char** argv)
{
    // Recreate Figure 8.51 from Bishop. Assume two states for all four variables
    // Factors want the states to be different

    int id = 0;
    auto a1 = std::make_shared<Variable>(id++, 2);
    auto a2 = std::make_shared<Variable>(id++, 2);
    auto a3 = std::make_shared<Variable>(id++, 2);
    auto a4 = std::make_shared<Variable>(id++, 2);

    /////////// Factor4 ////////////////
    std::vector<FactorStateProb> f1Prob;
    FactorStateProb f1fsp1;
    f1fsp1.states.emplace_back(0, 0);
    f1fsp1.prob = 0.9;
    f1Prob.push_back(f1fsp1);

    FactorStateProb f1fsp2;
    f1fsp2.states.emplace_back(0, 1);
    f1fsp2.prob = 0.1;
    f1Prob.push_back(f1fsp2);

    std::vector<Variable*> f1Vars;
    f1Vars.push_back(a1.get());

    auto factor4 = std::make_shared<Factor>(id++, f1Vars, f1Prob);
    auto edge1 = std::make_shared<FactorEdge>(a1->id(), factor4->id(), 2);
    a1->addEdge(edge1.get());
    factor4->addEdge(edge1.get());

    /////////// Factor5 ////////////////
    std::vector<FactorStateProb> f3Prob;
    FactorStateProb f3fsp1;
    f3fsp1.states.emplace_back(2, 0);
    f3fsp1.prob = 0.9;
    f3Prob.push_back(f3fsp1);

    FactorStateProb f3fsp2;
    f3fsp2.states.emplace_back(2, 1);
    f3fsp2.prob = 0.1;
    f3Prob.push_back(f3fsp2);

    std::vector<Variable*> f3Vars;
    f3Vars.push_back(a3.get());

    auto factor5 = std::make_shared<Factor>(id++, f3Vars, f3Prob);
    auto edge3 = std::make_shared<FactorEdge>(a3->id(), factor5->id(), 2);
    a3->addEdge(edge3.get());
    factor5->addEdge(edge3.get());

    /////////// Factor6 ////////////////
    std::vector<FactorStateProb> f4Prob;
    FactorStateProb a4fsp1;
    a4fsp1.states.emplace_back(3, 0);
    a4fsp1.prob = 0.9;
    f4Prob.push_back(a4fsp1);

    FactorStateProb a4fsp2;
    a4fsp2.states.emplace_back(3, 1);
    a4fsp2.prob = 0.1;
    f4Prob.push_back(a4fsp2);

    std::vector<Variable*> f4Vars;
    f4Vars.push_back(a4.get());

    auto factor6 = std::make_shared<Factor>(id++, f4Vars, f4Prob);
    auto edge4 = std::make_shared<FactorEdge>(a4->id(), factor6->id(), 2);
    a4->addEdge(edge4.get());
    factor6->addEdge(edge4.get());

    /////////// Factor7 ////////////////
    std::vector<FactorStateProb> f5Prob;
    FactorStateProb f5fsp1;
    f5fsp1.states.emplace_back(0, 0);
    f5fsp1.states.emplace_back(1, 0);
    f5fsp1.prob = 0.1;
    f5Prob.push_back(f5fsp1);

    FactorStateProb f5fsp2;
    f5fsp2.states.emplace_back(0, 0);
    f5fsp2.states.emplace_back(1, 1);
    f5fsp2.prob = 0.9;
    f5Prob.push_back(f5fsp2);

    FactorStateProb f5fsp3;
    f5fsp3.states.emplace_back(0, 1);
    f5fsp3.states.emplace_back(1, 0);
    f5fsp3.prob = 0.9;
    f5Prob.push_back(f5fsp3);

    FactorStateProb f5fsp4;
    f5fsp4.states.emplace_back(0, 1);
    f5fsp4.states.emplace_back(1, 1);
    f5fsp4.prob = 0.1;
    f5Prob.push_back(f5fsp4);

    std::vector<Variable*> f5Vars;
    f5Vars.push_back(a1.get());
    f5Vars.push_back(a2.get());

    auto factor7 = std::make_shared<Factor>(id++, f5Vars, f5Prob);
    auto edge5 = std::make_shared<FactorEdge>(a1->id(), factor7->id(), 2);
    auto edge6 = std::make_shared<FactorEdge>(a2->id(), factor7->id(), 2);
    a1->addEdge(edge5.get());
    a2->addEdge(edge6.get());
    factor7->addEdge(edge5.get());
    factor7->addEdge(edge6.get());

    /////////// Factor8 ////////////////
    std::vector<FactorStateProb> f6Prob;
    FactorStateProb f6fsp1;
    f6fsp1.states.emplace_back(2, 0);
    f6fsp1.states.emplace_back(1, 0);
    f6fsp1.prob = 0.1;
    f6Prob.push_back(f6fsp1);

    FactorStateProb f6fsp2;
    f6fsp2.states.emplace_back(2, 0);
    f6fsp2.states.emplace_back(1, 1);
    f6fsp2.prob = 0.9;
    f6Prob.push_back(f6fsp2);

    FactorStateProb f6fsp3;
    f6fsp3.states.emplace_back(2, 1);
    f6fsp3.states.emplace_back(1, 0);
    f6fsp3.prob = 0.9;
    f6Prob.push_back(f6fsp3);

    FactorStateProb f6fsp4;
    f6fsp4.states.emplace_back(2, 1);
    f6fsp4.states.emplace_back(1, 1);
    f6fsp4.prob = 0.1;
    f6Prob.push_back(f6fsp4);

    std::vector<Variable*> f6Vars;
    f6Vars.push_back(a3.get());
    f6Vars.push_back(a2.get());

    auto factor8 = std::make_shared<Factor>(id++, f6Vars, f6Prob);
    auto edge7 = std::make_shared<FactorEdge>(a3->id(), factor8->id(), 2);
    auto edge8 = std::make_shared<FactorEdge>(a2->id(), factor8->id(), 2);
    a3->addEdge(edge7.get());
    a2->addEdge(edge8.get());
    factor8->addEdge(edge7.get());
    factor8->addEdge(edge8.get());

    /////////// Factor9 ////////////////
    std::vector<FactorStateProb> f7Prob;
    FactorStateProb f7fsp1;
    f7fsp1.states.emplace_back(3, 0);
    f7fsp1.states.emplace_back(1, 0);
    f7fsp1.prob = 0.1;
    f7Prob.push_back(f7fsp1);

    FactorStateProb f7fsp2;
    f7fsp2.states.emplace_back(3, 0);
    f7fsp2.states.emplace_back(1, 1);
    f7fsp2.prob = 0.9;
    f7Prob.push_back(f7fsp2);

    FactorStateProb f7fsp3;
    f7fsp3.states.emplace_back(3, 1);
    f7fsp3.states.emplace_back(1, 0);
    f7fsp3.prob = 0.9;
    f7Prob.push_back(f7fsp3);

    FactorStateProb f7fsp4;
    f7fsp4.states.emplace_back(3, 1);
    f7fsp4.states.emplace_back(1, 1);
    f7fsp4.prob = 0.1;
    f7Prob.push_back(f7fsp4);

    std::vector<Variable*> f7Vars;
    f7Vars.push_back(a4.get());
    f7Vars.push_back(a2.get());

    auto factor9 = std::make_shared<Factor>(id++, f7Vars, f7Prob);
    auto edge9 = std::make_shared<FactorEdge>(a4->id(), factor9->id(), 2);
    auto edge10 = std::make_shared<FactorEdge>(a2->id(), factor9->id(), 2);
    a4->addEdge(edge9.get());
    a2->addEdge(edge10.get());
    factor9->addEdge(edge9.get());
    factor9->addEdge(edge10.get());

    std::vector<FactorEdge::Ptr> edges;
    edges.push_back(edge1);
    edges.push_back(edge3);
    edges.push_back(edge4);
    edges.push_back(edge5);
    edges.push_back(edge6);
    edges.push_back(edge7);
    edges.push_back(edge8);
    edges.push_back(edge9);
    edges.push_back(edge10);

    std::vector<Factor::Ptr> factors;
    factors.push_back(factor4);
    factors.push_back(factor5);
    factors.push_back(factor6);
    factors.push_back(factor7);
    factors.push_back(factor8);
    factors.push_back(factor9);

    std::vector<Variable::Ptr> variables;
    variables.push_back(a1);
    variables.push_back(a2);
    variables.push_back(a3);
    variables.push_back(a4);

    FactorGraph graph(variables, factors, edges);
    belief_propagation(graph, UpdateType::sum);

    for(auto v : variables)
    {
        std::cout << "Marginal " << v->id() << ":\n" << v->marginal();
    }

    return 0;
}
