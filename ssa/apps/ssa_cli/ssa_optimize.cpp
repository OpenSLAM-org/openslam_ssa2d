// Sparse Surface Optimization 2D
// Copyright (C) 2011 M. Ruhnke, R. Kuemmerle, G. Grisetti, W. Burgard
// 
// SSA2D is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// SSA2D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <iostream>
#include <signal.h>

#include "ssa/core/ssa_graph.h"
#include "ssa/core/allocate_solver.h"
#include "ssa/data_association/normal_shooting.h"

using namespace std;
using namespace ssa;

const char *message[]={
  "ssa_optimize: optimizes a given ssa file",
  "usage ssa_optimize [options] <ssa_file>",
  "options:",
  "-i [int]       number of (inner) g2o iterations with fixed data association.",
  "-n [int]       number of (outer) ssa iterations (updated data association)",
  "-o [filename]  save result in given file",
  0
};

static bool running = true;
unsigned int breakCounter = 0;

void sighandler(int sig)
{
    cout << endl << "Signal " << sig << " caught..." << endl;

   running = false;
   breakCounter++;
   if(breakCounter >= 3){
      exit(1);
   }
}


int main(int argc, char **argv)
{

  const char**v=message;
  while (*v){
    cout << *v << endl;
    v++;
  }

  // HACK reset numeric locale, we need C
  setlocale (LC_NUMERIC,"C");
  int c=1;
  const char* logfile=0;
  const char* outfile=0;
  bool saveOutput = false;
  int iterations = 3;
  int outerIterations = 1;
  double targetResolution = 0.05;
 
  while (c<argc){
    if (!strcmp(argv[c],"-debug")){
//       debug=true;
      c++;
    } else 
    if (!strcmp(argv[c],"-i")){
      c++;
      iterations=atoi(argv[c]);
      c++;
    } else
    if (!strcmp(argv[c],"-n")){
      c++;
      outerIterations=atoi(argv[c]);
      c++;
    } else  
    if (!strcmp(argv[c],"-o")){
      saveOutput = true;  
      c++;
      outfile=argv[c];
      c++;
    } else 
    if (!strcmp(argv[c],"-r")){
      c++;
      targetResolution=atof(argv[c]);
      c++;
    } else 
    if (! logfile){
      logfile=argv[c];
      c++;
      break;
    }
  }

  if(!logfile){
    cerr << "Error: no input ssa file given." << endl;
    exit(0);
  }
  if(!outfile){
    cerr << "Warning: missing output file." << endl;
  }

  //initialize optimizer
  SparseSurfaceAdjustmentGraph ssaGraph;
  ssaGraph._optimizer.setVerbose(true);
  ssaGraph._optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt); 
  g2o::BlockSolverX::LinearSolverType* linearSolver = AllocateLinearSolver<g2o::BlockSolverX>(0);
  ssaGraph._optimizer.setSolver(new g2o::BlockSolverX(&ssaGraph._optimizer, linearSolver));

  double normalNeighborhoodDistance = 0.2;
  double normalShootingStepSize = 0.01; 
  int    normalShootingSteps = 5;
  double normalShootingMaxAngleDifference = 10.0; // in degree
  if (! logfile){

  } else {
    ssaGraph.load(logfile);
  }

  double timing = get_time();
  double startTime = get_time();
  for(int i = 0; i < outerIterations; ++i){
    cerr << "Iteration " << i+1 << endl;
    cerr << "[start] recalc mean and covariances: " << endl;
    ssaGraph.calcMeanCov(normalNeighborhoodDistance);


    cerr << "Updating observation covariances took " << (get_time() - timing) * 1000 << " ms" << endl;
    //Drop current data association
    ssaGraph.dropDataAssociation();

    timing = get_time();
    NormalShooting::shootNormals(ssaGraph, normalShootingSteps, normalShootingStepSize, normalShootingMaxAngleDifference);
    cerr << "Normal shooting took " << (get_time() - timing) << " s" << endl;
    g2o::OptimizableGraph::EdgeSet eset = ssaGraph.getEdgeset();
    ssaGraph._optimizer.initializeOptimization(eset);
    ssaGraph._optimizer.optimize(iterations);
    ssaGraph.moveUnoptimizedPoints();

  }
  cerr << "calculation took " << get_time() - startTime << " seconds." << endl;
  if(saveOutput){
    ssaGraph.save(outfile);
  }

  return 0;
}
