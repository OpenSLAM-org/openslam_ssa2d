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
#include <QApplication>
#include <QtGui/QMainWindow>
#include "ui_base_main_window.h"

#include "ssa/core/allocate_solver.h"
#include "ssa/data_association/normal_shooting.h"

using namespace std;
using namespace ssa;

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
    signal(SIGINT, &sighandler);
    QApplication qapp(argc, argv);
    // HACK reset numeric locale, we need C
    setlocale (LC_NUMERIC,"C");
  bool debug = false;
  bool restoreViewerState = false;
  const char* logfile=0;
  const char* viewerStateFile=0;
  int c=1;
  while (c<argc){
    if (!strcmp(argv[c],"-debug")){
      debug=true;
      c++;
    } else 
    if (!strcmp(argv[c],"-cam")){
      restoreViewerState=true;
      c++;
      viewerStateFile = argv[c];
      c++;
    } else 
    if (! logfile){
      logfile=argv[c];
      c++;
      break;
    }
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

    QMainWindow* mw = new QMainWindow;
    Ui_MainWindow umw;
    umw.setupUi(mw);

    umw.ssaGLWidget->setMap(&ssaGraph);
    mw->show();
    umw.ssaGLWidget->setBackgroundColor(Qt::white);
    if(restoreViewerState){
      umw.ssaGLWidget->setStateFileName(viewerStateFile);
      umw.ssaGLWidget->restoreStateFromFile();
    }

    QObject::connect( &(ssaGraph._optimizer), SIGNAL(iterationDone()), umw.ssaGLWidget, SLOT(updateGL()));
    QObject::connect( &(ssaGraph._optimizer), SIGNAL(iterationDone()), umw.ssaGLWidget, SLOT(saveSnapshotVideo()));
    bool needReDraw = true;
    double timing = get_time();

    while(mw->isVisible() && running) {
       qapp.processEvents();
       if(umw.ssaGLWidget->_optimize){
          umw.ssaGLWidget->_optimize = false;

        timing = get_time();
        double startTime = get_time();
        umw.ssaGLWidget->saveSnapshotVideo();
        for(unsigned int i = 0; i < umw.ssaGLWidget->_iterations; ++i){
          cerr << "Iteration " << i+1 << endl;
          umw.stateLabel->setText(QString("running"));
          QPalette palette;
          palette.setColor(umw.stateLabel->backgroundRole(), Qt::yellow);
          umw.stateLabel->setPalette(palette);
          umw.stateLabel->setAutoFillBackground(true);
          mw->repaint();
          qapp.processEvents();
          timing = get_time();
          ssaGraph.calcMeanCov(normalNeighborhoodDistance);
          cerr << "Updating observation covariances took " << (get_time() - timing) * 1000 << " ms" << endl;
          //Drop current data association
          ssaGraph.dropDataAssociation();

          timing = get_time();
          NormalShooting::shootNormals(ssaGraph, normalShootingSteps, normalShootingStepSize, normalShootingMaxAngleDifference);
          cerr << "Normal shooting took " << (get_time() - timing) << " s" << endl;

          umw.stateLabel->setPalette(palette);
          mw->repaint();
          qapp.processEvents();
          g2o::OptimizableGraph::EdgeSet eset = ssaGraph.getEdgeset();
          ssaGraph._optimizer.initializeOptimization(eset);
          ssaGraph._optimizer.optimize(6);
          ssaGraph.moveUnoptimizedPoints();

          umw.stateLabel->setText(QString("ready"));
          palette.setColor(umw.stateLabel->backgroundRole(), Qt::green);
          umw.stateLabel->setPalette(palette);

          umw.stateLabel->setAutoFillBackground(true);
          needReDraw = true;
      
        }
        cerr << "calculation took " << get_time() - startTime << " seconds." << endl;
       }

       if(needReDraw){
         umw.ssaGLWidget->updateGL();
         needReDraw = false;
       }
       usleep(1000);
    }
    return 0;
}
