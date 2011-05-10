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

#include "ssa/core/ssa_graph.h"
#include <QGLViewer/qglviewer.h>

namespace ssa {
  using namespace Eigen;
  using namespace std;

class SSAQGLViewerWidget: public QGLViewer
{
  Q_OBJECT
  public:
    SSAQGLViewerWidget();
    SSAQGLViewerWidget(QWidget*& widget);
    ~SSAQGLViewerWidget();

  inline void setInitialParams(){
    _backgroundHeight = -0.01;
    _height = 0.01;
    _landmarkSize = 0.01;
    _poseSize = 0.2;
    _optimize = false;
    _iterations = 10;
    _DAmaxdist = 0.2;
    _pointSize = 0.01;
    _drawRobotPoses = true;
    _drawObservationPoints = true;
    _drawVertexPointCovariance = false;
    _drawObservationCovariance = false;
    _drawEdgeOdometry = true;
    _drawEdgeObservation = false;
    _drawEdgeCorrespondences = false;
    _drawGrid = false;
    //setShortcut(SAVE_SCREENSHOT, Qt::Key_S);
    setShortcut(EXIT_VIEWER, Qt::Key_Q);
    setSnapshotCounter(0);
  }


  void draw();
  virtual void drawWithNames();

  ///draw point observation verticies
  void drawVertexPoints();

  ///draw covariances of point observation verticies
  void drawVertexPointsCovariance();

  ///draw odometry edges
  void drawOdometryEdges();

  ///draw covariances of observation edges
  void drawObservationEdgeCovariance();

  ///draw data association edges
  void drawDataAssociationEdges();

  /// Basic draw primitives
  /** draws circle with radius r at the current pose */
  void drawCircle(double r);
  void drawEllipse(double l1, double l2);
  void drawCovariance(Matrix2d& cov, double angle);

  void drawBox(double size);
  void drawBox(double& x, double& y, double size);

  void drawPoseCircle(double& x, double& y, double& theta);
  void drawNode(Vector2d& pose);
  void drawGrid();

  void postSelection(const QPoint& point);

  inline void setMap(SparseSurfaceAdjustmentGraph* accurateMap){
    _ssaGraph = accurateMap;
  }

  public slots:
      void setOptimize();
      inline void setIterations(int it){ _iterations = it;};
      inline void setDAMaxDist(double i){ _DAmaxdist = i;};
      inline void setPointSize(double i){ _pointSize = i; updateGL(); cerr << "pointSize " << _pointSize << endl;};
      void openFile();
      void saveFile();
      inline void setDrawGrid(int i){ _drawGrid = (bool) i; updateGL();}
      inline void setDrawRobotPoses(int i){ _drawRobotPoses = (bool) i; updateGL();}
      inline void setDrawObservationPoints(int i){ _drawObservationPoints = (bool) i; updateGL();}
      inline void setDrawObservationPointsCovariances(int i){ _drawVertexPointCovariance = (bool) i; updateGL();}
      inline void setDrawObservationCovariances(int i){ _drawObservationCovariance = (bool) i; updateGL();}
      inline void setDrawEdgeOdometry(int i){ _drawEdgeOdometry = (bool) i; updateGL();}
      inline void setDrawEdgeObservation(int i){ _drawEdgeObservation = (bool) i; updateGL();}
      inline void setDrawEdgeCorrespondence(int i){ _drawEdgeCorrespondences = (bool) i; updateGL();}
      void saveFMap();
      void saveSnapshotCustom();
      void saveSnapshotVideo();

      //inline void setStrategie(int i){ _ssaGraph->_strategie = (AISNavigation::DataAssociationStrategie) i; }
  public:
  QWidget*               _parent;
  SparseSurfaceAdjustmentGraph*  _ssaGraph;

  bool _optimize;
  bool _drawGrid;
  bool _drawRobotPoses;
  bool _drawObservationPoints;
  bool _drawVertexPointCovariance;
  bool _drawObservationCovariance;
  bool _drawEdgeOdometry;
  bool _drawEdgeObservation;
  bool _drawEdgeCorrespondences;

  unsigned int _iterations;
  double _DAmaxdist;

  double _backgroundHeight;
  double _height;
  double _landmarkSize;
  double _poseSize;
  double _pointSize;
  qglviewer::Vec _selectedPoint;
};

}