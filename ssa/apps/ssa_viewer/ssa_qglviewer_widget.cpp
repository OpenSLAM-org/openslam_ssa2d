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

#include "ssa_qglviewer_widget.h"
#include <QFileDialog>

namespace ssa {

SSAQGLViewerWidget::SSAQGLViewerWidget(){
  setInitialParams();
}

SSAQGLViewerWidget::SSAQGLViewerWidget(QWidget*& widget){
  _parent = widget;
  setInitialParams(); 
}

SSAQGLViewerWidget::~SSAQGLViewerWidget(){
}

void SSAQGLViewerWidget::draw(){
  if(!_ssaGraph)
    return;

  glDisable(GL_LIGHTING); 
    if(_drawGrid){
      drawGrid();
    }
  // Draw Vertex2 Nodes
    for (OptimizableGraph::VertexIDMap::const_iterator it=_ssaGraph->_optimizer.vertices().begin(); it!=_ssaGraph->_optimizer.vertices().end(); it++){
      VertexSE2* v=dynamic_cast<VertexSE2*>(it->second);
      if(v){
        if(selectedName() == v->id()){
            glColor3f(1.0, 0.0, 0.0);
          } else{
            glColor3f(0.0, 0.0, 1.0);
          }
          if(_drawRobotPoses || selectedName() == v->id()){
            drawPoseCircle(v->estimate()[0],v->estimate()[1], v->estimate()[2]);
          }
      }
    }

  if(_drawRobotPoses)
    drawOdometryEdges();

  ///draw point verticies
  drawVertexPoints();

  if(_drawVertexPointCovariance)
    drawVertexPointsCovariance();

  ///draw current data association
  if(_drawEdgeCorrespondences)
    drawDataAssociationEdges();

  glColor3f(0.0, 0.0, 0.0);
  if(_drawObservationCovariance)
    drawObservationEdgeCovariance();

    glEnable(GL_LIGHTING); 
}

void SSAQGLViewerWidget::drawVertexPoints(){
   double normalDrawLenght = 0.05;

   glPointSize(_pointSize * 100);
   glColor3f(0.0, 0.0, 0.0);
   for (size_t i = 0; i < _ssaGraph->_verticies_points.size(); ++i){
      VertexPointXYCov* v= _ssaGraph->_verticies_points[i];
      if(v){
        if(selectedName() == v->parentVertex()->id()){
          glColor3f(0.0, 0.0, 0.0);

          //draw normal
          glPushMatrix();
            glTranslatef(v->estimate()(0), v->estimate()(1), 0.0);
            glBegin(GL_LINES);
              glVertex3f(0.0, 0.0, _height);
              glVertex3f(v->normal()(0) * normalDrawLenght,v->normal()(1) * normalDrawLenght, _height);
              //glVertex3f(v->globalNormal()(0) * normalDrawLenght,v->globalNormal()(1) * normalDrawLenght, _height);
            glEnd();
          glPopMatrix();

        } else{
          glColor3f(0.0, 0.0, 0.0);
          if(!_drawObservationPoints){
            continue;
          }
        }

        if(v->covariance() == Eigen::Matrix2d::Identity())
          continue;
          //glColor3f(0.0, 0.0, 1.0);
        //draw point
        glPushMatrix();
          glBegin(GL_POINTS);
            glVertex3f(v->estimate()(0),v->estimate()(1), _height);
          glEnd();
        glPopMatrix();
      }
  }
}

void SSAQGLViewerWidget::drawOdometryEdges(){
    glColor3f(0.0, 0.0, 0.0);
    // Draw odometry edges
    for(unsigned int i = 0; i < _ssaGraph->_edges_odometry.size(); i++){
      EdgeSE2*& e = _ssaGraph->_edges_odometry[i];

      VertexSE2* from = dynamic_cast<VertexSE2*>(e->vertices()[0]);
      VertexSE2* to = dynamic_cast<VertexSE2*>(e->vertices()[1]);

      glBegin(GL_LINES);
        glVertex3f(from->estimate()[0], from->estimate()[1], _height);
        glVertex3f(to->estimate()[0], to->estimate()[1], _height);
      glEnd();  
    }
}

void SSAQGLViewerWidget::drawObservationEdgeCovariance(){
  for(unsigned int i = 0; i < _ssaGraph->_edges_observations.size(); i++){
    EdgeSE2PointXYCov*& edge = _ssaGraph->_edges_observations[i];
    VertexSE2* from = dynamic_cast<VertexSE2*>(edge->vertices()[0]);
    VertexPointXYCov* to = dynamic_cast<VertexPointXYCov*>(edge->vertices()[1]);
    if(selectedName() == from->id()){
      glPushMatrix();
        glTranslatef(to->estimate()[0], to->estimate()[1], 0.0);
        Matrix2d covariance = edge->information().inverse();
        //Rotate in global frame
        glColor3f(0.0, 0.0, 1.0);
        drawCovariance(covariance, from->estimate()[2]);
      glPopMatrix();
    }
  }
}

void SSAQGLViewerWidget::drawVertexPointsCovariance(){
  for(unsigned int i = 0; i < _ssaGraph->_verticies_points.size(); i++){
    VertexPointXYCov*& vertex = _ssaGraph->_verticies_points[i];
    if(selectedName() == vertex->parentVertex()->id() && vertex->covariance() != Eigen::Matrix2d::Identity()){
      glPushMatrix();
        glTranslatef(vertex->estimate()[0], vertex->estimate()[1], 0.0);
        //Matrix2d covariance = edge->information().inverse();
        //Rotate in global frame
        glScalef(0.05, 0.05, 0.05);
        glColor3f(0.0, 1.0, 0.0);
        drawCovariance(vertex->covariance(), 0.0);
      glPopMatrix();
    }
  }
}

void SSAQGLViewerWidget::drawDataAssociationEdges(){
    glColor3f(1.0, 0.0, 0.0);
    // Draw data association edges
    for(unsigned int i = 0; i < _ssaGraph->_edges_data_association.size(); i++){
      EdgePointXYCovPointXYCov*& e = _ssaGraph->_edges_data_association[i];

      VertexPointXYCov* from = dynamic_cast<VertexPointXYCov*>(e->vertices()[0]);
      VertexPointXYCov* to = dynamic_cast<VertexPointXYCov*>(e->vertices()[1]);  

      glBegin(GL_LINES);
        glVertex3f(from->estimate()[0], from->estimate()[1], _height);
        glVertex3f(to->estimate()[0], to->estimate()[1], _height);
      glEnd();  
    }
}

void SSAQGLViewerWidget::drawWithNames(){

  glColor3f(0.0, 0.0, 1.0);
  // Draw Vertex2 Nodes
  for (OptimizableGraph::VertexIDMap::const_iterator it=_ssaGraph->_optimizer.vertices().begin(); it!=_ssaGraph->_optimizer.vertices().end(); it++){
    VertexSE2* v=dynamic_cast<VertexSE2*>(it->second);
    if(v){
      glPushName(v->id());
        drawBox(v->estimate()[0],v->estimate()[1], _poseSize);
      glPopName();
    }
  }

}

void SSAQGLViewerWidget::drawPoseCircle(double& x, double& y, double& theta){
  glPushMatrix();
    glTranslatef(x, y, _height);
    drawCircle(_poseSize);
    float dx,dy;
    dx = _poseSize * cos(theta);
    dy = _poseSize * sin(theta);
    glBegin(GL_LINES);
      glVertex3f(0.0, 0.0, _height);
      glVertex3f(dx, dy, _height);
    glEnd();
  glPopMatrix();
}

void SSAQGLViewerWidget::drawCircle(double r = 1.0){
  float x,y;
  glBegin(GL_LINE_LOOP);
  for(int i = -180; i < 180; i += 5){
    x = r * cos(DEG2RAD(i));
    y = r * sin(DEG2RAD(i));
    glVertex3f(x, y, _height);
  }
  glEnd();
}

void SSAQGLViewerWidget::drawEllipse(double l1, double l2){
  float x,y;
  glBegin(GL_LINE_LOOP);
    for(double i = -M_PI; i <= M_PI; i +=.1){
      x = l1*sin(i);
      y = l2*cos(i);
      glVertex3f(x,y,_height);
    }
  glEnd();
}
 

void SSAQGLViewerWidget::drawCovariance(Matrix2d& cov, double angle = 0.0){
  const double& a = cov(0,0); 
  const double& b = cov(0,1); 
  const double& d = cov(1,1);
  /* get eigen-values */
  double D = a*d - b*b; // determinant of the matrix
  double T = a+d;       // Trace of the matrix
  double h = sqrt(0.25*(T*T) - D);
  double lambda1 = 0.5*T + h;  // solving characteristic polynom using p-q-formula
  double lambda2 = 0.5*T - h;
  double theta     = 0.5 * atan2(2.0 * b, a - d);
  lambda1 = 3.0 * sqrt(lambda1); //3.0
  lambda2 = 3.0 * sqrt(lambda2);

  glRotatef(RAD2DEG(theta + angle), 0.0, 0.0, 1.0);
  drawEllipse(lambda1, lambda2);
}

void SSAQGLViewerWidget::drawBox(double size = 1.0){
  glBegin(GL_QUADS);
    glVertex3f(-size, -size, _height);
    glVertex3f(size, -size, _height);
    glVertex3f(size, size, _height);
    glVertex3f(-size, size, _height);
  glEnd();
}

void SSAQGLViewerWidget::drawBox(double& x, double& y, double size = 1.0){
  glBegin(GL_QUADS);
    glVertex3f(x-size, y-size, _height);
    glVertex3f(x+size, y-size, _height);
    glVertex3f(x+size, y+size, _height);
    glVertex3f(x+-size, y+size, _height);
  glEnd();
}


void SSAQGLViewerWidget::drawGrid(){
  glColor3f(0.7, 0.7, 0.7);
  int size = 100;
  glBegin(GL_LINES);
    for(int i = -size; i < size; ++i){
      glVertex3f(i,-size,_height);
      glVertex3f(i,size,_height);
      glVertex3f(-size, i,_height);
      glVertex3f(size, i,_height);
    }
  glEnd();

  glColor3f(0.8, 0.8, 0.9);
  size = 1000;
  glBegin(GL_LINES);
    for(int i = -size; i < size; ++i){
      glVertex3f(0.1 * i,0.1 * -size,_height);
      glVertex3f(0.1 * i,0.1 *size,_height);
      glVertex3f(0.1 * -size,0.1 * i,_height);
      glVertex3f(0.1 * size,0.1 * i,_height);
    }
  glEnd();

}

void SSAQGLViewerWidget::postSelection(const QPoint& point)
{
  if (selectedName() != -1) {
    qglviewer::Vec orig;
    qglviewer::Vec dir;

    // Compute orig and dir, used to draw a representation of the intersecting line
    camera()->convertClickToLine(point, orig, dir);
    bool found;
    _selectedPoint = camera()->pointUnderPixel(point, found);
    cerr << "Selected Vertex " << selectedName() << endl;
  }
}

void SSAQGLViewerWidget::setOptimize(){
 _optimize = true;
}

void SSAQGLViewerWidget::openFile(){
  QString fileName = QFileDialog::getOpenFileName(this,
     tr("Open Graph"), ".", tr("Graph Files (*.ssa *.graph *.ssa2d)"));

  if(fileName.size() > 0 && _ssaGraph->_optimizer.vertices().size() == 0){
   ifstream graph(fileName.toStdString().c_str());
    if (! graph ){
      cerr << "error in loading the graph" << endl;
    }
    _ssaGraph->_optimizer.load(graph);
    _ssaGraph->linkNodesToVertices();
    cerr << "loaded ssa graph with " << _ssaGraph->_optimizer.vertices().size() << " vertices and " << _ssaGraph->_optimizer.edges().size() << " edges." << std::endl; 
  }
}

void SSAQGLViewerWidget::saveFile(){
 QString fileName = QFileDialog::getSaveFileName(this, tr("Save Graph"),
                            ".",
                            tr("Graph Files (*.ssa *.graph *.ssa2d)"));
  if(fileName.size() > 0){
    _ssaGraph->_optimizer.save(fileName.toStdString().c_str());
  }
}


void SSAQGLViewerWidget::saveFMap(){
 QString fileName = QFileDialog::getSaveFileName(this, tr("Save FMap"),
                            ".",
                            tr("Frequency Map Files (*.fmap)"));
 if(fileName.size() > 0){
    cerr << __PRETTY_FUNCTION__ << " not yet reimplemented..." << endl;
 }
}

void SSAQGLViewerWidget::saveSnapshotCustom(){
  cerr << __PRETTY_FUNCTION__ << ": not yet implemented." << endl;
  QDateTime current;
  QString fileName = "snapshot-";
  fileName.append(current.currentDateTime().toString("dd_MM_yy-hh_mm_ss"));

  QString stateFileName = fileName;
  stateFileName.append(".xml");

  QString snapshotFileName = fileName;
  snapshotFileName.append(".png");
  setSnapshotFileName (snapshotFileName);
  setSnapshotFormat ("PNG");
  setSnapshotQuality (99);
  saveSnapshot(snapshotFileName, true);

  setStateFileName(stateFileName);
  saveStateToFile();
}


void SSAQGLViewerWidget::saveSnapshotVideo(){
  QDateTime current;
  char fileName[2048];
  sprintf(fileName, "snapshot-%03d.png", snapshotCounter());
  
  QString snapshotFileName = fileName;
  setSnapshotFileName (snapshotFileName);
  setSnapshotFormat ("PNG");
  setSnapshotQuality (99);
  saveSnapshot(snapshotFileName, true);
  int i = snapshotCounter();
  setSnapshotCounter(++i);
}

}