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

#include "import_posegraph2d.h"
#include <iostream>
#include <vector>
#include "g2o/stuff/macros.h"
#include "g2o/math_groups/se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "ssa/types/vertex_point_xycov.h"
#include "ssa/core/ssa_graph.h"
#include "ssa/sensor_models/laser_sensor_model_2d.h"
#include <EXTERNAL/eigen3/Eigen/Core>

namespace ssa {
  using namespace std;
  using namespace g2o;

  void SSAPoseGraph2D::importPoseGraph2D(std::string logfile, SparseSurfaceAdjustmentGraph& ssaGraph){
    ifstream is(logfile.c_str());
    if (! is ){
        cerr << "error in loading the graph" << endl;
        return;
    }

    VertexSE2* previousVertex = 0;
    while(is){
      char buf[4096];
      is.getline(buf,4096);
      istringstream ls(buf);
      string tag;
      ls >> tag;
      if (tag=="VERTEX" || tag=="VERTEX2"){
        int id;
        double x,y,theta;
        ls >> id >> x >> y >> theta;

        VertexSE2* v = new VertexSE2;
        v->setId(id);
        v->estimate() = SE2(x, y, theta);

        ssaGraph.addVertex(v);
        previousVertex = v;
      } else if (tag=="EDGE" || tag=="EDGE2"){
        int id1, id2;
        double x,y,theta;
        ls >> id1 >> id2 >> x >> y >> theta;
        SE2 measurement(x,y,theta);

        Matrix3d m = Matrix3d::Identity();
        ls >> m(0, 0) >> m(0, 1) >> m (1, 1)
           >> m(2, 2) >> m(0, 2) >> m (1, 2);
        m(1, 0)=m(0, 1);
        m(2, 0)=m(0, 2);
        m(2, 1)=m(1, 2);

        previousVertex=0;
        VertexSE2* v1=dynamic_cast<VertexSE2*>(ssaGraph._optimizer.vertex(id1));
        VertexSE2* v2=dynamic_cast<VertexSE2*>(ssaGraph._optimizer.vertex(id2));
        if (! v1 ) {
          cerr << "vertex " << id1 << " is not existing, cannot add edge (" << id1 << "," << id2 << ")" << endl; 
          continue;
        }
        if (! v2 ) {
          cerr << "vertex " << id2 << " is not existing, cannot add edge (" << id1 << "," << id2 << ")" << endl; 
          continue;
        }
        EdgeSE2* edge = new EdgeSE2;
        edge->measurement() = measurement;
        edge->inverseMeasurement() = edge->measurement().inverse();
        edge->information() = m;
        edge->vertices()[0]=v1;
        edge->vertices()[1]=v2;
        edge->computeError();
        ssaGraph.addEdge(edge);
      } 
        else if(tag=="ROBOTLASER1" && previousVertex)
      {
        importRobotLaser(ls, previousVertex, ssaGraph);
      }
    }
  }

  void SSAPoseGraph2D::importRobotLaser(std::istream& is, VertexSE2* parent, SparseSurfaceAdjustmentGraph& ssaGraph)  
  {
    int type;
    double angle, fov, res, maxrange, acc;
    int remission_mode;
    int beams;
    std::vector<double> ranges;
    std::vector<double> remissions;
    double x,y,theta;
    double sensorX,sensorY,sensorTheta;
    double laserTv, laserRv, forwardSafetyDist, sideSaftyDist, turnAxis, timestamp, hostname, loggerTimestamp;

    //parsing log line
    is >> type >> angle >> fov >> res >> maxrange >> acc >> remission_mode;
    is >> beams;
    ranges.resize(beams);
    for (int i=0; i<beams; i++)
      is >> ranges[i];

    is >> beams;
    remissions.resize(beams);
    for (int i = 0; i < beams; i++)
      is >> remissions[i];

    // special robot laser stuff
    is >> x >> y >> theta;
    is >> sensorX >> sensorY >> sensorTheta;
    is >> laserTv >>  laserRv >>  forwardSafetyDist >> sideSaftyDist >> turnAxis;

    // timestamp + host
    is >> timestamp;
    is >> hostname;
    is >> loggerTimestamp;

    SE2 robotPose = SE2(x,y,theta);
    SE2 laserPose = SE2(sensorX,sensorY,sensorTheta);
    SE2 laserOffSet = robotPose.inverse()*laserPose;

    //Convert range readings to observation vertices
    double beamAngle = angle;
    Observation2D observation;
    for(unsigned int i = 0; i < ranges.size(); i++){
      double range = ranges[i];
      Vector2d endpoint(range,0);
      SE2 beam = SE2(0.0,0.0,beamAngle);
      endpoint = laserOffSet * beam * endpoint; // in robot frame

      //create vertex for laserpoint
      VertexPointXYCov* vertex = new VertexPointXYCov;
      vertex->setParentVertex(parent);
      vertex->setToOrigin();
      vertex->estimate() = parent->estimate() * endpoint;
      vertex->covariance() = Eigen::Matrix2d::Identity();
      vertex->setId((parent->id()+1)*10000+i);
      vertex->ratio = beamAngle;
      ssaGraph.addVertex(vertex);

      //add observation edge
      EdgeSE2PointXYCov* edge  = new EdgeSE2PointXYCov;

      edge->vertices()[0]=parent;
      edge->vertices()[1]=vertex;

      edge->measurement() = parent->estimate().inverse() * vertex->estimate();
      edge->inverseMeasurement() = -edge->measurement();

      Matrix2d covariance = Matrix2d::Identity();
      edge->information() = covariance.inverse();
      edge->computeError();
      ssaGraph.addEdge(edge);

      beamAngle += res;
    }
  }

} //end namespace
