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

#include "ssa_graph.h"
#include <Eigen/QR> 

namespace ssa{

  SparseSurfaceAdjustmentGraph::SparseSurfaceAdjustmentGraph()
  {
  }

  SparseSurfaceAdjustmentGraph::~SparseSurfaceAdjustmentGraph()
  {
  }

  void SparseSurfaceAdjustmentGraph::load(string filename){
    ifstream graphstream(filename.c_str());
    if (! graphstream ){
      cerr << "SparseSurfaceAdjustmentGraph::load( " << filename << ")" << ": error in loading the graph." << endl;
      return;
    }
    _optimizer.load(graphstream);
    linkNodesToVertices();
    cerr << "loaded ssa graph with " << _optimizer.vertices().size() << " vertices and " << _optimizer.edges().size() << " edges." << std::endl; 
  }

  void SparseSurfaceAdjustmentGraph::save(string filename){
    ofstream graphstream(filename.c_str());
    if (! graphstream ){
      cerr << "SparseSurfaceAdjustmentGraph::save( " << filename << ")" << ": error while opening the target file." << endl;
      return;
    }
    _optimizer.save(graphstream);
    graphstream.close();
  }

  void SparseSurfaceAdjustmentGraph::linkNodesToVertices(){

    for (g2o::OptimizableGraph::VertexIDMap::const_iterator it=_optimizer.vertices().begin(); it!=_optimizer.vertices().end(); it++){
      VertexPointXYCov* v=dynamic_cast<VertexPointXYCov*>(it->second);
      if(v){  
        v->setParentVertex(dynamic_cast<g2o::VertexSE2*> (_optimizer.vertices()[v->parentVertexId()]));
        if(v->parentVertex()){
          _verticies_points.push_back(v);
         }
      }
    }

    for (g2o::OptimizableGraph::EdgeSet::iterator it=_optimizer.edges().begin(); it!=_optimizer.edges().end(); it++){
      g2o::EdgeSE2* e1=dynamic_cast<g2o::EdgeSE2*>(*it);
      double chi2 = 0.0;
      if(e1){
        e1->computeError();
        chi2 = e1->chi2();
        if(chi2 < 0.0){
          cerr << "Error: NEGATIVE INFORMATION MATRIX... THIS SHOULD NOT HAPPEN..." << endl;
          cerr << "g2o::EdgeSE2 " << chi2 << "\t";
          cerr << e1->information() << std::endl;
          exit(-1);
        }
        _edges_odometry.push_back(e1);
      } else {
        EdgeSE2PointXYCov* e2=dynamic_cast<EdgeSE2PointXYCov*>(*it);
        if(e2){
          e2->computeError();
          chi2 = e2->chi2();
          if(chi2 < 0.0){
            cerr << "Error: NEGATIVE INFORMATION MATRIX... THIS SHOULD NOT HAPPEN..." << endl;
            cerr << "EdgeSE3PointXYCov " << e2->vertices()[0]->id() << " " << e2->vertices()[1]->id()  << " " << chi2 << endl;
            cerr << e2->information() << std::endl;
            e2->information().setIdentity();
          }
          VertexPointXYCov* v1=dynamic_cast<VertexPointXYCov*>(e2->vertices()[1]);
          _verticies_observations[v1->parentVertexId()].push_back(v1);
          _edges_observations.push_back(e2);
        } else {
          EdgePointXYCovPointXYCov* e3=dynamic_cast<EdgePointXYCovPointXYCov*>(*it);
          if(e3){
            e3->computeError();
            chi2 = e3->chi2();
            if(chi2 < 0.0){
              cerr << "Error: NEGATIVE INFORMATION MATRIX... THIS SHOULD NOT HAPPEN..." << endl;
              cerr << e3->vertices()[0]->id() << " " << e3->vertices()[1]->id() << " " << endl;
              cerr << fixed << chi2 << "\r\n";
              cerr << e3->information() << std::endl;
              exit(-1);
            }
            _edges_data_association.push_back(e3);
          }
        }
      }

    }
    cerr << endl;
  }

  std::map<int, int> SparseSurfaceAdjustmentGraph::getScanIds(){
    std::map<int, int> keys;
    for (g2o::OptimizableGraph::VertexIDMap::const_iterator it=_optimizer.vertices().begin(); it!=_optimizer.vertices().end(); it++){
      g2o::VertexSE2* v=dynamic_cast<g2o::VertexSE2*>(it->second);
      if(v)
        keys[v->id()] = v->id();
    }
    return keys;
  }

  void SparseSurfaceAdjustmentGraph::calcMeanCov(double& maxDist){
    //create kdtree per scan
    std::map<int, int> keys = getScanIds();
    for(std::map<int, int>::iterator it = keys.begin(); it != keys.end(); ++it){
      int scanId = it->first;
      calcMeanCov(maxDist, scanId);
    }
  }

  void SparseSurfaceAdjustmentGraph::calcMeanCov(double& distance, int& scanId){
    Observation2D& observation = _verticies_observations[scanId];
//     cerr << __PRETTY_FUNCTION__ << " " << PVAR(scanId) << " " << observation.size() << " " << distance << endl;
    PointTree2D kdTree = observation.getKDTree();

    for(unsigned int j = 0; j < observation.size(); ++j){
      VertexPointXYCov* point = observation[j];
      std::deque< VertexPointXYCov* > neighbors;
      kdTree.find_within_range(point, distance, std::back_inserter(neighbors));

      if(neighbors.size() > 2){
        //calc mean and covariance
        Eigen::Vector2d mean = Observation2D::getMean(neighbors);
        point->covariance() = Observation2D:: getCovariance(neighbors, mean);
      } else {
        point->covariance()= Eigen::Matrix2d::Identity();
      }
      point->_hasNormal = false;
    }
  }


  g2o::OptimizableGraph::EdgeSet SparseSurfaceAdjustmentGraph::getEdgeset()
  {
    g2o::OptimizableGraph::EdgeSet eset;
    std::map<int, bool>  pointWithCorrespondence;
    for (g2o::OptimizableGraph::EdgeSet::iterator it=_optimizer.edges().begin(); it!=_optimizer.edges().end(); it++){
        EdgePointXYCovPointXYCov* e3=dynamic_cast<EdgePointXYCovPointXYCov*>(*it);
        if(e3){
          VertexPointXYCov* v1=dynamic_cast<VertexPointXYCov*>(e3->vertices()[0]);
          VertexPointXYCov* v2=dynamic_cast<VertexPointXYCov*>(e3->vertices()[1]);
          if(v1->parentVertex()->id() != v2->parentVertex()->id()){
            pointWithCorrespondence[v1->id()] = true;
            pointWithCorrespondence[v2->id()] = true;
            eset.insert(e3);
          } 
        }
    }
    for (g2o::OptimizableGraph::EdgeSet::iterator it=_optimizer.edges().begin(); it!=_optimizer.edges().end(); it++){
      EdgeSE2PointXYCov* e2=dynamic_cast<EdgeSE2PointXYCov*>(*it);
      if(e2){
        VertexPointXYCov* v1=dynamic_cast<VertexPointXYCov*>(e2->vertices()[1]);
        if(pointWithCorrespondence[v1->id()]){
          eset.insert(e2);
        } else{
          _edges_points_to_move.push_back(e2);
        }
      } 
    }
    return eset;
  }

  void SparseSurfaceAdjustmentGraph::moveUnoptimizedPoints(){
    for(size_t i = 0; i < _edges_points_to_move.size(); ++i){
      EdgeSE2PointXYCov* e=_edges_points_to_move[i];
      g2o::VertexSE2*    v1=dynamic_cast<g2o::VertexSE2*>(e->vertices()[0]);
      VertexPointXYCov* v2=dynamic_cast<VertexPointXYCov*>(e->vertices()[1]);
      v2->estimate() = v1->estimate() * e->measurement();
    }
    cerr << "moved " << _edges_points_to_move.size() << " point without correspondences." << endl;
    _edges_points_to_move.clear();
  }


  ssa::PointTree2D SparseSurfaceAdjustmentGraph::getKDTreeOfScans(int& scanId)
  {
    PointTree2D kdTree;
    std::map<int, int> keys = getScanIds();
    for(std::map<int, int>::iterator it = keys.begin(); it != keys.end(); ++it){
      if(it->second != scanId){
        Observation2D& o = _verticies_observations[it->second];
        for(unsigned int j = 0; j < o.size(); ++j){
          VertexPointXYCov* point = o[j];
          kdTree.insert(point);
        }
      }
    }
    kdTree.optimize();
    return kdTree;
  }

  void SparseSurfaceAdjustmentGraph::addVertex(VertexSE2*& v)
  {
    _optimizer.addVertex(v);
    _verticies_poses.push_back(v);
  }

  void SparseSurfaceAdjustmentGraph::addVertex(VertexPointXYCov*& v)
  {
    _optimizer.addVertex(v);
    _verticies_points.push_back(v);
    _verticies_observations[v->parentVertex()->id()].push_back(v);
  }

  void SparseSurfaceAdjustmentGraph::addEdge(EdgeSE2*& edge)
  {
    _edges_odometry.push_back(edge);
    _optimizer.addEdge(edge);
  }

  void SparseSurfaceAdjustmentGraph::addEdge(EdgeSE2PointXYCov*& edge)
  {
    _edges_observations.push_back(edge);
    _optimizer.addEdge(edge);
  }

  void SparseSurfaceAdjustmentGraph::addEdge(VertexPointXYCov*& from, VertexPointXYCov*& to)
  {
    if(from == to){
      cerr << "Warning: you tried to create an edge between an observation and itself." << endl;
      return;
    }

    EdgePointXYCovPointXYCov* e = new EdgePointXYCovPointXYCov;
    e->measurement().fill(0.0);
    e->inverseMeasurement().fill(0.0);
    e->vertices()[0]=from;
    e->vertices()[1]=to;
    e->information() = Eigen::Matrix2d::Identity();
    e->computeError();
    if(from->parentVertex()->id() != to->parentVertex()->id()){
      _edges_data_association.push_back(e);
    } else {
      cerr << "Warning: you tried to create an edge between two points of the same observation." << endl;
    }
    _optimizer.addEdge(e);

  }

  void SparseSurfaceAdjustmentGraph::dropDataAssociation()
  {

    for(size_t i = 0; i < _edges_data_association.size(); ++i){
      EdgePointXYCovPointXYCov*& e = _edges_data_association[i];
      if(e){
        _optimizer.removeEdge(e);
      }
    }
    _edges_data_association.clear();
  }

} //end namespace