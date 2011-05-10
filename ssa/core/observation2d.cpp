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


#include "observation2d.h"
using namespace ssa;

Eigen::Vector2d Observation2D::getMean(std::deque< VertexPointXYCov* >& neighbors){
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for(unsigned int i = 0; i < neighbors.size(); ++i){
      mean += neighbors[i]->estimate();
  }
  mean = mean / (double) neighbors.size();
  return mean;
}

Eigen::Matrix2d Observation2D::getCovariance(std::deque< VertexPointXYCov* >& neighbors, Eigen::Vector2d& mean){
  Eigen::Matrix2d cov = Eigen::Matrix2d::Zero(); 
  for(unsigned int j = 0; j < neighbors.size(); ++j){
    cov +=  (neighbors[j]->estimate() - mean) *  (neighbors[j]->estimate() - mean).transpose();
  }
  double factor = 1.0 / ((double) neighbors.size() - 1.0);
  cov =  factor * cov;
  cov(0,0) += 1e-7;
  cov(1,1) += 1e-7;
  return cov;
}

PointTree2D Observation2D::getKDTree(){
  const Observation2D& observation = *this;
  PointTree2D kdTree;
  for(unsigned int j = 0; j < observation.size(); ++j){
    VertexPointXYCov* point = observation[j];
    kdTree.insert(point);
  }
  kdTree.optimize();
  return kdTree;
}

void Observation2D::calcMeanCov(double& distance){
  Observation2D& observation = *this;
  PointTree2D kdTree = observation.getKDTree();

  for(unsigned int j = 0; j < observation.size(); ++j){
    VertexPointXYCov* point = observation[j];
    std::deque< VertexPointXYCov* > neighbors;
    kdTree.find_within_range(point, distance, std::back_inserter(neighbors));

    if(neighbors.size() > 5){
      //calc mean and covariance
      Eigen::Vector2d mean = Observation2D::getMean(neighbors);
      point->covariance() = Observation2D:: getCovariance(neighbors, mean);
    } else {
      point->covariance()= Eigen::Matrix2d::Identity();
    }
    point->_hasNormal = false;
  }
}