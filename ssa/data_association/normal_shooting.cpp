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

#include "normal_shooting.h" 
#include "ssa/core/ssa_graph.h" 
#include <vector>

namespace ssa{

  NormalShooting::NormalShooting()
  {
  };
  
  NormalShooting::~NormalShooting()
  {
  };

  void 
  NormalShooting::shootNormals(SparseSurfaceAdjustmentGraph& graph, int& maxSteps, double& stepSize, double& maxAngleDifference)
  {
    std::map<int, int> keys = graph.getScanIds();
  
    //create kdtree of remaining points
    int dummyId = 0;
    PointTree2D kdTree = graph.getKDTreeOfScans(dummyId);
  
    for(std::map<int, int>::iterator it = keys.begin(); it != keys.end(); ++it)
    {
      int id1 = it->first;
      shootNormals(graph, kdTree, id1, maxSteps, stepSize, maxAngleDifference);
    } 
  }
  
  void 
  NormalShooting::shootNormals(SparseSurfaceAdjustmentGraph& graph, PointTree2D& kdTree, int& scanId, int& maxSteps, double& stepSize, double& maxAngleDifference)
  {
    //get vertices for scan
    Observation2D& scan = graph._verticies_observations[scanId];
    for(unsigned int k = 0; k < scan.size(); ++k)
    {
      VertexPointXYCov* point = scan[k];

      //We do not apply normal shooting only if the normal is well defined
      if(point->covariance() == Eigen::Matrix2d::Identity())
        continue;

      Eigen::Vector2d normal = point->normal();
      VertexPointXYCov* vp = new VertexPointXYCov;
      for(int l=0; l < maxSteps; ++l)
      {
        Eigen::Vector2d delta;
        for(int i=0; i < 2; ++i)
          delta(i) = ((l*stepSize)*normal(i));
  
        vp->estimate() = point->estimate() + delta;
        std::pair<PointTree2D::const_iterator,double> found = kdTree.find_nearest(vp, stepSize);
        if(found.first != kdTree.end())
        {
          VertexPointXYCov* correspondence = (*found.first);
          if(correspondence->parentVertex()->id() == point->parentVertex()->id())
            continue;
          double angleBetweenNormals = fabs(acos(point->normal().dot(correspondence->normal())));
          if(angleBetweenNormals < DEG2RAD(maxAngleDifference) || correspondence->covariance() == Eigen::Matrix2d::Identity()) 
          {
            graph.addEdge(correspondence, point);
          } 
        }
      }
      delete vp;
    }
  }
}
