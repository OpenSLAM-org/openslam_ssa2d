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

#ifndef __SPARSE_SURFACE_ADJUSTMENT_OBSERVATION_2D__
#define __SPARSE_SURFACE_ADJUSTMENT_OBSERVATION_2D__

#include <vector>
#include <deque>
#include <Eigen/Core>
#include "ssa/types/vertex_point_xycov.h"

namespace ssa {
  typedef KDTree::KDTree<2, VertexPointXYCov*, KdTreeAccessXYCov>    PointTree2D;

  class Observation2D : public std::vector<VertexPointXYCov* >
  {
    public:
    /** calculate mean */
    static Eigen::Vector2d getMean(std::deque< VertexPointXYCov* >& neighbors);

    /** calculate covariance */
    static Eigen::Matrix2d getCovariance(std::deque< VertexPointXYCov* >& neighbors, Eigen::Vector2d& mean);

    //!returns a kdtree with all observation vertices of this observation
    PointTree2D getKDTree();

    void calcMeanCov(double& distance);
  };

} //end namespace

#endif