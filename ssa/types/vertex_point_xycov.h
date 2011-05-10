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

#ifndef __SSA_VERTEX_XY_COV_2D__
#define __SSA_VERTEX_XY_COV_2D__
#include <Eigen/Geometry>

#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include <kdtree++/kdtree.hpp>

#define NUMERIC_JACOBIAN_TWO_D_TYPES

//forward declaration
namespace g2o {
  class VertexSE2;
}

namespace ssa {

  class VertexPointXYCov : public g2o::BaseVertex<2, Eigen::Vector2d>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      explicit VertexPointXYCov();
  
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;
  
      virtual void setToOrigin();
      virtual void oplus(double* update);
  
      /** Sensor pose from which this point was observed */
    private:
      g2o::VertexSE2* _parentVertex;
      unsigned int _parentVertexId;
    public:
      g2o::VertexSE2* parentVertex() const;
      unsigned int parentVertexId() const;
      void setParentVertex(g2o::VertexSE2* pose);
    
      /** normal in sensor frame (not saved to disk) */
    private:
      Eigen::Vector2d    _normal; 
  
    public:
      Eigen::Vector2d normal();
      Eigen::Vector2d globalNormal();
      bool            _hasNormal;
  
    /** Covariance in sensor frame */
    private:
      Eigen::Matrix2d _cov; 
    public: 
      Eigen::Matrix2d covariance() const;
      Eigen::Matrix2d& covariance();

    /** ratio between eigenvalues */
    double ratio;
  };

  struct KdTreeAccessXYCov{
    typedef double result_type;
    double operator()( VertexPointXYCov* const& v, size_t k ) const { return v->estimate()(k); }
  };

}
#endif
