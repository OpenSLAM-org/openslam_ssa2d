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

#ifndef __SPARSE_SURFACE_ADJUSTMENT_GRAPH__
#define __SPARSE_SURFACE_ADJUSTMENT_GRAPH__

#define NUMERIC_JACOBIAN_TWO_D_TYPES

#include <vector>
#include <QObject>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "EXTERNAL/g2o/g2o/stuff/timeutil.h"

//Basic Vertex / Edge types
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "ssa/types/vertex_point_xycov.h"
#include "ssa/types/edge_se2_xycov.h"
#include "ssa/types/edge_xycov_xycov.h"

#include "observation2d.h"
#include "ssa_sparse_optimizer.h"

#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimizable_graph.h"

#include <kdtree++/kdtree.hpp>

namespace ssa {

  class SparseSurfaceAdjustmentGraph{
  
    public:
    SparseSurfaceAdjustmentGraph();
    ~SparseSurfaceAdjustmentGraph();
  
    void load(string filename);
    void save(string filename);
  
    /** This method sets the parent vertex pointer of all 
        VertexPointXYCov vertices and fills the pointer into 
        type specific vectors for faster and easier access. */
    void linkNodesToVertices();
  
    //!returns a map with the ids of all scans
    std::map<int, int> getScanIds();
  
    //--------------------------
    // Mean and covariance stuff
    //--------------------------
    void calcMeanCov();
    void calcMeanCov(double& distance);
    //! calculate mean and covariance of for scan with the given scanId
    void calcMeanCov(double& distance, int& scandId);
  
  
    /** Returns a subset of edges from the current graph for the 
      optimization. The subset contains all edge types for point 
      vertices  that have a data association to another point 
      vertex from a different observation.*/
    g2o::OptimizableGraph::EdgeSet getEdgeset();
  
    /** move points that have no data association */
    void moveUnoptimizedPoints();
  
    //!returns a kdtree with all observation vertices without the given scan observations
    PointTree2D getKDTreeOfScans(int& scanId);
  
    void addVertex(VertexSE2*& v);
    void addVertex(VertexPointXYCov*& v);
    void addEdge(EdgeSE2*& edge);
    void addEdge(EdgeSE2PointXYCov*& edge);
    void addEdge(VertexPointXYCov*& from, VertexPointXYCov*& to);
  
    void dropDataAssociation();
  
    /** Optimizer */
    SSASparseOptimizer _optimizer;
  
    /** Direct access to differnt edgetypes */
  
    std::vector<VertexSE2* >                _verticies_poses;
    std::vector<VertexPointXYCov* >         _verticies_points;
    std::map< int, Observation2D >          _verticies_observations;
    std::vector< EdgeSE2* >                 _edges_odometry;
    std::vector< EdgeSE2PointXYCov* >       _edges_observations;
    std::vector<EdgePointXYCovPointXYCov* > _edges_data_association;
  
    /** vertices/edges of points that are not in the current optimization set 
        are moved accordingly to the resulting observation position */
    std::vector<EdgeSE2PointXYCov* >        _edges_points_to_move;
  };

} //end namespace

#endif