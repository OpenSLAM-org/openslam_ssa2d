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

#ifndef __SSA_NORMAL_SHOOTING__
#define __SSA_NORMAL_SHOOTING__

#include "ssa/types/vertex_point_xycov.h"
//PointTree2D
#include "ssa/core/observation2d.h" 

namespace ssa {
  //forward declaration
  class SparseSurfaceAdjustmentGraph;

  class NormalShooting{
    public:
    NormalShooting();
    ~NormalShooting();

    static void shootNormals(SparseSurfaceAdjustmentGraph& graph, int& maxSteps, double& stepSize, double& maxAngleDifference);
    static void shootNormals(SparseSurfaceAdjustmentGraph& graph, PointTree2D& landmarkTreeB, int& scanId, int& maxSteps, double& stepSize, double& maxAngleDifference);
  
  };
}
#endif
