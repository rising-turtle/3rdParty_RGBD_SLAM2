#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/core/hyper_dijkstra.h"
#include "g2o/core/robust_kernel_impl.h"

#include <tr1/unordered_map>
// #include <trl/memory>
#include <ros/ros.h>
#include <iostream>

using namespace std;

typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
typedef g2o::LinearSolverDense<SlamBlockSolver::PoseMatrixType> SlamLinearDenseSolver;
typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*>     VertexIDMap;
typedef std::pair<int, g2o::HyperGraph::Vertex*> VertexIDPair;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;

void process_in_rgbdslam();

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "g2o_ext");
  SlamBlockSolver * solver;
  SlamLinearPCGSolver* linearSolver = new SlamLinearPCGSolver();
  solver = new SlamBlockSolver(linearSolver);
  ROS_INFO("what' the matter!");
  process_in_rgbdslam();
  return 0; 
}

void process_in_rgbdslam()
{
  // initialize g2o optimizer, 
  g2o::SparseOptimizer* optimizer_ = new g2o::SparseOptimizer(); 
  optimizer_->setVerbose(false);
  SlamLinearCholmodSolver * linearSolver = new SlamLinearCholmodSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver * solver = new SlamBlockSolver(linearSolver); 
  g2o::OptimizationAlgorithmLevenberg * algo = new g2o::OptimizationAlgorithmLevenberg(solver);
  optimizer_->setAlgorithm(algo);
  
  g2o::HyperGraph::EdgeSet cam_cam_edges_;
  g2o::HyperGraph::VertexSet camera_vertices;

  // add first node 
  {
    Eigen::Quaterniond eigen_quat(1, 0, 0, 0); // W, X, Y, Z
    Eigen::Vector3d translation(0, 0, 0); // X Y Z
    g2o::SE3Quat g2o_ref_se3(eigen_quat, translation);
    g2o::VertexSE3* reference_pose = new g2o::VertexSE3;
    reference_pose->setEstimate(g2o_ref_se3);
    reference_pose->setId(1); 
    reference_pose->setFixed(true);//fix at origin
    optimizer_->addVertex(reference_pose); 
    
    camera_vertices.insert(reference_pose);
  }
  // add second node, and the edge between them 
  {
    Eigen::Quaterniond eigen_quat(1, 0, 0, 0); // W, X, Y, Z
    Eigen::Vector3d translation(1, 0, 0); // X Y Z
    g2o::SE3Quat g2o_ref_se3(eigen_quat, translation);
    g2o::VertexSE3* reference_pose = new g2o::VertexSE3;
    reference_pose->setEstimate(g2o_ref_se3); 
    reference_pose->setId(2);
    reference_pose->setFixed(false);
    optimizer_->addVertex(reference_pose);

    camera_vertices.insert(reference_pose);

    // add edge 
    // g2o::RobustKernelHuber robust_kernel_;
    g2o::VertexSE3* v1 =  dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(1));
    g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3;
    g2o_edge->vertices()[0] = v1;
    g2o_edge->vertices()[1] = reference_pose;
    Eigen::Isometry3d meancopy; 
    g2o_edge->setMeasurement(meancopy);
    // g2o_edge->setRobustKernel(&robust_kernel_);
    Eigen::Matrix<double, 6,6> informationMatrix = Eigen::Matrix<double, 6, 6>::Identity(); 
    g2o_edge->setInformation(informationMatrix);
    optimizer_->addEdge(g2o_edge);

    cam_cam_edges_.insert(g2o_edge);
  }

  // optimize it
  optimizer_->setFixed(camera_vertices, false);
  optimizer_->vertex(1)->setFixed(true);
  optimizer_->initializeOptimization(cam_cam_edges_);
  optimizer_->optimize(5);
  optimizer_->computeActiveErrors();
  double chi2 = optimizer_->chi2();
  //ROS_WARN_STREAM_NAMED("eval", 
  cout<<"G2O Statistics: " << std::setprecision(15) << camera_vertices.size() 
      << " cameras, " << cam_cam_edges_.size() << " edges. " << chi2
      << " ; chi2 "<<endl; 

  g2o::HyperDijkstra hypdij(optimizer_);
  g2o::UniformCostFunction cost_function;
  g2o::VertexSE3* prev_vertex = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(1));
  hypdij.shortestPaths(prev_vertex,&cost_function,1);
}















