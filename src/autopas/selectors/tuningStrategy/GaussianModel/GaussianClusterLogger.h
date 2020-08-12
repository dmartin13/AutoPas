/**
 * @file GaussianClusterLogger.h
 * @author Jan Nguyen
 * @date 30.06.20
 */

#pragma once

#include "GaussianModelTypes.h"
#include "GaussianProcess.h"

namespace autopas {

/**
 * Used to print out the clusters of GaussianClusters.
 * The resulting graph represents each cluster as a node and the weight between clusters as edges.
 * The graph is printed as two csv-files.
 */
class GaussianClusterLogger {
  const std::string node_start_marker = "GaussianCluster Graph: Nodes";
  const std::string edge_start_marker = "GaussianCluster Graph: Edges";
  const std::string end_marker = "GaussianCluster Graph: End";

 public:
  /**
   * Options how to output the graphs.
   */
  enum OutputType { none, trace, file };

  /**
   * Constructor. Use default output type.
   * @param vecToStringFun
   */
  GaussianClusterLogger(GaussianModelTypes::VectorToStringFun vecToStringFun)
      : GaussianClusterLogger(vecToStringFun, _defaultOutputType) {}

  /**
   * Constructor
   * @param vecToStringFun function to convert vectors to readable string
   * @param outputType
   */
  GaussianClusterLogger(GaussianModelTypes::VectorToStringFun vecToStringFun, OutputType outputType);

  /**
   * Set the default OutType used if GaussianClusterLogger is initialized with OutputType::defaultType.
   * @param outType
   */
  static void setDefaultOutType(OutputType outType) { _defaultOutputType = outType; }

  /**
   * Change the used function to convert from vector to string.
   * @param fun new converter
   */
  void setVectorToStringFun(const GaussianModelTypes::VectorToStringFun &fun);

  /**
   * Add nodes and edges for given continous sample.
   * @param clusters all clusters
   * @param discreteVectorMap map to convert index to vector
   * @param currentContinous continuous sample
   * @param means predicted mean for each cluster
   * @param vars predicted variance for each cluster
   * @param neighbourWeights neighbours for each cluster
   */
  void add(const std::vector<GaussianProcess> &clusters,
           const std::vector<GaussianModelTypes::VectorDiscrete> &discreteVectorMap,
           const GaussianModelTypes::VectorContinuous &currentContinous, std::vector<double> means,
           std::vector<double> vars, const GaussianModelTypes::NeighboursWeights &neighbourWeights);

  /**
   * Output graph stream accumulated from add() since last call of end().
   */
  void end();

 private:
  /**
   * Reset the stream for both csv-files.
   */
  void reset();

  /**
   * Checks if logger can skip calculations.
   * @return
   */
  bool generatesNoOutput() const;

  OutputType _outType;
  static OutputType _defaultOutputType;

  std::string _outputFileName;

  /**
   * Stream for the nodes csv-file.
   */
  std::stringstream _nodeStream;
  /**
   * Stream for the edges csv-files
   */
  std::stringstream _edgeStream;

  /**
   * Function to convert vectors to strings.
   */
  GaussianModelTypes::VectorToStringFun _vecToStringFun;
};
}  // namespace autopas
