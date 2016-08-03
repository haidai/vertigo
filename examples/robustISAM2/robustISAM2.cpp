/*
 * robustISAM2.cpp
 *
 *  Created on: Jul 13, 2012
 *      Author: niko
 */

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <gtsam/base/Vector.h>
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "boost/foreach.hpp"
#define foreach BOOST_FOREACH

#include <fstream>

#include "BetweenFactorHai.h"
#include "timer.h"

using namespace std;


// ===================================================================
struct Pose {
  int id;
  double x,y,th;
};

struct Edge {
  int i, j;
  double x, y, th;
  bool switchable;
  bool maxMix;
  gtsam::Matrix covariance;
  double weight;
};


// ===================================================================
bool parseDataset(string inputFile, vector<Pose>&poses, vector<Edge>&edges,multimap<int, int> &poseToEdges)
{
	 cout << endl << "Parsing dataset " << inputFile << " ... " << endl;

	 // open the file
	 ifstream inFile(inputFile.c_str());
	 if (!inFile) {
		 cerr << "Error opening dataset file " << inputFile << endl;
		 return false;
	 }

	 // go through the dataset file
	 while (!inFile.eof()) {
		 // depending on the type of vertex or edge, read the data
		 string type;
		 inFile >> type;

		 if (type == "VERTEX_SE2") {
			 Pose p;
			 inFile  >> p.id >> p.x >> p.y >> p.th;
			 poses.push_back(p);
		 }

		 else if (type == "EDGE_SE2_SWITCHABLE" || type == "EDGE_SE2" || type == "EDGE_SE2_MAXMIX") {
			 double dummy;
			 Edge e;

			 // read pose IDs
			 inFile >> e.i >> e.j;

			 if (e.i>e.j) {
			   swap(e.i,e.j);
			 }

			 // read the switch variable ID (only needed in g2o, we dont need it here in the gtsam world)
			 if (type == "EDGE_SE2_SWITCHABLE") inFile >> dummy;
			 if (type == "EDGE_SE2_MAXMIX") inFile >> e.weight;


			 // read odometry measurement
			 inFile >> e.x >> e.y >> e.th;

			 // read information matrix
			 double info[6];
			 inFile >> info[0] >> info[1] >> info[2] >> info[3] >> info[4] >> info[5];
             gtsam::Matrix informationMatrix(3,3);
             informationMatrix << info[0], info[1], info[2], info[1], info[3], info[4], info[2], info[4], info[5];
			 e.covariance = informationMatrix.inverse();

			 if (type == "EDGE_SE2_SWITCHABLE") {
			   e.switchable=true;
			   e.maxMix=false;
			 }
			 else if (type == "EDGE_SE2_MAXMIX") {
			   e.switchable=false;
			   e.maxMix=true;
			 }
			 else {
			   e.switchable=false;
			   e.maxMix=false;
			 }

			 edges.push_back(e);

			 int id = edges.size()-1;
			 poseToEdges.insert(pair<int, int>(e.j, id));
		 }
		 // else just read the next item at next iteration until one of the if clauses is true
	 }

     return true;
}


// ===================================================================
int main(int argc, char *argv[])
{
    std::string inputFile = argv[1];

    std::vector<Pose> poses;
    std::vector<Edge> edges;
    std::multimap<int, int> poseToEdges;
    if (!parseDataset(inputFile, poses, edges, poseToEdges)) {
        cerr << "Error parsing the dataset."<< endl;
        return 0;
    }

    // init iSam
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);


    // Insert new observations as factors into graph and initialEstimate and
    // then hand them over to iSAM2 incrementally (implying that we clear out
    // graph and initialEstimate everytime).
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    int switchCounter = -1;

    printf("Number of poses %d\n", poses.size());
    printf("Number of edges %d\n", edges.size());

    // Build problem
    // Loop through each pose
    for (int i = 0; i < poses.size(); i++) {
        auto p = poses[i];
        auto edgesFromPose = poseToEdges.equal_range(p.id);
        printf("Pose %d: \n", p.id);

        // Loop through each edge associated with that pose
        for (auto it = edgesFromPose.first; it != edgesFromPose.second; it++) {
            auto edge = edges[it->second];
            printf("    edge: %d -- %d (%.2f %.2f %.2f)\n", edge.i, edge.j,
                    edge.x, edge.y, edge.th);
            printf("        switchable: %d\n", edge.switchable);
            printf("        maxMix: %d\n", edge.maxMix);
            printf("        weight: %.3f\n", edge.weight);
            //std::cout <<  "        cov:\n[" << edge.covariance << "]" << std::endl;

            // If this is an odometry edge
            if (edge.j == edge.i + 1) {

                // If it links to the first pose, since we can't incrementally solve for the first pose
                // (iSAM needs at least two factors on a variable), we can't ask
                // for a smoothed estimate of it yet
                if (edge.i == 0) {
                    initialEstimate.insert(gtsam::Symbol('x', p.id), gtsam::Pose2(edge.x, edge.y, edge.th));

                } else {
                    gtsam::Pose2 predecessorPose = isam.calculateEstimate<gtsam::Pose2>(gtsam::Symbol('x', p.id - 1));
                    if (std::isnan(predecessorPose.x()) || std::isnan(predecessorPose.y()) || std::isnan(predecessorPose.theta())) {
                        std::cout << "! Degenerated solution (NaN) detected. Solver failed." << std::endl;
                        //writeResults(globalInitialEstimate, outputFile);
                        return 0;
                    }

                    initialEstimate.insert(gtsam::Symbol('x', p.id), predecessorPose * gtsam::Pose2(edge.x, edge.y, edge.th));
                }

                //globalInitialEstimate.insertPose(p.id, predecessorPose * Pose2(e.x, e.y, e.th));
                gtsam::SharedNoiseModel odom_model = gtsam::noiseModel::Gaussian::Covariance(edge.covariance);
                printf("graph: Odom BetweenFactor x%d -- x%d\n", edge.i, edge.j);
                graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
                            gtsam::Symbol('x', edge.i), gtsam::Symbol('x', edge.j),
                            gtsam::Pose2(edge.x, edge.y, edge.th),
                            odom_model));

            } else if (edge.switchable) {
                // create new switch variable
                initialEstimate.insert(gtsam::Symbol('s', ++switchCounter), gtsam::Vector1(1.0));

                // create switch prior factor
                gtsam::SharedNoiseModel switchPriorModel = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(1.0));

                printf("graph: Switch Prior s%d\n", switchCounter);
                graph.add(gtsam::PriorFactor<gtsam::Vector1>(
                            gtsam::Symbol('s', switchCounter),
                            gtsam::Vector1(1.0),
                            switchPriorModel));

                // create switchable odometry factor
                printf("graph: Switch BetweenFactor x%d -- x%d -- s%d\n", edge.i, edge.j, switchCounter);
                gtsam::SharedNoiseModel odom_model = gtsam::noiseModel::Gaussian::Covariance(edge.covariance);
                graph.add(gtsam::BetweenFactorHai<gtsam::Pose2>(
                            gtsam::Symbol('x', edge.i),
                            gtsam::Symbol('x', edge.j),
                            gtsam::Symbol('s', switchCounter),
                            gtsam::Pose2(edge.x, edge.y, edge.th), odom_model));
            }
        }

        if (p.id == 0) {
            printf("    init: %d\n", i);
            // add prior for first pose
            gtsam::noiseModel::Diagonal::shared_ptr prior_model = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(0.1));
            printf("graph: x0 prior\n");
            graph.add(gtsam::PriorFactor<gtsam::Pose2>(
                        gtsam::Symbol('x', 0),
                        gtsam::Pose2(p.x, p.y, p.th),
                        prior_model));

            // initial value for first pose
            initialEstimate.insert(gtsam::Symbol('x', 0), gtsam::Pose2(p.x, p.y, p.th));
        } else {
            printf("isam: update with estimate\n");
            isam.update(graph, initialEstimate);
            printf("isam: calculate\n");
            isam.update();
            gtsam::Values currentEstimate = isam.calculateEstimate();

            if (i % 100 || i == std::max(poses.size() - 1, size_t(0))) {
                std::cout << "****************************************************" << std::endl;
                std::cout << "Iteration " << i << ": " << std::endl;
                currentEstimate.print("Current estimate: ");
            }

            graph.resize(0);
            initialEstimate.clear();
        }
    }

    return 0;
}
