#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/prettywriter.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Vector.h>
#include <gtsam/slam/BetweenFactor.h>

#include <iostream>
#include "BetweenFactorHai.h"
#include "write_graph.h"
#include <tuple>

int writeGraph(const std::string &fname, const gtsam::Values &v, const gtsam::NonlinearFactorGraph& graph) {
    using namespace rapidjson;
    StringBuffer s;
    PrettyWriter<StringBuffer> writer(s);
    writer.SetFormatOptions(kFormatSingleLineArray);

    writer.StartObject();
    writer.Key("poses");
    writer.StartArray();
    for (auto kv: v.filter<gtsam::Pose2>()) {
        gtsam::Symbol s(kv.key);

        writer.StartArray();
        writer.Int64(s.index());
        writer.Double(kv.value.x());
        writer.Double(kv.value.y());
        writer.Double(kv.value.theta());
        writer.EndArray();
    }
    writer.EndArray();

    std::vector<std::tuple<int64_t, int64_t> > odomEdges;
    std::vector<std::tuple<int64_t, int64_t, double> > loopClosureEdges;

    for (auto factor_: graph) {
        boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2> > factor =
                boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose2> >(factor_);
        if (factor) {
            odomEdges.push_back(std::make_tuple(gtsam::Symbol(factor->key1()).index(),
                                                gtsam::Symbol(factor->key2()).index()));
        }

        boost::shared_ptr<gtsam::BetweenFactorHai<gtsam::Pose2> > factor_switch =
                boost::dynamic_pointer_cast<gtsam::BetweenFactorHai<gtsam::Pose2> >(factor_);
        if (factor_switch) {
            gtsam::Vector1 sval = v.at<gtsam::Vector1>(factor_switch->key3());
            loopClosureEdges.push_back(std::make_tuple(
                    gtsam::Symbol(factor_switch->key1()).index(),
                    gtsam::Symbol(factor_switch->key2()).index(),
                    sval[0]));
        }
    }

    writer.Key("loop");
    writer.StartArray();
    for (auto ledge: loopClosureEdges) {
        writer.StartArray();
        writer.Int64(std::get<0>(ledge));
        writer.Int64(std::get<1>(ledge));
        writer.Double(std::get<2>(ledge));
        writer.EndArray();
    }
    writer.EndArray();

    writer.Key("odom");
    writer.StartArray();
    for (auto oedge: odomEdges) {
        writer.StartArray();
        writer.Int64(std::get<0>(oedge));
        writer.Int64(std::get<1>(oedge));
        writer.EndArray();
    }
    writer.EndArray();
    writer.EndObject();
    std::cout << s.GetString() << std::endl;
    return 0;
}

void test_graph() {
    //using namespace std;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values v;
    v.insert(gtsam::Symbol('x', 0), gtsam::Pose2(1., 2., 3.));
    v.insert(gtsam::Symbol('x', 1), gtsam::Pose2(1.1, 2., 3.));
    v.insert(gtsam::Symbol('x', 2), gtsam::Pose2(1.2, 2., 3.));
    //gtsam::Vector1 v1;
    //v1 << .75;
    //v.insert(gtsam::Symbol('s', 0), v1);
    //v.insert(gtsam::Symbol('s', 0), gtsam::Vector1::Constant(.75));
    //v.insert(gtsam::Symbol('s', 0), gtsam::Vector1(1.));
    v.insert(gtsam::Symbol('s', 0), (gtsam::Vector1() << gtsam::Vector1::Constant(.75)).finished());

    auto odom_model = gtsam::noiseModel::Gaussian::Covariance(gtsam::Matrix33::Identity(3,3));

    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(
            gtsam::Symbol('x', 0),
            gtsam::Symbol('x', 1),
            gtsam::Pose2(1, 2, 3), odom_model));

    graph.add(gtsam::BetweenFactorHai<gtsam::Pose2>(
            gtsam::Symbol('x', 0),
            gtsam::Symbol('x', 2),
            gtsam::Symbol('s', 0),
            gtsam::Pose2(1, 2, 3), odom_model));

    writeGraph("file.json", v, graph);
}

