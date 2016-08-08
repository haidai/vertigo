#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/filewritestream.h>


class JsonGraph {
    protected:
        FILE *file;
        char buffer[65536];

    public:
        std::shared_ptr<rapidjson::FileWriteStream> fws;
        std::shared_ptr<rapidjson::PrettyWriter<rapidjson::FileWriteStream> > writer;

        JsonGraph(const std::string &fname);
        ~JsonGraph();
        
        void add(const std::string &name, const gtsam::Values &v, const gtsam::NonlinearFactorGraph& graph);
};


int writeGraph(const std::string &fname, const gtsam::Values &v, const gtsam::NonlinearFactorGraph& graph);

