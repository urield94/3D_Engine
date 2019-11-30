

#ifndef ENGIENIGLNEW_VIEWER_SEMPL_H
#define ENGIENIGLNEW_VIEWER_SEMPL_H


#include <igl/opengl/glfw/Viewer.h>
#include <csetjmp>
#include <csignal>
#include <cstdlib>

class ViewerInh: public igl::opengl::glfw::Viewer {
    public:
        typedef std::set<std::pair<double,int> > PriorityQueue;
        struct edges{
            Eigen::MatrixXi E;
            Eigen::MatrixXi EF;
            Eigen::MatrixXi EI;
            Eigen::VectorXi EMAP;
            PriorityQueue Q;
            std::vector<PriorityQueue::iterator > Qit;
            Eigen::MatrixXd C;
        };
        std::vector<edges> data_edges;
        edges init_ds(Eigen::MatrixXd V, Eigen::MatrixXi F);
        void ce(int num_to_collapse);
        void init_ds_for_data();
        void simplification();


        static void on_sigabrt (int signum);

         void try_and_catch_abort (int e);
};


#endif //ENGIENIGLNEW_VIEWER_SEMPL_H
