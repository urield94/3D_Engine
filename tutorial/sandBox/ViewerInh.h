

#ifndef ENGIENIGLNEW_VIEWER_SEMPL_H
#define ENGIENIGLNEW_VIEWER_SEMPL_H


#include <igl/opengl/glfw/Viewer.h>


class ViewerInh: public igl::opengl::glfw::Viewer {
    public:
        typedef std::set<std::pair<double,int> > PriorityQueue;
        struct edges{
            int num_collapsed;
            Eigen::MatrixXi E;
            Eigen::MatrixXi EF;
            Eigen::MatrixXi EI;
            Eigen::VectorXi EMAP;
            PriorityQueue Q;
            std::vector<PriorityQueue::iterator > Qit;
            Eigen::MatrixXd C;
        };
        std::vector<edges> data_edges;
        edges reset(igl::opengl::ViewerData obj);
        void simplify(int num_to_collapse);
        void init_ds_for_data();
        void simplification();
};


#endif //ENGIENIGLNEW_VIEWER_SEMPL_H
