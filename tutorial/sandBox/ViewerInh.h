

#ifndef ENGIENIGLNEW_VIEWER_SEMPL_H
#define ENGIENIGLNEW_VIEWER_SEMPL_H


#include <igl/opengl/glfw/Viewer.h>
#include <queue>
#include <iostream>


class ViewerInh: public igl::opengl::glfw::Viewer {
    public:
//        typedef std::set<std::pair<double,int> > PriorityQueue;
        typedef std::pair<double, int> pi;
        struct edges{
            int num_collapsed;
            Eigen::MatrixXi E;
            Eigen::MatrixXi EF;
            Eigen::MatrixXi EI;
            Eigen::VectorXi EMAP;
            std::priority_queue <pi, std::vector<pi>, std::greater<pi>> Q;
            std::vector<Eigen::Vector3d> C;
        };
        std::vector<edges> data_edges;
        void my_collapse(  const int e,
                           const Eigen::RowVectorXd & p,
                           Eigen::MatrixXd & V,
                           Eigen::MatrixXi & F,
                           Eigen::MatrixXi & E,
                           Eigen::VectorXi & EMAP,
                           Eigen::MatrixXi & EF,
                           Eigen::MatrixXi & EI);
        edges reset(igl::opengl::ViewerData obj);
        void simplify(int num_to_collapse);
        void init_ds_for_data();
        void simplification();
};


#endif //ENGIENIGLNEW_VIEWER_SEMPL_H
