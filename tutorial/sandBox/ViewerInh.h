

#ifndef ENGIENIGLNEW_VIEWER_SEMPL_H
#define ENGIENIGLNEW_VIEWER_SEMPL_H


#include <igl/opengl/glfw/Viewer.h>
#include <queue>
#include <iostream>

typedef std::set<std::pair<double,int> > PriorityQueue;
typedef struct {
    int num_collapsed;
    Eigen::MatrixXi E;
    Eigen::MatrixXi EF;
    Eigen::MatrixXi EI;
    Eigen::VectorXi EMAP;
    Eigen::MatrixXd C;
    PriorityQueue Q;
    std::vector<PriorityQueue::iterator > Qit;
    std::vector<Eigen::Matrix4d> EQ; //Error Quadrics vector
} edges;

class ViewerInh: public igl::opengl::glfw::Viewer {
    public:

        edges *reset(igl::opengl::ViewerData obj, edges* ds);
        void simplify(int num_to_collapse);
        void init_ds_for_data();
        void simplification();
        ~ViewerInh() {
            for (auto &ds : data_edges) {
                delete(ds);
            }
        }
        std::vector<edges*> data_edges;
private:

    static std::vector<Eigen::Matrix4d> get_Q(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &F_normals);
    static  void calc_edges_cost( const int e,
                                     const Eigen::MatrixXd & V,
                                     const Eigen::MatrixXi & F,
                                     const Eigen::MatrixXi & E,
                                     const Eigen::VectorXi & EMAP,
                                     const Eigen::MatrixXi & EF,
                                     const Eigen::MatrixXi & EI,
                                     double & cost,
                                     Eigen::RowVectorXd & p,
                                     std::vector<Eigen::Matrix4d> &Q);
    static bool my_collapse(Eigen::MatrixXd & V,
                            Eigen::MatrixXi & F,
                            Eigen::MatrixXd &F_normals,
                            edges *ds);


};


#endif //ENGIENIGLNEW_VIEWER_SEMPL_H
