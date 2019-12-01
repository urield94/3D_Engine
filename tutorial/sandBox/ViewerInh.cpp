#include "ViewerInh.h"
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/read_triangle_mesh.h>
#include <Eigen/Core>
#include <set>


void ViewerInh::init_ds_for_data() {
    for (auto &obj : data_list) {
        ViewerInh::edges ds = reset(obj);;
        data_edges.push_back(ds);
    }
}


ViewerInh::edges ViewerInh::reset(igl::opengl::ViewerData obj){
    ViewerInh::edges ds;
    Eigen::MatrixXd V = obj.V;
    Eigen::MatrixXi F = obj.F;
    igl::edge_flaps(F,ds.E,ds.EMAP,ds.EF,ds.EI);

    ds.Qit.resize(ds.E.rows());

    ds.C.resize(ds.E.rows(),V.cols());
    Eigen::VectorXd costs(ds.E.rows());
    ds.Q.clear();
    for(int e = 0;e<ds.E.rows();e++)
    {
        double cost = e;
        Eigen::RowVectorXd p(1,3);
        igl::shortest_edge_and_midpoint(e,V,F,ds.E,ds.EMAP,ds.EF,ds.EI,cost,p);
        ds.C.row(e) = p;
        ds.Qit[e] = ds.Q.insert(std::pair<double,int>(cost,e)).first;
    }
    obj.clear();
    obj.set_mesh(V, F);
    obj.set_face_based(true);
    return ds;
}


void ViewerInh::simplify(int num_to_collapse){
    edges ds = data_edges[selected_data_index];
    Eigen::MatrixXd V = data_list[selected_data_index].V;
    Eigen::MatrixXi F = data_list[selected_data_index].F;
    bool something_collapsed = false;
    int collapsed = 0;
    while(collapsed < num_to_collapse)
    {
        const int top = ds.Q.begin()->second;
        const Eigen::RowVectorXd p = ds.C.row(top);
        ds.Q.erase(ds.Q.begin());
            if(!igl::collapse_edge(top, p,V, F, ds.E, ds.EMAP, ds.EF, ds.EI))
            {
                continue;
            }
            something_collapsed = true;
            ds.num_collapsed++;
            collapsed++;
    }

    if(something_collapsed) {
        data_list[selected_data_index].clear();
        data_list[selected_data_index].set_mesh(V, F);
        data_list[selected_data_index].set_face_based(true);
    }
}


void ViewerInh::simplification(){
    data_edges[selected_data_index] = reset(data());
    long num_of_edges = data_edges[selected_data_index].E.rows();
    if (num_of_edges < 8)
        return;
    const int max_iter = std::ceil(num_of_edges * 0.05);
    simplify(max_iter);
}



