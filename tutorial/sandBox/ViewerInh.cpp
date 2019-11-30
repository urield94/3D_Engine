//
// Created by rochberg on 11/29/19.
//

#include "ViewerInh.h"
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/collapse_edge.h>


ViewerInh::edges ViewerInh::init_ds(Eigen::MatrixXd V, Eigen::MatrixXi F){
    edges ds;
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

    return ds;
}

void ViewerInh::init_ds_for_data() {
    for (auto &obj : data_list) {
        Eigen::MatrixXd V = obj.V;
        Eigen::MatrixXi F = obj.F;
        data_edges.push_back(init_ds(V, F));
        obj.clear();
        obj.set_mesh(V,F);
        obj.set_face_based(true);
    }
}

void ViewerInh::ce(int num_to_collapse){

    Eigen::MatrixXd V = data_list[selected_data_index].V;
    Eigen::MatrixXi F = data_list[selected_data_index].F;
    data_edges[selected_data_index] = init_ds(V, F);

    edges ds = data_edges[selected_data_index];

    int num_collapsed = 0;
    bool something_collapsed = false;
    for(int j = 0;j<num_to_collapse;j++)
    {
            if(!igl::collapse_edge(igl::shortest_edge_and_midpoint,V, F, ds.E, ds.EMAP, ds.EF, ds.EI, ds.Q, ds.Qit, ds.C))
            {
                break;
            }
            something_collapsed = true;
            num_collapsed++;

    }

    if(something_collapsed) {
        data_list[selected_data_index].clear();
        data_list[selected_data_index].set_mesh(V, F);
        data_list[selected_data_index].set_face_based(true);
        data_edges[selected_data_index] = init_ds(V, F);
    }
}



void ViewerInh::simplification(){
//    Eigen::MatrixXd V = data_list[selected_data_index].V;
//    Eigen::MatrixXi F = data_list[selected_data_index].F;
//    data_edges[selected_data_index] = init_ds(V, F);

    const int max_iter = std::ceil(data_edges[selected_data_index].E.rows() * 0.005);
    ce(max_iter);
    draw();

}


