/**
 * dag_nodelet_class.h
 *
 * Author: jixiang
 */
#ifndef DAG_DAG_NODELET_NS_H_
#define DAG_DAG_NODELET_NS_H_ 
#include <nodelet/nodelet.h>

namespace dag_nodelet_ns
{
    class DagNodeletClass : public nodelet::Nodelet
    {
    public:
        DagNodeletClass();
        ~DagNodeletClass();

        virtual void onInit();
    };
} //namespace dag_nodelet-ns

#endif
