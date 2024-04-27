#include "booksim.hpp"
#include <vector>
#include <sstream>

#include "hammingmesh.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
#include "globals.hpp"

HammingMesh::HammingMesh( const Configuration& config, const string & name ) 
: Network(config, name) 
{
cout<< "hammingmesh constructor starts..."<<endl;
_ComputeSize(config);
_Alloc();
_BuildNet(config);
cout<< "hammingmesh constructor ends..."<<endl;

}
void HammingMesh::_ComputeSize( const Configuration &config ) {

cout<< "_ComputeSize starts..."<<endl;

// _a _p _k 均为硬编码，但可以使用GetInt()从配置文件中获取
_a = 4;// 路由器总数
_p = 3;// 每个路由器管理的处理节点

_k = (_a-1) + _p;// 每个路由器的端口数

// 以下三个参数为network中需要使用的
_nodes    = _a * _p; // Number of nodes in network 总节点数(除却路由器)
_size     = _a + _a * _p;      // Number of routers in network 包括路由器和处理节点
_channels = _a * (_a-1);     // 路由器之间的uni-directional link，不包括处理节点

// 全局变量，路由函数需要
gP_hammingmesh = _p;
gA_hammingmesh = _a;

cout<< "_ComputeSize ends..."<<endl;
}
void HammingMesh::_BuildNet( const Configuration& config ) {
// for every router 
    // build the router object
    // add the links to the processing nodes
    // add the links to the other routers

ostringstream router_name;
int node;
int c, cnt;
int port_to_routers = _a - 1;

for (node = 0; node < _a; ++node) {
    // create router
    router_name<< "router";
    router_name<< "_"<< node;
    // k是输入输出端口数
    _routers[node] = Router::NewRouter(config, this, router_name.str(), node, _k, _k);
    _timed_modules.push_back(_routers[node]);
    router_name.str("");
    
    // add input and output channels to processing nodes
    for(cnt = 0; cnt < _p; cnt++){
    c = _p * node + cnt; // for router 0, c is 0,1,2; router 1, c is 3,4,5 and so on.
    _routers[node]->AddInputChannel(_inject[c], _inject_cred[c]);
    }

    for(cnt = 0; cnt < _p; cnt++){
    c = _p * node + cnt; // for router 0, c is 0,1,2; router 1, c is 3,4,5 and so on.
    _routers[node]->AddOutputChannel(_eject[c], _eject_cred[c]);
    }

    // add output and input channels to other routers
    // add output channels
    for(cnt = 0; cnt < _a - 1; cnt++){
    c = port_to_routers * node + cnt; // for router 0, c is 0,1,2; router 1, c is 3,4,5 and so on.
    _routers[node]->AddOutputChannel(_chan[c], _chan_cred[c]);
    }

    // add input channels
    for(cnt = 0; cnt < _a; cnt++)
    {
    if(cnt == node)
    {
        continue;// do nothing
    }
    else if(cnt < node)
    {
        c = cnt * port_to_routers - 1 + node;
    }
    else if(cnt > node)
    {
        c = cnt * port_to_routers + node;
    }
    _routers[node]->AddInputChannel(_chan[c], _chan_cred[c]);
    }
}
}

int hammingmesh_port(int rID, int src, int dest) const// find the right port
{
    int dst_router;
    int out_port;

    dst_router = dest / gP_hammingmesh;

    if(rID == dst_router)// 目标node是在当前路由器管理之下
    {
        out_port = dest % gP_hammingmesh;
    }
    else// 如果是在其他router下管理
    {
        if(dst_router < rID)
        {
        out_port = gP_hammingmesh + dst_router;
        }
        else{
        out_port = gP_hammingmesh + dst_router - 1;
        }
    }

    return out_port;
}

void min_hammingmesh( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject ) const
{
  int debug = f->watch;
  outputs->Clear();

  if(inject)
  {
      int inject_vc = RandomInt(gNumVCs-1);
      outputs->AddRange(-1, inject_vc, inject_vc);
      return;
  }

  int rID = r->GetID();

  int out_port = -1;
  int out_vc = 0;

  if(in_channel < gP_hammingmesh)// source node assign to vc0
  {
      out_vc = 0;
  }
  else// dest node assign it to vc1
  {
      out_vc = 1;
  }

  out_port = hammingmesh_port(rID, f->src, f->dest);

  outputs->AddRange(out_port, out_vc, out_vc);

  if(debug)
  {
      *gWatchOut << GetSimTime()<<" | "<<r->FullName()<<" | "
          <<" through output port : "<< out_port
          <<" out vc: "<< out_vc << endl;
  }
}



