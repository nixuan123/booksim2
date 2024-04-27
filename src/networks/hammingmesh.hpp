 #ifndef _TestNet_HPP_
 #define _TestNet_HPP_

 #include "network.hpp"
 #include "routefunc.hpp"

 class HammingMesh : public Network {
 void _ComputeSize( const Configuration &config );
 void _BuildNet( const Configuration &config );
 
 public:
 HammingMesh( const Configuration &config, const string & name );
 static void RegisterRoutingFunctions(){};

 int _a;// 路由器总数
 int _p;// 每个路由器管理的处理节点
 int _k;// 每个路由器的端口数
 int hammingmesh_port(int rID, int src, int dest) const;

 //
 // Routing Functions
 //
 void min_hammingmesh( const Router *r, const Flit *f, int in_channel, OutputSet *outputs, bool inject ) const;

 };



 #endif


