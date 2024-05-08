#ifndef _KNCUBE_HPP_
#define _KNCUBE_HPP_

#include "network.hpp"

class HammingMesh : public Network {

  bool _mesh;//一个布尔值，表示网络是否是网格状的

  int _a;
  int _b;
  int _x;
  int _y;
  int _num_routers;
  int _num_switches;

  void _ComputeSize( const Configuration &config );
  void _BuildNet( const Configuration &config );

  //用于获取网络中节点的邻居通道
  int _LeftChannel( int node, int dim );
  int _RightChannel( int node, int dim );

  //用于获取网络中的邻居节点
  int _LeftNode( int node, int dim );
  int _RightNode( int node, int dim );

  //获取路由器在拓扑中的位置
  void _IdToLocation( int node, int* location);
  //获取路由器的行列交换机信息
  std::vector<int> _EdgeRouterGetSwitchIds(int rtr_id);

public:
  //公共方法，这些方法可以被类的实例或其他类使用
  HammingMesh( const Configuration &config, const string & name, bool mesh );
  static void RegisterRoutingFunctions();

  int GetN( ) const;
  int GetK( ) const;

  double Capacity( ) const;

  void InsertRandomFaults( const Configuration &config );

};

#endif
