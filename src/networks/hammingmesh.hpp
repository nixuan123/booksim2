#ifndef _HAMMINGMESH_HPP_
#define _HAMMINGMESH_HPP_

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

  //用于建表
  int _LeftNode( int node, int dim );
  int _RightNode( int node, int dim );

  //用于找寻路由器相邻节点
  int find_LeftNode( int node, int other_node, int dim );
  int find_RightNode( int node, int other_node, int dim );

  //获取路由器在拓扑中的位置
  void _IdToLocation( int run_id, int *location);
  //获取路由器的行列交换机信息
  std::vector<int> _EdgeRouterGetSwitchIds(int rtr_id);

  //查表switch_to_routers，返回当前路由器的行或者列交换机(至少有1个)
  std::vector<int> Search_STR(int rID);
  
  //查找switch_input_channels表格
  std::vector<int> Search_SIC(int node);
  
  //查找switch_output_channels表格
  std::vector<int> Search_SOC(int node);

  //查找switch_output_channels表格中交换机
  int Search_OutPort_SOC(int sID);
  
  //hammingmesh中的端口选择函数
  int hammingmesh_xy_port(int cur_router, int dest_router);
  int hammingmesh_ugal_port(int cur_router, int source, int intm, int dest);
  int hammingmesh_hopcnt(int src, int dest);
  bool isEdgeRouter(int rID);
  std::vector<int> midBoard(vector<int> cur_loc,vector<int> dest_loc );
  
public:
  //公共方法，这些方法可以被类的实例或其他类使用
  HammingMesh( const Configuration &config, const string & name );
  static void RegisterRoutingFunctions();

  int GetN( ) const;
  int GetK( ) const;

  double Capacity( ) const;

  void InsertRandomFaults( const Configuration &config );

};

#endif
