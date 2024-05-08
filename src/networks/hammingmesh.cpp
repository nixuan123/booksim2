#include "booksim.hpp"
#include <vector>
#include <sstream>
#include <ctime>
#include <cassert>
#include "kncube.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
 //#include "iq_router.hpp"

int switch_fid;
std::vector<int> switch_ids;
std::map<int, int> switch_port;
std::vector<std::vector<int>> switch_loc;
std::vector<int> _dim_size;
std::map<int, std::vector<int>> switch_to_routers;

HammingMesh::HammingMesh( const Configuration &config, const string & name ) :
Network( config, name )
{
  _ComputeSize( config );
  _Alloc( );
  _BuildNet( config );
}
//一个主机，每个维度上有两条输出通道和输入通道,在_ComputeSize函数中赋值的变量也全局可用
void HammingMesh::_ComputeSize( const Configuration &config )
{
  _a = config.GetInt( "a" );
  _b = config.GetInt( "b" );
  _x = config.GetInt( "x" );
  _y = config.GetInt( "y" );
  //dim_size已被设置为全局变量
  _dim_size.clear(); // 清空向量，确保没有元素
  _dim_size.push_back(_a);
  _dim_size.push_back(_b);
  _dim_size.push_back(_x);
  _dim_size.push_back(_y);
  //整个拓扑的路由器数量
  _num_routers     = _a*_b*_x*_y;
  //整个拓扑交换机的数量
  _num_switches = _x+_y;
  //交换机的起始id
  switch_fid = _dim_size[0] * _dim_size[1] * _dim_size[2] * _dim_size[3];

  //给switch_ids集合赋值，例如switch_ids=[16,17,18,19]
  for (int i = 0; i < num_switches; ++i) {
    switch_ids.push_back(switch_fid + i);
  }
	
  //将switch_port初始化
  for (int i = 0; i < num_switches; ++i) {
    switch_port[switch_ids[i]] = -1;
  }	
  //将switch_loc初始化
  std::vector<int> index;
  for (int i = 0; i < num_switches; ++i) {
    _IdToLocation(switch_ids[i],index)
    switch_loc.push_back(index);
  }

  //将switch_to_routers初始化，比如switch_to_routers=[16:[],17:[],18:[],19:[]]
  for (int i = 0; i < num_switches; ++i) {
    switch_to_routers[switch_ids[i]] = std::vector<int>();
  }
  //整个拓扑通道的数量,路由器的通道数加上行列交换机的通道数
  _channels = 2*2*_num_routers+(2*_a*_y)*_x+(2*_b*_x)*_y;

  _nodes = _num_routers+_num_switches;
}

void HammingMesh::RegisterRoutingFunctions() {

}

void KNCube::_BuildNet( const Configuration &config )
{
  int* my_location
  int left_node;
  int right_node;

  int right_input;
  int left_input;

  int right_output;
  int left_output;
  //创建了一个ostringstream对象，它是一个能将输出流定向到一个字符串的类
  ostringstream router_name;

  //latency type, noc or conventional network
  bool use_noc_latency;
  use_noc_latency = (config.GetInt("use_noc_latency")==1);
  
  for ( int node = 0; node < _num_routers+_num_switches; ++node ) {
    //将字符串"router"插入到router_name对象中
    router_name << "router";
    
    router_name << location[0] <<locaiton[1]<<location[2]<<location[3];
    if(node<_num_routers){
    //2*_n+1代表路由器的出度和入度
    _routers[node] = Router::NewRouter( config, this, router_name.str( ), 
					node, 2*2+ 1, 2*2 + 1 );
    _timed_modules.push_back(_routers[node]);

    router_name.str("");

    for ( int dim = 0; dim < _n; ++dim ) {

      //find the neighbor 
      left_node  = _LeftNode( node, dim );
      right_node = _RightNode( node, dim );

      //
      // Current (N)ode
      // (L)eft node
      // (R)ight node
      //
      //   L--->N<---R
      //   L<---N--->R
      //

      // torus channel is longer due to wrap around
      int latency = _mesh ? 1 : 2 ;

      //get the input channel number
      right_input = _LeftChannel( node, right_node, dim );
      left_input  = _RightChannel( node, left_node, dim );

      //add the input channel
      _routers[node]->AddInputChannel( _chan[right_input], _chan_cred[right_input] );
      _routers[node]->AddInputChannel( _chan[left_input], _chan_cred[left_input] );

      //set input channel latency
      if(use_noc_latency){
	_chan[right_input]->SetLatency( latency );
	_chan[left_input]->SetLatency( latency );
	_chan_cred[right_input]->SetLatency( latency );
	_chan_cred[left_input]->SetLatency( latency );
      } else {
	_chan[left_input]->SetLatency( 1 );
	_chan_cred[right_input]->SetLatency( 1 );
	_chan_cred[left_input]->SetLatency( 1 );
	_chan[right_input]->SetLatency( 1 );
      }
      //get the output channel number
      right_output = _RightChannel( node, dim );
      left_output  = _LeftChannel( node, dim );
      
      //add the output channel
      _routers[node]->AddOutputChannel( _chan[right_output], _chan_cred[right_output] );
      _routers[node]->AddOutputChannel( _chan[left_output], _chan_cred[left_output] );

      //set output channel latency
      if(use_noc_latency){
	_chan[right_output]->SetLatency( latency );
	_chan[left_output]->SetLatency( latency );
	_chan_cred[right_output]->SetLatency( latency );
	_chan_cred[left_output]->SetLatency( latency );
      } else {
	_chan[right_output]->SetLatency( 1 );
	_chan[left_output]->SetLatency( 1 );
	_chan_cred[right_output]->SetLatency( 1 );
	_chan_cred[left_output]->SetLatency( 1 );

      }
    }
    //injection and ejection channel, always 1 latency
    _routers[node]->AddInputChannel( _inject[node], _inject_cred[node] );
    _routers[node]->AddOutputChannel( _eject[node], _eject_cred[node] );
    _inject[node]->SetLatency( 1 );
    _eject[node]->SetLatency( 1 );
    }
    //配置交换机的输入输出链路
    else{
	  
    }
  }
 
}


int HammingMesh::_LeftChannel( int node, int other_node, int dim )
{
  if(other_node<_num_routers){
  // The base channel for a node is 2*_n*node
  int base = 2*_n*node;
  // The offset for a left channel is 2*dim + 1
  int off  = 2*dim + 1;

  return ( base + off );
  }
  else{
	  
  }
}

int HammingMesh::_RightChannel( int node, int other_node, int dim )
{
  if(other_node<_num_routers){
  // The base channel for a node is 2*_n*node
  int base = 2*2*node;
  // The offset for a right channel is 2*dim 
  int off  = 2*dim;
  return ( base + off );
  }else{
	  //查询表格对应通道
  }
}
//这个函数是找寻该路由器在dim维度中的左邻居路由器/交换机节点id（难点）
int HammingMesh::_LeftNode( int node, int dim )
{
  int* location;
  _IdToLocation(node,location);//比如node=4，现在location=[0,0,1,0] 
  std::vector<int> my_switches(2,0);
  int left_node=0;
  if(dim==0){
	if( location[0]>0){
	    left_node=node-1;
	}else{
		//返回行交换机的id
		my_switches=_EdgeRouterGetSwitchIds(node);
		left_node=my_switches[0];
	}
  }
  if(dim==1){
	if(location[1]<_dim_size[0]){
	    left_node=node+_dim_size[1]	  
	}else{
		//返回列交换机的id
		my_switches=_EdgeRouterGetSwitchIds(node);
		left_node=my_switches[1];
	}	  
  }

return left_node;
}

int HammingMesh::_RightNode( int node, int dim )
{
  int* location;
  _IdToLocation(node,location);//比如node=4，现在location=[0,0,1,0] 
  int right_node=0;
  if(dim==0){
	if( location[0]<_dim_size[1]-1){
	    right_node=node+1;
	}else{
		//返回行交换机的id
		my_switches=_EdgeRouterGetSwitchIds(node);
		right_node=my_switches[0];
	}
  }
  if(dim==1){
	if(location[1]>0){
	    right_node=node-_dim_size[1]	  
	}else{
		//返回列交换机的id
		my_switches=_EdgeRouterGetSwitchIds(node);
		right_node=my_switches[1];
	}
   }
  
return right_node;
}

int HammingMesh::_IdToLocation(int run_id, int *location) {
    int hm_id = 0;
    int inner_id = 0;
    int num = 0;
    
    int i;
    // 初始化location数组
    for (i = 0; i < 4; i++) {
        location[i] = 0;
    }
	
    if (0 < run_id && run_id < switch_fid) {
        // 设置前两位
        inner_id = run_id % (dim_size[0] * dim_size[1]);
        location[0] = inner_id % dim_size[1];
        location[1] = inner_id / dim_size[1];

        // 设置后两位
        hm_id = run_id / (dim_size[0] * dim_size[1]);
        location[2] = hm_id % dim_size[3];
        location[3] = hm_id / dim_size[3];
    } else if (switch_fid <= run_id && run_id < switch_fid + dim_size[2]) {
        // 设置前两位
        location[0] = run_id;
        location[1] = run_id;

        // 设置后两位
        num = run_id - switch_fid;
        location[2] = run_id;
        location[3] = num;
    } else if (switch_fid + dim_size[2] <= run_id && run_id < switch_fid + dim_size[2] + dim_size[3]) {
        // 设置前两位为 run_id
        location[0] = run_id;
        location[1] = run_id;

        // 设置后两位
        num = run_id - (switch_fid + _dim_size[2]);
        location[2] = num;
        location[3] = run_id;
    } else {
        // 如果 rtr_id 不在预期的范围内，设置每个位置为 run_id
        for (i = 0; i < 4; i++) {
            location[i] = run_id;
        }
    }
}

std::vector<int> HammingMesh::_EdgeRouterGetSwitchIds(int rtr_id){
	//初始化一个空vector用于保存当前路由器的行列交换机信息
	std::vector<int> my_switches(2,0)
	int* location;
	_IdToLocation(rtr_id,location);
	// 判断路由器是否在对角的位置
        if (((loc[0] == 0 && loc[1] == 0) ||
             (loc[0] == _dim_size[1] - 1 && loc[1] == 0) ||
             (loc[0] == 0 && loc[1] == _dim_size[0] - 1) ||
             (loc[0] == _dim_size[1] - 1 && loc[1] == _dim_size[0] - 1))) {
             // 找寻行交换机
             for (auto& location : switch_loc) {
                    if (location[3] == loc[3]) {
                       switch_port[location[0]]++;
                       switch_to_routers[location[0]].push_back(i);
                       my_switches[0] = location[0];
                       break;
                     }
	     }
	     for (auto& location : switch_loc) {
                    if (location[2] == loc[2]) {
                       switch_port[location[0]]++;
                       switch_to_routers[location[0]].push_back(i);
                       my_switches[0] = location[0];
                       break;
		    }
	      }
	}
	//如果是列边缘，则寻找行交换机
	else if(loc[0] == 0 || loc[0] == _dim_size[1] - 1){
	     for (auto& location : switch_loc) {
                    if (location[3] == loc[3]) {
                       switch_port[location[0]]++;
                       switch_to_routers[location[0]].push_back(i);
                       my_switches[0] = location[0];
                       break;
	             }
	      }
	}
	//如果是行边缘，则寻找列交换机
	else if(loc[1] == 0 || loc[1] == _dim_size[0] - 1){
	      for (auto& location : switch_loc) {
                    if (location[2] == loc[2]) {
                       switch_port[location[0]]++;
                       switch_to_routers[location[0]].push_back(i);
                       my_switches[1] = location[0];
                       break;
		    }
	      }	
	}
	else{
		printf("Error message: The current router is not an edge router!");
	}
//返回该节点的行列交换机的信息
return my_switches;
}


int KNCube::GetN( ) const
{
  return _n;
}

int KNCube::GetK( ) const
{
  return _k;
}

/*legacy, not sure how this fits into the new scheme of things*/
void HammingMesh::InsertRandomFaults( const Configuration &config )
{
  int num_fails = config.GetInt( "link_failures" );
  
  if ( _size && num_fails ) {
    vector<long> save_x;
    vector<double> save_u;
    SaveRandomState( save_x, save_u );
    int fail_seed;
    if ( config.GetStr( "fail_seed" ) == "time" ) {
      fail_seed = int( time( NULL ) );
      cout << "SEED: fail_seed=" << fail_seed << endl;
    } else {
      fail_seed = config.GetInt( "fail_seed" );
    }
    RandomSeed( fail_seed );

    vector<bool> fail_nodes(_size);

    for ( int i = 0; i < _size; ++i ) {
      int node = i;

      // edge test
      bool edge = false;
      for ( int n = 0; n < _n; ++n ) {
	if ( ( ( node % _k ) == 0 ) ||
	     ( ( node % _k ) == _k - 1 ) ) {
	  edge = true;
	}
	node /= _k;
      }

      if ( edge ) {
	fail_nodes[i] = true;
      } else {
	fail_nodes[i] = false;
      }
    }

    for ( int i = 0; i < num_fails; ++i ) {
      int j = RandomInt( _size - 1 );
      bool available = false;
      int node = -1;
      int chan = -1;
      int t;

      for ( t = 0; ( t < _size ) && (!available); ++t ) {
	int node = ( j + t ) % _size;
       
	if ( !fail_nodes[node] ) {
	  // check neighbors
	  int c = RandomInt( 2*_n - 1 );

	  for ( int n = 0; ( n < 2*_n ) && (!available); ++n ) {
	    chan = ( n + c ) % 2*_n;

	    if ( chan % 1 ) {
	      available = fail_nodes[_LeftNode( node, chan/2 )];
	    } else {
	      available = fail_nodes[_RightNode( node, chan/2 )];
	    }
	  }
	}
	
	if ( !available ) {
	  cout << "skipping " << node << endl;
	}
      }

      if ( t == _size ) {
	Error( "Could not find another possible fault channel" );
      }

      assert(node != -1);
      assert(chan != -1);
      OutChannelFault( node, chan );
      fail_nodes[node] = true;

      for ( int n = 0; ( n < _n ) && available ; ++n ) {
	fail_nodes[_LeftNode( node, n )]  = true;
	fail_nodes[_RightNode( node, n )] = true;
      }

      cout << "failure at node " << node << ", channel " 
	   << chan << endl;
    }

    RestoreRandomState( save_x, save_u );
  }
}

double HammingMesh::Capacity( ) const
{
  return (double)_k / ( _mesh ? 8.0 : 4.0 );
}
