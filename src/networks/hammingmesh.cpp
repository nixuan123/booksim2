#include "booksim.hpp"
#include <vector>
#include <map>
#include <sstream>
#include <ctime>
#include <cassert>
#include "kncube.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
 //#include "iq_router.hpp"

int switch_fid;
int switch_x_fchannel;
int switch_y_fchannel;
std::vector<int> switch_ids;
std::map<int, int> switch_port;
std::vector<std::vector<int>> switch_loc;
std::vector<int> _dim_size;
std::map<int, std::vector<int>> switch_to_routers;
std::map<int, std::vector<std::vector<int>>> switch_input_channels;
std::map<int, std::vector<std::vector<int>>> switch_output_channels;

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
  
  //行交换机起始通道号
  switch_x_fchannel=2*2*_num_routers;

  //列交换机的起始通道号
  switch_y_fchannel=switch_x_fchannel+(2*_a*_y)*_x;
  
  //给switch_ids集合赋值，例如switch_ids=[16,17,18,19]
  for (int i = 0; i < num_switches; ++i) {
    switch_ids.push_back(switch_fid + i);
  }
	
  //将switch_port初始化
  for (int i = 0; i < num_switches; ++i) {
    switch_port[switch_ids[i]] = -1;
  }	
  //创建一个集合，用于存储交换机映射后的位置
  vector<int> s_loc(4,0);
  for (int i = 0; i < num_switches; ++i) {
    _IdToLocation(switch_ids[i],s_loc)
    switch_loc.push_back(s_loc);
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
      gRoutingFunctionMap["min_hammingmesh"]=&min_hammingmesh;
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

  vector<int> s_to_nodes;
  vector<int> s_input;
  vector<int> s_output;
  //创建了一个ostringstream对象，它是一个能将输出流定向到一个字符串的类
  ostringstream router_name;

  //latency type, noc or conventional network
  bool use_noc_latency;
  use_noc_latency = (config.GetInt("use_noc_latency")==1);

  //建表,从路由器0遍历到_num_routers+_num_switches;维度从0->1;行交换机->列交换机;
  for ( int node = 0; node < _num_routers+_num_switches; ++node ) {
	  for ( int dim = 0; dim < 2; ++dim ) { 
           left_node  = _LeftNode( node, dim );
           right_node = _RightNode( node, dim );
          }
  }

  //搭建拓扑
  for ( int node = 0; node < _num_routers+_num_switches; ++node ) {
    //将字符串"router"插入到router_name对象中
    router_name << "router";
    _IdToLocation(node,my_location);
    router_name << my_location[0] <<my_location[1]<<my_location[2]<<my_location[3];
    
    if(node<_num_routers){
    //2*_n+1代表路由器的出度和入度
    _routers[node] = Router::NewRouter( config, this, router_name.str( ), 
					node, 2*2+ 1, 2*2 + 1 );
    _timed_modules.push_back(_routers[node]);

    router_name.str("");

    for ( int dim = 0; dim < 2; ++dim ) {

      //find the neighbor 
      left_node  = find_LeftNode( node, dim );
      right_node = find_RightNode( node, dim );
      
      //
      // Current (N)ode
      // (L)eft node
      // (R)ight node
      //
      //   L--->N<---R
      //   L<---N--->R
      //

      // torus channel is longer due to wrap around
      int latency = 2;

      //get the input channel number
      right_input = find_LeftChannel( node, right_node, dim );
      left_input  = find_RightChannel( node, left_node, dim );

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
      right_output = find_RightChannel( node, 0, dim );
      left_output  = find_LeftChannel( node, 0, dim );
      
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
    //injection and ejection channel, always 1 latency, 终端的连接
    _routers[node]->AddInputChannel( _inject[node], _inject_cred[node] );
    _routers[node]->AddOutputChannel( _eject[node], _eject_cred[node] );
    _inject[node]->SetLatency( 1 );
    _eject[node]->SetLatency( 1 );
    }
    //配置交换机的输入输出链路
    else{
    _routers[node] = Router::NewRouter( config, this, router_name.str( ), 
					node,switch_port[node]+1, switch_port[node]+1);
    _timed_modules.push_back(_routers[node]);

    router_name.str("");
    //get the input channel vector
    std::vector v;
    v=switch_input_channels[node];
    for(auto num:v){
       //add the input vector of numbers
       _routers[node]->AddInputChannel( _chan[v[1]], _chan_cred[v[1]] );
       //set input channel latency
       if(use_noc_latency){
	_chan[v[1]]->SetLatency( latency );
	_chan_cred[v[1]]->SetLatency( latency );
      } else {
	_chan[v[1]]->SetLatency( 1 );
	_chan_cred[v[1]]->SetLatency( 1 );
      }	    
     }
    
    //get the output channel number
    v=switch_output_channels[node];
    for(auto num:v){
       //add the output vector of numbers
       _routers[node]->AddInputChannel( _chan[v[1]], _chan_cred[v[1]] );
       //set input channel latency
       if(use_noc_latency){
	_chan[v[1]]->SetLatency( latency );
	_chan_cred[v[1]]->SetLatency( latency );
      } else {
	_chan[v[1]]->SetLatency( 1 );
	_chan_cred[v[1]]->SetLatency( 1 );
      }	    
     }
    }
  }
 
}


int HammingMesh::find_LeftChannel( int node, int other_node, int dim )
{
  if(other_node<_num_routers){
  // The base channel for a node is 2*_n*node
  int base = 2*2*node;
  // The offset for a left channel is 2*dim + 1
  int off  = 2*dim + 1;
  return ( base + off );
  }
  else{  //说明是找行 or 列交换机的输出通道
         //查表找dim维度的交换机（0-行 1-列）的输出通道
	  my_channels=Search_SOC(node);
	  return my_channels[dim];
  } 
}
  


int HammingMesh::find_RightChannel( int node, int other_node, int dim )
{
  std::vector<int> my_channels(2,0);
  if(other_node<_num_routers){
  // The base channel for a node is 2*_n*node
  int base = 2*2*node;
  // The offset for a right channel is 2*dim 
  int off  = 2*dim;
  return ( base + off );
  }else{  //说明是找行（0） or 列（1）交换机的输出通道
	  //查表找dim维度的交换机（0-行 1-列）的输出通道
	  my_channels=Search_SOC(node);
	  return my_channels[dim];
	  
  }
}

//用于建表
int HammingMesh::_LeftNode( int node, int dim )
{
  std::vector<int> location(4,0);
  _IdToLocation(node,location);//比如node=4，现在location=[0,0,1,0] 
  std::vector<int> my_switches(2,0);
  int base = 2*2*node; 
  int off=0;
  int left_node=0;
  if(dim==0){
	if( location[0]>0){
	    left_node=node-1;
	}else{  //说明在0维度的左节点是（行）交换机
		//返回行交换机的id
		my_switches=_EdgeRouterGetSwitchIds(node);
		off=2*dim+1;
		//记录路由器的连接节点和输入通道
		std::vector<int> value = {node,base+off};
		switch_input_channels[my_switches[0]].push_back(value);	
		//记录路由器的连接节点和输出通道
		std::vector<int> value1 = {node,switch_x_fchannel};
		switch_output_channels[my_switches[0]].push_back(value1);
		switch_x_fchannel++;
		left_node=my_switches[0];
	}
  }
  if(dim==1){
	if(location[1]<_dim_size[0]-1){
	    left_node=node+_dim_size[1]	  
	}else{  //说明在1维度的左节点是（列）交换机
		//返回列交换机的id
		my_switches=_EdgeRouterGetSwitchIds(node);
		off=2*dim+1;
		//记录路由器的连接节点和输入通道
		std::vector<int> value = {node,base+off};
		switch_input_channels[my_switches[1]].push_back(value);	
		//记录路由器的连接节点和输出通道
		std::vector<int> value1 = {node,switch_x_fchannel};
		switch_output_channels[my_switches[1]].push_back(value1);
		switch_y_fchannel++; 
		left_node=my_switches[1];
	}	  
  }

return left_node;
}

//用于建表
int HammingMesh::_RightNode( int node, int dim )
{
  std::vector<int> location(4,0);
  _IdToLocation(node,location);//比如node=4，现在location=[0,0,1,0] 
  int base = 2*2*node; 
  int off=0;
  int right_node=0;
  if(dim==0){
	if( location[0]<_dim_size[1]-1){
	    right_node=node+1;
	}else{  //说明在0维度的右节点是（行）交换机
		//返回行交换机的id
		my_switches=_EdgeRouterGetSwitchIds(node);
		off=2*dim;
		//记录路由器的连接节点和输入通道
		std::vector<int> value = {node,base+off};
		switch_input_channels[my_switches[0]].push_back(value);	
		//记录路由器的连接节点和输出通道
		std::vector<int> value1 = {node,switch_x_fchannel};
		switch_output_channels[my_switches[0]].push_back(value1);
		switch_x_fchannel++;
		right_node=my_switches[0];
	}
  }
  if(dim==1){
	if(location[1]>0){
	    right_node=node-_dim_size[1]	  
	}else{  //说明在1维度的右节点是（列）交换机
		//返回列交换机的id
		my_switches=_EdgeRouterGetSwitchIds(node);
		off=2*dim;
		//记录路由器的连接节点和输入通道
		std::vector<int> value = {node,base+off};
		switch_input_channels[my_switches[0]].push_back(value);	
	        //记录路由器的连接节点和输出通道
		std::vector<int> value1 = {node,switch_y_fchannel};
		switch_output_channels[my_switches[1]].push_back(value1);
		switch_y_fchannel++;
		right_node=my_switches[1];
	}
   }
return right_node;
}


//路由器找左节点的函数
int HammingMesh::find_LeftNode( int node, int dim )  {
std::vector<int> my_switches(2,0);
if(dim==0){
	if( location[0]>0){
	    left_node=node-1;
	}else{
	    //查表返回（行）交换机
		my_switches=Search_STR(node);
		left_node=my_switches[0];
	}
    }
if(dim==1){
	if(location[1]<_dim_size[0]-1){
	    left_node=node+_dim_size[1]	  
	}else{
	    //查表返回（列）交换机
		my_switches=Search_STR(node);
		left_node=my_switches[1];
	}
return left_node;	
}

//路由器找右节点的函数
int HammingMesh::find_RightNode( int node, int dim )  {
if(dim==0){
	if( location[0]<_dim_size[1]-1){
	    right_node=node+1;
	}else{
	    //查表返回（行）交换机
	        my_switches=Search_STR(node);
		right_node=my_switches[0];
	}
    }
if(dim==1){
	if(location[1]>0){
	    right_node=node-_dim_size[1]	  
	}else{
	    //查表返回（列交换机）
		my_switches=Search_STR(node);
		right_node=my_switches[1];
	}
return right_node;	
}

//查表switch_to_routers，返回当前路由器的行或者列交换机(至少有1个)
std::vector<int> HammingMesh::Search_STR(int node){
	std::vector<int> my_switches(2,0);
	int flag=0；
	for (auto pair:switch_to_routers){
	    for (auto num:pair.second){
		if(num==node){
		    if(pair.first<_num_routers+_x){
			my_switches[0]=pair.first;   
		    }else{
			my_switches[1]=pair.first;
		    }
		}
	     }
	}
	return my_switches;
}

//查找switch_input_channels,返回当前路由器连接行或者列交换机的输入通道（至少1条）
std::vector<int> HammingMesh::Search_SIC(int node){
	std::vector<int> my_channels(2,0);
	for (auto pair:switch_input_channels){
	    for (auto v:pair.second){
		    if(v[0]==node){
		      if(pair.first<_num_routers+_x){
			my_channels[0]=v[1];   
		      }else{
			my_channels[1]=v[1];
		    }  
		}
	    }
	}
	return my_channels;
}

//查找switch_output_channels,返回当前路由器连接行或者列交换机的输出通道（至少1条）
std::vector<int> HammingMesh::Search_SOC(int node){
	std::vector<int> my_channels(2,0);
	for (auto pair:switch_input_channels){
	    for (auto v:pair.second){
		    if(v[0]==node){
		      if(pair.first<_num_routers+_x){
			my_channels[0]=v[1];   
		      }else{
			my_channels[1]=v[1];
		    }  
		}
	    }
	}
	return my_channels;
}

void HammingMesh::_IdToLocation(int run_id, vector<int>& location) {
    int hm_id = 0;
    int inner_id = 0;
    int num = 0;
    
    int i;
    // 初始化location数组
    for (int i = 0; i < 4; i++) {
        location[i] = 0;
    }
	
    if (0 < run_id && run_id < switch_fid) {
        // 设置前两位
        inner_id = run_id % (_dim_size[0] * _dim_size[1]);
        location[0] = inner_id % _dim_size[1];
        location[1] = inner_id / _dim_size[1];

        // 设置后两位
        hm_id = run_id / (_dim_size[0] * _dim_size[1]);
        location[2] = hm_id % _dim_size[3];
        location[3] = hm_id / _dim_size[3];
    } else if (switch_fid <= run_id && run_id < switch_fid + _dim_size[2]) {
        // 设置前两位
        location[0] = run_id;
        location[1] = run_id;

        // 设置后两位
        num = run_id - switch_fid;
        location[2] = run_id;
        location[3] = num;
    } else if (switch_fid + _dim_size[2] <= run_id && run_id < switch_fid + _dim_size[2] + _dim_size[3]) {
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
	int i = rtr_id;
        //初始化一个空vector用于保存当前路由器的行列交换机信息
	std::vector<int> my_switches(2,0);
	std::vector<int> location(4,0);
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
             // 找寻列交换机
	     for (auto& location : switch_loc) {
                    if (location[2] == loc[2]) {
                       switch_port[location[0]]++;
                       switch_to_routers[location[0]].push_back(i);
                       my_switches[1] = location[0];
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
