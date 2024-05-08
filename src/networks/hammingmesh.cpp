#include "booksim.hpp"
#include <vector>
#include <sstream>
#include <ctime>
#include <cassert>
#include "kncube.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
 //#include "iq_router.hpp"

const int switch_fid;
std::vector<int> switch_ids;
std::map<int, int> switch_port;
std::vector<std::vector<int>> switch_loc;
std::vector<int> _dim_size;
HammingMesh::HammingMesh( const Configuration &config, const string & name ) :
Network( config, name )
{
  _ComputeSize( config );
  _Alloc( );
  _BuildNet( config );
}
//一个主机，每个维度上有两条输出通道和输入通道
void HammingMesh::_ComputeSize( const Configuration &config )
{
  _a = config.GetInt( "a" );
  _b = config.GetInt( "b" );
  _x = config.GetInt( "x" );
  _y = config.GetInt( "y" );
  //dim_size已被设置为全局变量
  _dim_size[0]=_a;
  _dim_size[1]=_b;
  _dim_size[2]=_x;
  _dim_size[3]=_y;
  //整个拓扑的路由器数量
  _num_routers     = _a*_b*_x*_y;
  //整个拓扑交换机的数量
  _num_switches = _x+_y;
  //整个拓扑通道的数量
  _channels = 2*2*_num_routers+(2*_a*_y)*_x+(2*_b*_x)*_y;

  _nodes = _num_routers+_num_switches;
}

void HammingMesh::RegisterRoutingFunctions() {

}
void HammingMesh::_BuildNet( const Configuration &config )
{
  int* mylocation;
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
    if(node < _num_routers){
    //将字符串"router"插入到router_name对象中
    router_name << "router";
    //将路由器的id转换为在拓扑中的位置，比如mylocation=[0,0,1,0]
    _IdToLocation(node,mylocation);
    //给路由器命名
    router_name << location[0] <<locaiton[1]<<location[2]<<location[3];
    
    //2*n+1代表路由器的出度和入度,在这里规定n=2，只可能是二维的
    _routers[node] = Router::NewRouter( config, this, router_name.str( ), 
					node, 2*2 + 1, 2*2 + 1 );
    _timed_modules.push_back(_routers[node]);

    //将字符串对象内容重置为空串，以便下一次的命名
    router_name.str("");

    for ( int dim = 0; dim < 2; ++dim ) {

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
      int latency =  1 ;

      //get the input channel number
      right_input = _LeftChannel( right_node, dim );
      left_input  = _RightChannel( left_node, dim );

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
  }
  //给交换机配置链路
  else{
		
	}
}

int HammingMesh::_LeftChannel( int node, int dim )
{
  // The base channel for a node is 2*_n*node
  int base = 2*_n*node;
  // The offset for a left channel is 2*dim + 1
  int off  = 2*dim + 1;

  return ( base + off );
}

int HammingMesh::_RightChannel( int node, int dim )
{
  // The base channel for a node is 2*_n*node
  int base = 2*_n*node;
  // The offset for a right channel is 2*dim 
  int off  = 2*dim;
  return ( base + off );
}
//这个函数是找寻该路由器在dim维度中的左邻居路由器节点id
int HammingMesh::_LeftNode( int node, int dim )
{
  int* location;
  _IdToLocation(node,location);//比如node=4，现在location=[0,0,1,0] 
  int left_node;
  if(dim==0){
	if( location[0]>0)
	{
		left_node=node-1;
	}else{
		left_node=
	}
  }else{
	  
  }
  
  // if at the left edge of the dimension, wraparound
  if ( loc_in_dim == 0 ) {
    left_node = node + (_k-1)*k_to_dim;
  } else {
    left_node = node - k_to_dim;
  }

  return left_node;
}

int HammingMesh::_RightNode( int node, int dim )
{
  int k_to_dim = powi( _k, dim );
  int loc_in_dim = ( node / k_to_dim ) % _k;
  int right_node;
  // if at the right edge of the dimension, wraparound
  if ( loc_in_dim == ( _k-1 ) ) {
    right_node = node - (_k-1)*k_to_dim;
  } else {
    right_node = node + k_to_dim;
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
    //switch_fid【新增】是新定义的全局变量，用于存储hm拓扑的起始交换机id
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
