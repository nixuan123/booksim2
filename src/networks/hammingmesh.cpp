#include "booksim.hpp"
#include <vector>
#include <map>
#include <sstream>
#include <ctime>
#include <cassert>
#include "hammingmesh.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
 //#include "iq_router.hpp"

int switch_fid;
int switch_x_fchannel;
int switch_y_fchannel;
int _total_routers;
int switch_y_f_id;
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
  _num_routers  = _a*_b*_x*_y;
  _total_routers = _num_routers;

  switch_y_f_id=switch_fid+_x;
  //整个拓扑交换机的数量
  _num_switches = _x+_y;
  //每个路由器的终端数
  gC=1;
  
  //交换机的起始id
  switch_fid = _dim_size[0] * _dim_size[1] * _dim_size[2] * _dim_size[3];
  
  //行交换机起始通道号
  switch_x_fchannel = 2*2*_num_routers;

  //列交换机的起始通道号
  switch_y_fchannel = switch_x_fchannel+(2*_a*_y)*_x;
  
  //给switch_ids集合赋值，例如switch_ids=[16,17,18,19]
  for (int i = 0; i < _num_switches; ++i) {
    switch_ids.push_back(switch_fid + i);
  }
	
  //将switch_port初始化
  for (int i = 0; i < _num_switches; ++i) {
    switch_port[switch_ids[i]] = -1;
  }	
  
  //创建一个集合，用于存储交换机映射后的位置
  std::vector<int> s_loc(4,0);
  for (int i = 0; i < _num_switches; ++i) {
    _IdToLocation(switch_ids[i],s_loc);
    switch_loc.push_back(s_loc);
  }

  //将switch_to_routers初始化，比如switch_to_routers=[16:[],17:[],18:[],19:[]]
  for (int i = 0; i < _num_switches; ++i) {
    switch_to_routers[switch_ids[i]] = std::vector<int>();
  }

  //整个拓扑通道的数量,路由器的通道数加上行列交换机的通道数
  _channels = 2*2*_num_routers+(2*_a*_y)*_x+(2*_b*_x)*_y;
  
  //整个拓扑中节点的数量
  _size= _num_routers+_num_switches;

  //整个拓扑中终端的数量
  _nodes = _num_routers*1;
}

void HammingMesh::_BuildNet( const Configuration &config )
{
  std::vector<int> my_location;
  int left_node;
  int right_node;

  int right_input;
  int left_input;

  int right_output;
  int left_output;

  //std::vector<int> s_input(3,0);
  //std::vector<int> s_output(3,0);
  //创建了一个ostringstream对象，它是一个能将输出流定向到一个字符串的类
  ostringstream router_name;

  //latency type, noc or conventional network
  bool use_noc_latency;
  use_noc_latency = (config.GetInt("use_noc_latency")==1);

  //建表,从路由器0遍历到_num_routers+_num_switches;维度从0->1;行交换机->列交换机;
  for ( int node = 0; node < _num_routers; ++node ) {
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
    // torus channel is longer due to wrap around
    int latency = 2;
    if(node<_num_routers){
    //2*_n+1代表路由器的出度和入度，双向通道，输入和输出都是一个端口号
    _routers[node] = Router::NewRouter( config, this, router_name.str( ), 
					node, 2*2+ 1, 2*2 + 1 );
    _timed_modules.push_back(_routers[node]);

    router_name.str("");
    
    //injection and ejection channel, 1 latency, 终端的连接
    _routers[node]->AddInputChannel( _inject[node], _inject_cred[node] );
    _routers[node]->AddOutputChannel( _eject[node], _eject_cred[node] );
    _inject[node]->SetLatency( 1 );
    _eject[node]->SetLatency( 1 );
    
    //switch and router channel, always 1 latency, 路由器和交换机之间的连接
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
    
    }
    //配置交换机的输入输出链路
    else{
    _routers[node] = Router::NewRouter( config, this, router_name.str( ), 
					node,switch_port[node]+1, switch_port[node]+1);
    _timed_modules.push_back(_routers[node]);

    router_name.str("");
    //get the input channel vector
    auto s_input=switch_input_channels[node];
    for(auto num:s_input){
       //add the input vector of numbers
       _routers[node]->AddInputChannel( _chan[num[1]], _chan_cred[num[1]] );
       //set input channel latency
       if(use_noc_latency){
	_chan[num[1]]->SetLatency( latency );
	_chan_cred[num[1]]->SetLatency( latency );
      } else {
	_chan[num[1]]->SetLatency( 1 );
	_chan_cred[num[1]]->SetLatency( 1 );
      }	    
     }
    
    //get the output channel number
    auto s_output=switch_output_channels[node];
    for(auto num:s_output){
       //add the output vector of numbers
       _routers[node]->AddOutputChannel( _chan[num[1]], _chan_cred[num[1]] );
       //set input channel latency
       if(use_noc_latency){
	_chan[num[1]]->SetLatency( latency );
	_chan_cred[num[1]]->SetLatency( latency );
      } else {
	_chan[num[1]]->SetLatency( 1 );
	_chan_cred[num[1]]->SetLatency( 1 );
      }	    
     }
    }
  }
 
}

//返回路由器或者交换机在拓扑中的位置
void _IdToLocation(int run_id, std::vector<int>& location) {
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
		//记录路由器的连接节点和输入通道以及输入端口编号
		std::vector<int> value = {node,base+off,switch_port[my_switches[0]]};
		switch_input_channels[my_switches[0]].push_back(value);	
		//记录路由器的连接节点和输出通道以及输入端口编号
		std::vector<int> value1 = {node,switch_x_fchannel,switch_port[my_switches[0]]};
		switch_output_channels[my_switches[0]].push_back(value1);
		switch_x_fchannel++;
		left_node=my_switches[0];
	}
  }
  if(dim==1){
	if(location[1]<_dim_size[0]-1){
	    left_node=node+_dim_size[1]	;  
	}else{  //说明在1维度的左节点是（列）交换机
		//返回列交换机的id
		my_switches=_EdgeRouterGetSwitchIds(node);
		off=2*dim+1;
		//记录路由器的连接节点和输入通道
		std::vector<int> value = {node,base+off,switch_port[my_switches[1]]};
		switch_input_channels[my_switches[1]].push_back(value);	
		//记录路由器的连接节点和输出通道
		std::vector<int> value1 = {node,switch_x_fchannel,switch_port[my_switches[1]]};
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
  std::vector<int> my_switches(2,0);
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
	    right_node=node-_dim_size[1];	  
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

std::vector<int> HammingMesh::_EdgeRouterGetSwitchIds(int rtr_id){
	int i = rtr_id;
        //初始化一个空vector用于保存当前路由器的行列交换机信息
	std::vector<int> my_switches(2,0);
	std::vector<int> location(4,0);
	_IdToLocation(rtr_id,location);
	// 判断路由器是否在对角的位置
        if (((location[0] == 0 && location[1] == 0) ||
             (location[0] == _dim_size[1] - 1 && location[1] == 0) ||
             (location[0] == 0 && location[1] == _dim_size[0] - 1) ||
             (location[0] == _dim_size[1] - 1 && location[1] == _dim_size[0] - 1))) {
             // 找寻行交换机
             for (auto& intm : switch_loc) {
                    if (intm[3] == location[3]) {
                       switch_port[location[0]]++;
                       switch_to_routers[location[0]].push_back(i);	       
                       my_switches[0] = location[0];
                       break;
                     }
	     }
             // 找寻列交换机
	     for (auto& intm : switch_loc) {
                    if (intm[2] == location[2]) {
                       switch_port[location[0]]++;
                       switch_to_routers[location[0]].push_back(i);
                       my_switches[1] = location[0];
                       break;
		    }
	      }
	}
	//如果是列边缘，则寻找行交换机
	else if(location[0] == 0 || location[0] == _dim_size[1] - 1){
	     for (auto& intm : switch_loc) {
                    if (intm[3] == location[3]) {
                       switch_port[location[0]]++;
                       switch_to_routers[location[0]].push_back(i);
                       my_switches[0] = location[0];
                       break;
	             }
	      }
	}
	//如果是行边缘，则寻找列交换机
	else if(location[1] == 0 || location[1] == _dim_size[0] - 1){
	      for (auto& intm : switch_loc) {
                    if (intm[2] == location[2]) {
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



int HammingMesh::find_LeftChannel( int node, int other_node, int dim )
{
  std::vector<int> my_channels(2,0);
  if(other_node==0){
  // The base channel for a node is 2*_n*node
  int base = 2*2*node;
  // The offset for a left channel is 2*dim + 1
  int off  = 2*dim + 1;
  return ( base + off );
  }else if(other_node<_num_routers){
  // The base channel for a node is 2*_n*other_node
  int base = 2*2*other_node;
  // The offset for a left channel is 2*dim + 1
  int off  = 2*dim + 1;
  return ( base + off );	  
  }else{  //说明是找行 or 列交换机的输出通道
         //查表找dim维度的交换机（0-行 1-列）的输出通道
	  my_channels=Search_SOC(node);
	  return my_channels[dim];
  } 
}

int HammingMesh::find_RightChannel( int node, int other_node, int dim )
{
  std::vector<int> my_channels(2,0);
  if(other_node==0){
  // The base channel for a node is 2*_n*node
  int base = 2*2*node;
  // The offset for a right channel is 2*dim 
  int off  = 2*dim;
  return ( base + off );
  }else if(other_node<_num_routers){
  // The base channel for a node is 2*_n*other_node
  int base = 2*2*other_node;
  // The offset for a right channel is 2*dim 
  int off  = 2*dim;
  return ( base + off );  
  }else{  //说明是找行（0） or 列（1）交换机的输出通道
	  //查表找dim维度的交换机（0-行 1-列）的输出通道
	  my_channels=Search_SOC(node);
	  return my_channels[dim];
	  
  }
}

//路由器找左节点的函数
int HammingMesh::find_LeftNode( int node, int dim )  {
std::vector<int> my_switches(2,0);
std::vector<int> location(4,0);
_IdToLocation(node, location);
int left_node=0;
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
	    left_node=node+_dim_size[1]	;  
	}else{
	    //查表返回（列）交换机
		my_switches=Search_STR(node);
		left_node=my_switches[1];
	}
return left_node;	
}
}

//路由器找右节点的函数
int HammingMesh::find_RightNode( int node, int dim )  {
std::vector<int> my_switches(2,0);
std::vector<int> location(4,0);
_IdToLocation(node, location);
int right_node=0;
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
	    right_node=node-_dim_size[1];  
	}else{
	    //查表返回（列交换机）
		my_switches=Search_STR(node);
		right_node=my_switches[1];
	}
return right_node;	
}
}

//找寻中间板在拓扑中的位置信息
std::vector<int> midBoard(vector<int> cur_loc,vector<int> dest_loc ){
	vector<int> mid_hm_loc;//这个参数记录中间板的hm号
	if(cur_loc[2]==dest_loc[2]){
		mid_hm_loc[0]=cur_loc[3];
		mid_hm_loc[1]=dest_loc[4];
	}else if(cur_loc[3]==dest_loc[3]){
		mid_hm_loc[0]=dest_loc[3];
		mid_hm_loc[1]=cur_loc[4];
	}else{
		mid_hm_loc[0]=cur_loc[2];
		mid_hm_loc[1]=dest_loc[3];
	}
	return mid_hm_loc;
}

//注册路由函数
void HammingMesh::RegisterRoutingFunctions() {
      gRoutingFunctionMap["route_hammingmesh"]=&route_hammingmesh;
}

void route_hammingmesh( const Router *r, const Flit *f, int in_channel, 
			OutputSet *outputs, bool inject )
{
  //need 3 VCs for deadlock freedom
  //通过设置3条虚拟通道来避免死锁，因为ugal包括非最短和最短路由
  
  assert(gNumVCs==3);
  //用于存储即将进行输出操作或输出端口的信息
  outputs->Clear( );
  if(inject) {//判断是否正在进行数据包的注入
    int inject_vc= RandomInt(gNumVCs-1);//随机分配给注入通道一个虚拟通道
    outputs->AddRange(-1, inject_vc, inject_vc);
    return;
  }
  
  
  //计算每个hm板内路由器的数量
  int _hm_num_routers = _total_routers;
  //计算每个组内终端的数量
  int _hm_num_nodes = _hm_num_routers*1;//每个路由器连接的终端数为1
  //整个网络拓扑的终端数
  

  //获取目的终端
  int dest  = f->dest;
  //获取当前路由器的id
  int rID =  r->GetID();
  //计算当前路由器的hm板id
  int hm_ID = (int) (rID / _hm_num_routers);
  //计算目的路由器的hm板id
  int dest_hm_ID = (int) (dest/ _hm_num_nodes);
  //计算当前路由器在拓扑中的位置信息
  std::vector<int> cur_loc;
  _IdToLocation(rID,cur_loc);
  //计算目的路由器在拓扑中的位置信息
  std::vector<int> dest_loc;
  _IdToLocation(rID,dest_loc);
  //计算目的路由器的id	
  int dest_router=dest/gC;
  
  int debug = f->watch;
  int out_port = -1;
  int out_vc = 0;
  
  int intm_hm_ID;//中间hm板id
  int intm_rID;//中间路由器id
  //查看路由器有没有损坏
  if(debug){
    cout<<"At router "<<rID<<endl;
  }

    
    //目的节点和源节点在同一个hm板，使用xy路由
    if (dest_hm_ID == hm_ID) {
      f->ph = 2;//算当前在哪个路由阶段（ph），然后判断用哪个虚拟通道 
    } else {//选择一个中间板,intm记录中间板的hm板号
	std::vector<int> mid_hm_loc;
        mid_hm_loc = midBoard(cur_loc,dest_loc);
	intm_hm_ID = mid_hm_loc[1]*_dim_size[3]+mid_hm_loc[0]-1;
	//选择中间板上的左上角位置的路由器终端
	f->intm = (intm_hm_ID*_dim_size[0]*_dim_size[1])*1;
	if (debug){
	cout<<"Intermediate node "<<f->intm<<" hm_id "<<intm_hm_ID<<endl;
      }
      //1、如果源节点和目标节点在同一行上，它们将在原板上进行自适应路由到最接近目标的边缘（西或东），然后通过交换机路由到最接近目标的目标板端口
      //2、如果在同一列上与1类似    
      //3、如果原板和目标板位于不同的行和列上，数据包必须通过一个中间板进行转发，该中间板必须与原版的行相同，并且与目标板的列相同，路径选择是自适应的和最小的，数据包穿过两个交换机，每个维度一个
      //4、为了保证板内的死锁自由，数据包使用north-last路由进行转发。板间采用增加3个虚拟通道的方式来避免死锁
      //首先计算出中间板的位置，选择下一个输出端口（输出通道）
      //select a random node
      //random intermediate are in the same group, use minimum routing
      //随机选择的中间节点和目的节点位于同一个组内，使用最短路径路由
      
      if(dest_hm_ID == intm_hm_ID){//说明目标板和源板在同一行或者同一列
	f->ph = 1;
      } else {//否则说明目标板和源板在不同行和不同列
	f->ph = 0;
      }
    }

  //port assignement based on the phase,基于不同的阶段分配端口
  if(f->ph == 0){
    out_port = hammingmesh_ugal_port(rID, f->src, f->intm, f->dest);
  } else if(f->ph == 1){
    out_port = hammingmesh_ugal_port(rID, f->src, f->intm, f->dest);
  } else if(f->ph == 2){
    out_port = hammingmesh_xy_port(rID, dest_router, dest);
  } else {
    assert(false);
  }

  //vc assignemnt based on phase,基于不同的阶段分配虚拟通道
  out_vc = f->ph;

  outputs->AddRange( out_port, out_vc, out_vc );
}

int hammingmesh_xy_port(int cur_router, int dest_router, int dest){
  const int POSITIVE_X = 0 ;
  const int NEGATIVE_X = 1 ;
  const int POSITIVE_Y = 2 ;
  const int NEGATIVE_Y = 3 ;
  //计算当前和目的路由器在拓扑中的位置
  std::vector<int> cur_router_loc;
  _IdToLocation(cur_router,cur_router_loc);
  
  std::vector<int> dest_router_loc;
  _IdToLocation(dest_router,dest_router_loc);
  

   // Dimension-order Routing: x , y
  if (cur_router_loc[0] < dest_router_loc[0]) {
    return gC + POSITIVE_X ;
  }
  if (cur_router_loc[0] > dest_router_loc[0]) {
    return gC + NEGATIVE_X ;
  }
  if (cur_router_loc[1] < dest_router_loc[1]) {
    return gC + POSITIVE_Y ;
  }
  if (cur_router_loc[1] > dest_router_loc[1]) {
    return gC + NEGATIVE_Y ;
  } 
  //At the last hop,如果当前路由器已到达目标路由器
  if((cur_router_loc[0] = dest_router_loc[0])&&(cur_router_loc[1] = dest_router_loc[1])){
    
    return dest % gC;
  }
	
}

//查找switch_output_channels,返回当前行/列交换机连接特定路由器的输出端口
int Search_OutPort_SOC(int cur_router, int sID){
     int out_port;
	 bool found =false;
	 for (auto pair:switch_input_channels){
		 if(pair.first == sID){
	                for (auto v:pair.second){
		              if(v[0] == cur_router){
		                out_port=v[2];
		                found=true;
		                break;
		              }
		              if (found) break;
	                }
		 }
	}
	return out_port;
	
}

//能调用这个函数的数据包，至少会跨越一个交换机
//计算板间数据包的下一个输出端口,板间采用类似dragongfly的ugal路由,传入的参数为当前路由器id，源终端节点和目的终端节点
int hammingmesh_ugal_port(int cur_router, int source, int intm, int dest){
  //计算源、当前、中间和目的路由器在拓扑中的位置
  int source_router= source / gC;
  std::vector<int> source_router_loc;
  _IdToLocation(source_router,source_router_loc);
	
  std::vector<int> cur_router_loc;
  _IdToLocation(cur_router,cur_router_loc);
  
  int intm_router= intm / gC;
  std::vector<int> intm_router_loc;
  _IdToLocation(intm_router,intm_router_loc);
  
  int dest_router= dest / gC;
  std::vector<int> dest_router_loc;
  _IdToLocation(dest_router,dest_router_loc);
  
  int _hm_num_routers= _dim_size[0]*_dim_size[1];//计算出每个板子内的路由器数量
  int _hm_num_nodes =_hm_num_routers*gC;//gC是每个路由器的终端数

  int out_port = -1;
  int source_hm_ID = int(source / _hm_num_nodes);//计算出当前路由器的组id
  int cur_hm_ID = int(cur_router / _hm_num_routers);//计算出当前路由器的组id
  int intm_hm_ID = int(intm / _hm_num_nodes);//计算出中间板路由器的组id
  int dest_hm_ID = int(dest / _hm_num_nodes);//计算出目的路由器的组id
  int hm_output=-1;
  int hm_RID=-1;
  
  //which router within this group the packet needs to go to
  //假设现在有一个流是0->4的路由器
  //数据包需要去往当前所在路由器所在组内的哪个路由器
  //如果只跨越了一个交换机(只有两个板子)
  if (dest_hm_ID == intm_hm_ID) {
	  if(source_router_loc[3]==intm_router_loc[3]){//如果两个板子是在同一行
		  if(cur_router < _total_routers){//对于路由器来说
	               if(cur_hm_ID == source_hm_ID){//如果当前路由器的所在板子是原板
	               int hmOut_router=(source_router_loc[1]+1)*_dim_size[1]-1;
			   if(hmOut_router=cur_router){
				out_port=  gC+0;//走行出板路由器的正向端口
			   }else{
				out_port=hammingmesh_xy_port(cur_router,hmOut_router,-1);
			   }       
		       }else if(cur_hm_ID == intm_hm_ID){//如果当前路由器的所在板子是目标板/中间板
			int hmIn_router=   intm_hm_ID*_hm_num_routers;
			    if(dest_router == hmIn_router){
				    //At the last hop
				    out_port= dest % gC;
			    }else{
				 out_port=hammingmesh_xy_port(hmIn_router,dest_router,-1);   
			    }
		       }
		  }else{//对于行交换机来说,通过查表找端口
	          out_port=Search_OutPort_SOC(cur_router, intm_hm_ID*_hm_num_routers);
                  }
	    
          }else if(source_router_loc[2]==intm_router_loc[2]){//如果两个板子是在同一列
		  if(cur_router < _total_routers){//对于路由器来说
	               if(cur_hm_ID == source_hm_ID){//如果当前路由器的所在板子是原板
	               int hmOut_router=(_dim_size[0]-1)*_dim_size[1]+source_router_loc[0];
			   if(hmOut_router == cur_router){
				out_port=gC+2;//走列出板路由器的正向端口
			   }else{
				out_port=hammingmesh_xy_port(cur_router,hmOut_router,-1);
			   }       
		       }else if(cur_hm_ID == dest_hm_ID){//如果当前路由器的所在板子是目标板/中间板
			int hmIn_router=   intm_hm_ID*_hm_num_routers;
			    if(dest_router == hmIn_router){
				    //At the last hop
				    out_port= dest % gC;
			    }else{
				 out_port=hammingmesh_xy_port(hmIn_router,dest_router,-1);   
			    }       
		       }
		  }else{//对于列交换机来说,通过查表找端口
	          out_port=Search_OutPort_SOC(cur_router, intm_hm_ID*_hm_num_routers);
                  }
	  }
  }else {//如果跨越了两个交换机(三个板子)
              if(cur_router < _total_routers){//对于路由器来说
	               if(cur_hm_ID == source_hm_ID){//如果当前路由器的所在板子是原板
	               int hmOut_router=(source_router_loc[1]+1)*_dim_size[1]-1;
			   if(hmOut_router=cur_router){
				out_port=  gC+0;//走行出板路由器的正向端口
			   }else{
				out_port=hammingmesh_xy_port(cur_router,hmOut_router,-1);
			   }       
		       }else if(cur_hm_ID==intm_hm_ID){//如果当前路由器的所在板子是中间板
			int hmIn_router=   intm_hm_ID*_hm_num_routers;
			    if(dest_router == hmIn_router){
				    //走列交换机，左上角的路由器是gC+3号端口连接的列交换机
				    out_port=gC+3;
			    }else{
				 out_port=hammingmesh_xy_port(hmIn_router,dest_router,-1);   
			    }
		       }else{//如果当前路由器的所在板子是目的板
			int hmIn_router=   intm_hm_ID*_hm_num_routers;
			    if(dest_router == hmIn_router){
				    //At the last hop
				    out_port= dest % gC;
			    }else{
				 out_port=hammingmesh_xy_port(hmIn_router,dest_router,-1);   
			    }              
		       }
	      }else if((_total_routers < cur_router )&&(cur_router< switch_y_f_id )){//对于行交换机来说,通过查表找端口
	              out_port=Search_OutPort_SOC(cur_router, intm_hm_ID*_hm_num_routers);
              }else{//对于列交换机来说,通过查表找端口
	              out_port=Search_OutPort_SOC(cur_router, dest_hm_ID*_hm_num_routers);
	      }
    
    }
  assert(out_port!=-1);
  //返回数据包的输出端口
  return out_port;

}


//查表switch_to_routers，返回当前路由器的行或者列交换机(至少有1个)
std::vector<int> HammingMesh::Search_STR(int rID){
	std::vector<int> my_switches(2,0);
	int flag=0;
	for (auto pair:switch_to_routers){
	    for (auto num:pair.second){
		if(num==rID){
		    if(pair.first<switch_y_f_id){
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
std::vector<int> HammingMesh::Search_SIC(int rID){
	std::vector<int> my_channels(2,0);
	for (auto pair:switch_input_channels){
	    for (auto v:pair.second){
		    if(v[0]==rID){
		      if(pair.first<switch_y_f_id){
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
std::vector<int> HammingMesh::Search_SOC(int rID){
	std::vector<int> my_channels(2,0);
	for (auto pair:switch_input_channels){
	    for (auto v:pair.second){
		    if(v[0]==rID){
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


 


