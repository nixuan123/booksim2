/*
  Copyright (c) 2007-2015, Trustees of The Leland Stanford Junior University
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  Redistributions of source code must retain the above copyright notice, this list
  of conditions and the following disclaimer.
  Redistributions in binary form must reproduce the above copyright notice, this 
  list of conditions and the following disclaimer in the documentation and/or 
  other materials provided with the distribution.
  Neither the name of the Stanford University nor the names of its contributors 
  may be used to endorse or promote products derived from this software without 
  specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "booksim.hpp"
#include <vector>
#include <sstream>

#include "dragonfly.hpp"
#include "random_utils.hpp"
#include "misc_utils.hpp"
#include "globals.hpp"

#define DRAGON_LATENCY

int gP, gA, gG;

//calculate the hop count between src and estination
//计算源节点和目的节点之间的跳数
int dragonflynew_hopcnt(int src, int dest) 
{
  int hopcnt;//设置跳数变量
  int dest_grp_ID, src_grp_ID;//设置目的组id和源组id 
  int src_hopcnt, dest_hopcnt;
  int src_intm, dest_intm;
  int grp_output, dest_grp_output;//源组输出端口和目的组输出端口
  int grp_output_RID;//源组输出端口所在的路由器id

  int _grp_num_routers= gA;//将gA变量赋值给每组的路由器/交换机数量
  int _grp_num_nodes =_grp_num_routers*gP;//每组的终端数量。gP代表每个路由器/交换机上所连接的终端数量
  
  dest_grp_ID = int(dest/_grp_num_nodes);//int(dest/_grp_num_routers/_p);
  src_grp_ID = int(src / _grp_num_nodes);
  
  //source and dest are in the same group, either 0-1 hop
  //源节点和目的节点在同一个组内，要么是0跳要么是1跳，自己或者是相邻的另一个节点
  if (dest_grp_ID == src_grp_ID) {
    if ((int)(dest / gP) == (int)(src /gP))
      hopcnt = 0;
    else
      hopcnt = 1;
    
  } else {//源和目的节点不在同一个组内
    //source and dest are not in the same group
    //find the number of hops in the source group
    //find the number of hops in the dest group
    if (src_grp_ID > dest_grp_ID)  {
      grp_output = dest_grp_ID;
      dest_grp_output = src_grp_ID - 1;
    }
    else {
      grp_output = dest_grp_ID - 1;
      dest_grp_output = src_grp_ID;
    }
    grp_output_RID = ((int) (grp_output / (gP))) + src_grp_ID * _grp_num_routers;
    src_intm = grp_output_RID * gP;

    grp_output_RID = ((int) (dest_grp_output / (gP))) + dest_grp_ID * _grp_num_routers;
    dest_intm = grp_output_RID * gP;

    //hop count in source group
    if ((int)( src_intm / gP) == (int)( src / gP ) )
      src_hopcnt = 0;
    else
      src_hopcnt = 1; 

    //hop count in destination group
    if ((int)( dest_intm / gP) == (int)( dest / gP ) ){
      dest_hopcnt = 0;
    }else{
      dest_hopcnt = 1;
    }

    //tally
    hopcnt = src_hopcnt + 1 + dest_hopcnt;
  }

  return hopcnt;  
}


//packet output port based on the source, destination and current location
//根据源节点、目的节点和当前位置确定数据包的输出端口
int dragonfly_port(int rID, int source, int dest){
  int _grp_num_routers= gA;//计算出每个组内的路由器数量，我们现在考虑_grp_num_routers=a=2的情况，即组内只存在两个交换机
  int _grp_num_nodes =_grp_num_routers*gP;//gP是每个路由器的终端数=_p，由于a=1，所以gP=1

  int out_port = -1;
  int grp_ID = int(rID / _grp_num_routers);//计算出当前路由器的组id，假设当前路由器id=0，那么其组id=grp_id=0
  int dest_grp_ID = int(dest/_grp_num_nodes);//计算出目的路由器的组id，假设要去往的目的路由器id=4，那么其组id=dest_grp_id=2
  int grp_output=-1;
  int grp_RID=-1;
  
  //which router within this group the packet needs to go to
  //假设现在有一个流是0->4的路由器
  //数据包需要去往当前所在路由器所在组内的哪个路由器
  //如果目的路由器的组id和当前路由器的组id一致
  if (dest_grp_ID == grp_ID) {
    //则去往的目的地是自己组的路由器终端
    grp_RID = int(dest / gP);
  } else {//如果不一致,说明不是在同一个组：1、若当前路由器所在的组id大于目的路由器的组id
    if (grp_ID > dest_grp_ID) {
      //将目的路由器的组id赋值给输出组变量
      grp_output = dest_grp_ID;
    } else {
      //2、若当前路由器所在的组id小于目的路由器的组id，0->4
      //将目的路由器的组id-1再赋值给输出组变量
      grp_output = dest_grp_ID - 1;//grp_output=2-1=1
    } 
    //统一计算需要发往当前组内的路由器id
    grp_RID = int(grp_output /gP) + grp_ID * _grp_num_routers;//grp_ID * _grp_num_routers是当前路由器所在组的起始路由器id，grp_output/gP是偏移量,grp_RID=1
  }

  //At the last hop，在路由函数的最后一跳时
  if (dest >= rID*gP && dest < (rID+1)*gP) {//如果目的终端在当前路由器上
    //注入的输出端口为dest对gP取余
    out_port = dest%gP;
  } else if (grp_RID == rID) {//如果为了到达目的终端要发往的组内id等于当前路由器的id
    //At the optical link
    out_port = gP + (gA-1) + grp_output %(gP);
  } else {
    //need to route within a group,需要在组内进行路由
    assert(grp_RID!=-1);

    if (rID < grp_RID){
      out_port = (grp_RID % _grp_num_routers) - 1 + gP;
    }else{
      out_port = (grp_RID % _grp_num_routers) + gP;
    }
  }  
 
  assert(out_port!=-1);
  //返回数据包的输出端口
  return out_port;
}


DragonFlyNew::DragonFlyNew( const Configuration &config, const string & name ) :
  Network( config, name )
{

  _ComputeSize( config );
  _Alloc( );
  _BuildNet( config );
}

void DragonFlyNew::_ComputeSize( const Configuration &config )
{

  // LIMITATION，先考虑p，a，h，g=(1,2,1,3)的dragonfly拓扑
  //  -- only one dimension between the group//组与组之间只有一个维度
  // _n == # of dimensions within a group//组内的维度系数,默认为1
  // _p == # of processors within a router//路由器内的处理器数量，现取1
  // inter-group ports : _p//组件端口，1
  // terminal ports : _p//每个路由器连接终端的端口数量，1
  // intra-group ports : 2*_p - 1//组内端口，1
  _p = config.GetInt( "k" );	// # of ports in each switch//每个交换机的端口数，1
  _n = config.GetInt( "n" );


  assert(_n==1);//确保_n的值为1，保证维度为1
  // dimension

  if (_n == 1)
    _k = _p + _p + 2*_p  - 1;//1+1+2-1=3，代表每个路由器/交换机连接的端口数，local（组内连接和终端连接）+global
  else
    _k = _p + _p + 2*_p;

  
  // FIX...
  gK = _p; gN = _n;

  // with 1 dimension, total of 2p routers per group
  // N = 2p * p * (2p^2 + 1)//N=2*1*3=6,代表整个dragonfly拓扑的路由器/交换机的数量
  // a = # of routers per group//每个组包含的路由器数量
  //   = 2p (if n = 1)//=2
  //   = p^(n) (if n > 2)
  //  g = # of groups//拓扑中组的数量
  //    = a * p + 1//2*1+1=3，即3组
  // N = a * p * g;//N=2*1*3=6
  
  if (_n == 1)
    _a = 2 * _p;//_a=2,每个组内的路由器数量
  else
    _a = powi(_p, _n);

  _g = _a * _p + 1;//2*1+1=3,"ah+1",表示整个拓扑的组数量
  _nodes   = _a * _p * _g;//2*1*3=6，表示整个拓扑的终端的数量

  _num_of_switch = _nodes / _p;//6/1=6，表示整个拓扑的交换机/路由器的数量，等于拓扑中的终端数量除以每个路由器连接的终端数_p。
  _channels = _num_of_switch * (_k - _p); //通道数量6*（3-1）=12，每个路由器都有输入输出通道
  _size = _num_of_switch;//拓扑的大小（表示整个拓扑中的交换机/路由器数量）


  
  gG = _g;//表示整个拓扑的组数量，这里为3
  gP = _p;//表示每个交换机/路由器连接的终端数量
  gA = _a;//表示每个组内的路由器数量
  _grp_num_routers = gA;
  _grp_num_nodes =_grp_num_routers*gP;//计算出dragonfly拓扑中的终端数量

}

void DragonFlyNew::_BuildNet( const Configuration &config )
{

  int _output=-1;
  int _input=-1;
  int _dim_ID=-1;
  int _num_ports_per_switch=-1;
  int c;

  ostringstream router_name;



  cout << " Dragonfly " << endl;
  cout << " p = " << _p << " n = " << _n << endl;
  cout << " each switch - total radix =  "<< _k << endl;
  cout << " # of switches = "<<  _num_of_switch << endl;
  cout << " # of channels = "<<  _channels << endl;
  cout << " # of nodes ( size of network ) = " << _nodes << endl;
  cout << " # of groups (_g) = " << _g << endl;
  cout << " # of routers per group (_a) = " << _a << endl;

  //根据拓扑中的节点数量来构建dragonfly拓扑
  for ( int node = 0; node < _num_of_switch; ++node ) {
    // ID of the group
    int grp_ID;
    grp_ID = (int) (node/_a);//_a代表每个组内的路由器数量，这段代码是为了计算当前路由器id所在的组号，假设id=4，则grp_ID=2
    router_name << "router";
    
    router_name << "_" <<  node ;
    //_k代表每个路由器上的端口数量，包括终端和非终端端口
    _routers[node] = Router::NewRouter( config, this, router_name.str( ), 
					node, _k, _k );
    _timed_modules.push_back(_routers[node]);

    router_name.str("");
    //_p代表终端数，设置本地输入链路
    for ( int cnt = 0; cnt < _p; ++cnt ) {
      //c=4+0=4
      c = _p * node +  cnt;
      _routers[node]->AddInputChannel( _inject[c], _inject_cred[c] );

    }
    //设置本地输出链路
    for ( int cnt = 0; cnt < _p; ++cnt ) {
      //c=4+0=4
      c = _p * node +  cnt;
      _routers[node]->AddOutputChannel( _eject[c], _eject_cred[c] );

    }

    // add OUPUT channels，添加输出通道
    // _k == # of processor per router
    //  need 2*_k routers  --thus, 
    //  2_k-1 outputs channels within group
    //  _k-1 outputs for intra-group

    //

    if (_n > 1 )  { cout << " ERROR: n>1 dimension NOT supported yet... " << endl; exit(-1); }

    //********************************************
    //   connect OUTPUT channels
    //********************************************
    // add intra-group output channel，添加组内输出通道
    for ( int dim = 0; dim < _n; ++dim ) {
      for ( int cnt = 0; cnt < (2*_p -1); ++cnt ) {
	//对于id=4的路由器，output=（2*1-1+1）*1*4+（2*1-1）*0+0=8;id=5时，output=10
	_output = (2*_p-1 + _p) * _n  * node + (2*_p-1) * dim  + cnt;

	_routers[node]->AddOutputChannel( _chan[_output], _chan_cred[_output] );

#ifdef DRAGON_LATENCY
	_chan[_output]->SetLatency(10);
	_chan_cred[_output]->SetLatency(10);
#endif
      }
    }

    // add inter-group output channel
    // 添加组间输出通道
    for ( int cnt = 0; cnt < _p; ++cnt ) {
      // 对于id=4的路由器，_output=2*4+1+0=9;id=5时，_output=11
      _output = (2*_p-1 + _p) * node + (2*_p - 1) + cnt;

      //      _chan[_output].global = true;
      _routers[node]->AddOutputChannel( _chan[_output], _chan_cred[_output] );
#ifdef DRAGON_LATENCY
      _chan[_output]->SetLatency(100);
      _chan_cred[_output]->SetLatency(100);
#endif
    }

    //接下来添加每个路由器的输入链路
    //********************************************
    //   connect INPUT channels
    //********************************************
    // # of non-local nodes //非本地节点
    _num_ports_per_switch = (_k - _p);//3-1=2


    // intra-group GROUP channels，组内群组通道，来自组内的local link
    for ( int dim = 0; dim < _n; ++dim ) {
      //id=4时，_dim_ID=4/1=4;id=5时，_dim_ID=5
      _dim_ID = ((int) (node / ( powi(_p, dim))));



      // NODE ID withing group
      //id=4时，该节点在组内的相对位置为：4%2=0;id=5时，此值为：1
      _dim_ID = node % _a;




      for ( int cnt = 0; cnt < (2*_p-1); ++cnt ) {

	if ( cnt < _dim_ID)  {
         //id=5时，_input=2*2*2-1*2+1*2+0=8
	  _input = 	grp_ID  * _num_ports_per_switch * _a - 
	    (_dim_ID - cnt) *  _num_ports_per_switch + 
	    _dim_ID * _num_ports_per_switch + 
	    (_dim_ID - 1);
	}
	else {
           //id=4时，_input=2*2*2+0*2+(0-0+1)*2+0=10
	   //id=2时，_input=1*2*2+0*2+(0-0+1)*2+0=6
	  _input =  grp_ID * _num_ports_per_switch * _a + 
	    _dim_ID * _num_ports_per_switch + 
	    (cnt - _dim_ID + 1) * _num_ports_per_switch + 
	    _dim_ID;
			
	}

	if (_input < 0) {
	  cout << " ERROR: _input less than zero " << endl;
	  exit(-1);
	}


	_routers[node]->AddInputChannel( _chan[_input], _chan_cred[_input] );
      }
    }


    // add INPUT channels -- "optical" channels connecting the groups
    //添加输入通道，来自其他组的global link
    int grp_output;

    for ( int cnt = 0; cnt < _p; ++cnt ) {
      //	   _dim_ID
      //id=4时，grp_output=0*1+0=0;id=5时，此值为1+0=1
      grp_output = _dim_ID* _p + cnt;

      if ( grp_ID > grp_output)   {
        //id=4时，_input = 0+(2-1)*(1)+1+2-1=3;id=5时，_input=1*2*2+1*1+1+1=7
	_input = (grp_output) * _num_ports_per_switch * _a    +   		// starting point of group
	  (_num_ports_per_switch - _p) * (int) ((grp_ID - 1) / _p) +      // find the correct router within grp
	  (_num_ports_per_switch - _p) + 					// add offset within router
	  grp_ID - 1;	
      } else {
        //2*2*2+0+2-1+0=9
	_input = (grp_output + 1) * _num_ports_per_switch * _a    + 
	  (_num_ports_per_switch - _p) * (int) ((grp_ID) / _p) +      // find the correct router within grp
	  (_num_ports_per_switch - _p) +
	  grp_ID;	
      }

      _routers[node]->AddInputChannel( _chan[_input], _chan_cred[_input] );
    }

  }

  cout<<"Done links"<<endl;
}


int DragonFlyNew::GetN( ) const
{
  return _n;
}

int DragonFlyNew::GetK( ) const
{
  return _k;
}

void DragonFlyNew::InsertRandomFaults( const Configuration &config )
{
 
}

double DragonFlyNew::Capacity( ) const
{
  return (double)_k / 8.0;
}

void DragonFlyNew::RegisterRoutingFunctions(){

  gRoutingFunctionMap["min_dragonflynew"] = &min_dragonflynew;
  gRoutingFunctionMap["ugal_dragonflynew"] = &ugal_dragonflynew;
}


void min_dragonflynew( const Router *r, const Flit *f, int in_channel, 
		       OutputSet *outputs, bool inject )
{
  outputs->Clear( );

  if(inject) {
    int inject_vc= RandomInt(gNumVCs-1);
    outputs->AddRange(-1, inject_vc, inject_vc);
    return;
  }

  int _grp_num_routers= gA;

  int dest  = f->dest;//从flit中获取到目的终端
  int rID =  r->GetID(); //从router中获取到id
  //求出路由器所在的组ID
  int grp_ID = int(rID / _grp_num_routers); 
  int debug = f->watch;
  int out_port = -1;
  int out_vc = 0;
  int dest_grp_ID=-1;

  if ( in_channel < gP ) {
    out_vc = 0;
    f->ph = 0;
    if (dest_grp_ID == grp_ID) {
      f->ph = 1;
    }
  } 


  out_port = dragonfly_port(rID, f->src, dest);

  //optical dateline
  if (out_port >=gP + (gA-1)) {
    f->ph = 1;
  }  
  
  out_vc = f->ph;
  if (debug)
    *gWatchOut << GetSimTime() << " | " << r->FullName() << " | "
	       << "	through output port : " << out_port 
	       << " out vc: " << out_vc << endl;
  outputs->AddRange( out_port, out_vc, out_vc );
}


//Basic adaptive routign algorithm for the dragonfly
//dragonfly网路的基本自适应路由算法
void ugal_dragonflynew( const Router *r, const Flit *f, int in_channel, 
			OutputSet *outputs, bool inject )
{
  //need 3 VCs for deadlock freedom
  //通过设置3条虚拟通道来避免死锁，因为ugal包括非最短和最短路由

  assert(gNumVCs==3);
  //用于存储即将进行输出操作或输出端口的信息
  outputs->Clear( );
  if(inject) {//判断是否正在进行数据包的注入
    int inject_vc= RandomInt(gNumVCs-1);
    outputs->AddRange(-1, inject_vc, inject_vc);
    return;
  }
  
  //this constant biases the adaptive decision toward minimum routing
  //negative value woudl biases it towards nonminimum routing
  //下面这个参数会影响自适应路由算法的决策过程，使其倾向于选择最短路径进行数据包的路由，最短路径
  //通常指的是经过最少数量跳点的路径，如果这个值是负值，那么路由算法的决策过程
  int adaptive_threshold = 30;
  //计算每个组内路由器的数量
  int _grp_num_routers= gA;
  //计算每个组内终端的数量
  int _grp_num_nodes =_grp_num_routers*gP;//gP指的是每个路由器所连接的终端数量
  //整个网络拓扑的终端数
  int _network_size =  gA * gP * gG;//gG是指整个拓扑的组数

  //获取目的路由器的id
  int dest  = f->dest;
  //获取当前路由器的id
  int rID =  r->GetID();
  //计算当前路由器的组id
  int grp_ID = (int) (rID / _grp_num_routers);
  //计算目的路由器的组id
  int dest_grp_ID = int(dest/ _grp_num_nodes);

  int debug = f->watch;
  int out_port = -1;
  int out_vc = 0;
  int min_queue_size;//设置最短路径队列的size
  int nonmin_queue_size;//设置非最短路径队列的size
  int intm_grp_ID;
  int intm_rID;
  //查看路由器有没有损坏
  if(debug){
    cout<<"At router "<<rID<<endl;
  }
  int min_router_output, nonmin_router_output;
  
  //at the source router, make the adaptive routing decision
  //在源路由器进行自适应路由决策
  if ( in_channel < gP )   {
    //dest are in the same group, only use minimum routing
    //目的节点和源节点在同一个组，使用最短路径路由
    if (dest_grp_ID == grp_ID) {
      f->ph = 2;
    } else {//否则是非最短路径路由，随机选择一个节点
      //select a random node
      f->intm =RandomInt(_network_size - 1);//随机选择中间板上的节点id
      intm_grp_ID = (int)(f->intm/_grp_num_nodes);//计算出中间板的组id
      if (debug){
	cout<<"Intermediate node "<<f->intm<<" grp id "<<intm_grp_ID<<endl;
      }
      
      //random intermediate are in the same group, use minimum routing
      //随机选择的中间节点和当前节点位于同一个组内，使用最短路径路由
      if(grp_ID == intm_grp_ID){
	f->ph = 1;
      } else {
	//congestion metrics using queue length, obtained by GetUsedCredit()
	//使用队列长度的拥塞度量指标，通过GetUsedCredit()获得
	min_router_output = dragonfly_port(rID, f->src, f->dest); 
      	min_queue_size = max(r->GetUsedCredit(min_router_output), 0) ; 

      
	nonmin_router_output = dragonfly_port(rID, f->src, f->intm);
	nonmin_queue_size = max(r->GetUsedCredit(nonmin_router_output), 0);

	//congestion comparison, could use hopcnt instead of 1 and 2
	if ((1 * min_queue_size ) <= (2 * nonmin_queue_size)+adaptive_threshold ) {	  
	  if (debug)  cout << " MINIMAL routing " << endl;
	  f->ph = 1;
	} else {
	  f->ph = 0;
	}
      }
    }
  }

  //transition from nonminimal phase to minimal
  if(f->ph==0){
    intm_rID= (int)(f->intm/gP);
    if( rID == intm_rID){
      f->ph = 1;
    }
  }

  //port assignement based on the phase
  if(f->ph == 0){
    out_port = dragonfly_port(rID, f->src, f->intm);
  } else if(f->ph == 1){
    out_port = dragonfly_port(rID, f->src, f->dest);
  } else if(f->ph == 2){
    out_port = dragonfly_port(rID, f->src, f->dest);
  } else {
    assert(false);
  }

  //optical dateline
  if (f->ph == 1 && out_port >=gP + (gA-1)) {
    f->ph = 2;
  }  

  //vc assignemnt based on phase
  out_vc = f->ph;

  outputs->AddRange( out_port, out_vc, out_vc );
}
