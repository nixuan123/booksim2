// $Id$

/*
 Copyright (c) 2007-2015, Trustees of The Leland Stanford Junior University
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this 
 list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this
 list of conditions and the following disclaimer in the documentation and/or
 other materials provided with the distribution.

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

// ----------------------------------------------------------------------
//
// CMesh: Network with <Int> Terminal Nodes arranged in a concentrated
//        mesh topology
//
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
//  $Author: jbalfour $
//  $Date: 2007/06/28 17:24:35 $
//  $Id$
//  Modified 11/6/2007 by Ted Jiang
//  Now handeling n = most power of 2: 16, 64, 256, 1024
// ----------------------------------------------------------------------
#include "booksim.hpp"
#include <vector>
#include <sstream>
#include <cassert>
#include "random_utils.hpp"
#include "misc_utils.hpp"
#include "cmesh.hpp"

int CMesh::_cX = 0 ;
int CMesh::_cY = 0 ;
int CMesh::_memo_NodeShiftX = 0 ;
int CMesh::_memo_NodeShiftY = 0 ;
int CMesh::_memo_PortShiftY = 0 ;

CMesh::CMesh( const Configuration& config, const string & name ) 
  : Network(config, name) 
{
  _ComputeSize( config );
  _Alloc();
  _BuildNet(config);
}
//在一个全局路由函数映射表gRoutingFunctionMap中注册多个路由函数。
//将字符串标识符映射到实际的路由函数指针，一遍再模拟过程中根据标识符
//调用相应的路由逻辑
void CMesh::RegisterRoutingFunctions() {
  //通过字符串键映射到不同的路由函数，&运算符用于获取函数的地址，这些函数被注册到
  //gRoutingFunctionMap中，对应于上面的字符串键
  gRoutingFunctionMap["dor_cmesh"] = &dor_cmesh;
  gRoutingFunctionMap["dor_no_express_cmesh"] = &dor_no_express_cmesh;
  gRoutingFunctionMap["xy_yx_cmesh"] = &xy_yx_cmesh;
  gRoutingFunctionMap["xy_yx_no_express_cmesh"]  = &xy_yx_no_express_cmesh;
}
//根据Configuration对象提供的参数计算网络的大小和结构。
void CMesh::_ComputeSize( const Configuration &config ) {
  //从配置文件中读取
  int k = config.GetInt( "k" );//获取维度的宽度
  int n = config.GetInt( "n" );//获取网格的维度数
  //断言不能大于2，因为当前的实现仅支持最多两维的网格拓扑
  assert(n <= 2); // broken for n > 2
  int c = config.GetInt( "c" );//获取每个路由器的集中度，即终端节点数
  assert(c == 4); // broken for c != 4//当前实现只支持集中度为4的网格
  ostringstream router_name;
  //how many routers in the x or y direction
  //分别获取网络在X和y方向上的路由器数量，//x和y方向上的路由器数量必须相同，因为当前实现不支持非对称拓扑
  _xcount = config.GetInt("x");
  _ycount = config.GetInt("y");
  assert(_xcount == _ycount); // broken for asymmetric topologies
  //configuration of hohw many clients in X and Y per router
  //每个路由器上x和y维上的主机数量
  _xrouter = config.GetInt("xr");
  _yrouter = config.GetInt("yr");
  assert(_xrouter == _yrouter); // broken for asymmetric concentration
  //给全局常量赋值
  gK = _k = k ;//每个维度的路由器数量，假设为3
  gN = _n = n ;//维度的值，假设为2
  gC = _c = c ;//每个路由器的终端数量.假设为2

  assert(c == _xrouter*_yrouter);
  //计算网络中的节点总数=c*_k的_n次方
  _nodes    = _c * powi( _k, _n); // Number of nodes in network、18
  _size     = powi( _k, _n);      // Number of routers in network、9
  _channels = 2 * _n * _size;     // Number of channels in network、36

  _cX = _c / _n ;   // Concentration in X Dimension 2/2=1
  _cY = _c / _cX ;  // Concentration in Y Dimension 2/1=2

  //
  _memo_NodeShiftX = _cX >> 1 ;//计算节点右移一位的值、2
  //log_two表示以2为底的对数
  _memo_NodeShiftY = log_two(gK * _cX) + ( _cY >> 1 ) ;
  _memo_PortShiftY = log_two(gK * _cX)  ;

}

void CMesh::_BuildNet( const Configuration& config ) {
  //计算路由器在网格中的位置
  int x_index ;
  int y_index ;

  //standard trace configuration 
  if(gTrace){
    cout<<"Setup Finished Router"<<endl;
  }

  //latency type, noc or conventional network
  //声明一个变量来决定是否使用NoC的延迟模型，这个值基于配置对象中的一个整数值决定
  bool use_noc_latency;
  use_noc_latency = (config.GetInt("use_noc_latency")==1);
  
  //用于构建路由器的名称
  ostringstream name;
  // The following vector is used to check that every
  //  processor in the system is connected to the network
  //初始化一个布尔向量，其大小由_nodes决定，所有元素初始为false，这个
  //向量用于跟踪系统中的每个处理器是否都连接到了网络
  vector<bool> channel_vector(_nodes, false) ;
  
  //
  // Routers and Channel
  //开始一个循环，用于遍历网络中的所有节点
  for (int node = 0; node < _size; ++node) {

    // Router index derived from mesh index
    //计算该节点在mesh中的坐标
    y_index = node / _k ;
    x_index = node % _k ;

    //计算每个路由器的输入和输出端口的数量，每多一维就增加两个方向的端口
    const int degree_in  = 2 *_n + _c ;
    const int degree_out = 2 *_n + _c ;

    //构建当前节点的路由器名称，创建一个新的路由器对象，并将其
    //添加到_timed_modules容器中，然后清空name字符串以备下一个路由器使用
    name << "router_" << y_index << '_' << x_index;
    _routers[node] = Router::NewRouter( config, 
					this, 
					name.str(), 
					node,
					degree_in,
					degree_out);
    _timed_modules.push_back(_routers[node]);
    name.str("");

    //端口标号
    // Port Numbering: as best as I can determine, the order in
    //  which the input and output channels are added to the
    //  router determines the associated port number that must be
    //  used by the router. Output port number increases with 
    //  each new channel
    //

    //
    // Processing node channels
    //构建每个路由器到本地处理节点的通道，通过两层嵌套for循环遍历所有的处理节点
    //然后为每个处理节点创建输入和输出通道，并将它们添加到当前正在构建的路由器中
    for (int y = 0; y < _cY ; y++) {//y = 0、1
      for (int x = 0; x < _cX ; x++) {//x = 0
	//假设(x,y)=(1,1),link=(3*1)*(2*1+0)+(1*1+0)=7
	//_cx=1,_cy=2,k=3 link=(3*1)*(2*1+1)+(1*1+0)=9
	int link = (_k * _cX) * (_cY * y_index + y) + (_cX * x_index + x) ;
	assert( link >= 0 ) ;
	assert( link < _nodes ) ;
	assert( channel_vector[ link ] == false ) ;
	channel_vector[link] = true ;
	// Ingress Ports
	//为当前路由器添加一个输入通道，_inject[link]是输入通道对象
	//_inject_cred[link]是与该输入通道关联的信用信息，用于流量控制
	_routers[node]->AddInputChannel(_inject[link], _inject_cred[link]);
	// Egress Ports
	//添加一个输出通道，_eject[link]是输出通道对象...
	_routers[node]->AddOutputChannel(_eject[link], _eject_cred[link]);
	//injeciton ejection latency is 1
	//设置输入和输出通道的延迟为1，在网路模拟中，延迟表示数据包或信号
	//通过该通道所需要的时间
	_inject[link]->SetLatency( 1 );
	_eject[link]->SetLatency( 1 );
      }
    }

    //
    // router to router channels，路由器之间的通道建立
    //
    //计算当前路由器在mesh中的位置[1,1]
    const int x = node % _k ;//1
    const int y = node / _k ;//1
    const int offset = powi( _k, _n ) ;//9

    //the channel number of the input output channels.通道的标号
    //px:positive x | nx:negtive x
    int px_out = _k * y + x + 0 * offset ;//4
    int nx_out = _k * y + x + 1 * offset ;//13
    int py_out = _k * y + x + 2 * offset ;//22
    int ny_out = _k * y + x + 3 * offset ;//31
    int px_in  = _k * y + ((x+1)) + 1 * offset ;//14，x+1意味着来自东边邻居路由器
    int nx_in  = _k * y + ((x-1)) + 0 * offset ;//3
    int py_in  = _k * ((y+1)) + x + 3 * offset ;//34，y+1意味来自南边的邻居路由器
    int ny_in  = _k * ((y-1)) + x + 2 * offset ;//19

    // Express Channels
    if (x == 0){
      // Router on left edge of mesh. Connect to -x output of
      //  another router on the left edge of the mesh.
      if (y < _k / 2 )
	nx_in = _k * (y + _k/2) + x + offset ;
      else
	nx_in = _k * (y - _k/2) + x + offset ;
    }
    
    if (x == (_k-1)){
      // Router on right edge of mesh. Connect to +x output of 
      //  another router on the right edge of the mesh.
      if (y < _k / 2)
   	px_in = _k * (y + _k/2) + x ;
      else
   	px_in = _k * (y - _k/2) + x ;
    }
    
    if (y == 0) {
      // Router on bottom edge of mesh. Connect to -y output of
      //  another router on the bottom edge of the mesh.
      if (x < _k / 2) 
	ny_in = _k * y + (x + _k/2) + 3 * offset ;
      else
	ny_in = _k * y + (x - _k/2) + 3 * offset ;
    }
    
    if (y == (_k-1)) {
      // Router on top edge of mesh. Connect to +y output of
      //  another router on the top edge of the mesh
      if (x < _k / 2)
	py_in = _k * y + (x + _k/2) + 2 * offset ;
      else
	py_in = _k * y + (x - _k/2) + 2 * offset ;
    }

    /*set latency and add the channels*/
    //设置网络中路由器的输出和输入通道延迟，并添加这些通道到路由器中
    // Port 0: +x channel
    if(use_noc_latency) {//这个if用于检查是否使用NoC的延迟模型，如果是将根据网络拓扑计算通道的延迟
      int const px_latency = (x == _k-1) ? (_cY*_k/2) : _cX;
      _chan[px_out]->SetLatency( px_latency );
      _chan_cred[px_out]->SetLatency( px_latency );
    } else {
      _chan[px_out]->SetLatency( 1 );
      _chan_cred[px_out]->SetLatency( 1 );
    }
    _routers[node]->AddOutputChannel( _chan[px_out], _chan_cred[px_out] );
    _routers[node]->AddInputChannel( _chan[px_in], _chan_cred[px_in] );
    
    if(gTrace) {
      cout<<"Link "<<" "<<px_out<<" "<<px_in<<" "<<node<<" "<<_chan[px_out]->GetLatency()<<endl;
    }

    // Port 1: -x channel
    if(use_noc_latency) {
      int const nx_latency = (x == 0) ? (_cY*_k/2) : _cX;
      _chan[nx_out]->SetLatency( nx_latency );
      _chan_cred[nx_out]->SetLatency( nx_latency );
    } else {
      _chan[nx_out]->SetLatency( 1 );
      _chan_cred[nx_out]->SetLatency( 1 );
    }
    _routers[node]->AddOutputChannel( _chan[nx_out], _chan_cred[nx_out] );
    _routers[node]->AddInputChannel( _chan[nx_in], _chan_cred[nx_in] );

    if(gTrace){
      cout<<"Link "<<" "<<nx_out<<" "<<nx_in<<" "<<node<<" "<<_chan[nx_out]->GetLatency()<<endl;
    }

    // Port 2: +y channel
    if(use_noc_latency) {
      int const py_latency = (y == _k-1) ? (_cX*_k/2) : _cY;
      _chan[py_out]->SetLatency( py_latency );
      _chan_cred[py_out]->SetLatency( py_latency );
    } else {
      _chan[py_out]->SetLatency( 1 );
      _chan_cred[py_out]->SetLatency( 1 );
    }
    _routers[node]->AddOutputChannel( _chan[py_out], _chan_cred[py_out] );
    _routers[node]->AddInputChannel( _chan[py_in], _chan_cred[py_in] );
    
    if(gTrace){
      cout<<"Link "<<" "<<py_out<<" "<<py_in<<" "<<node<<" "<<_chan[py_out]->GetLatency()<<endl;
    }

    // Port 3: -y channel
    if(use_noc_latency){
      int const ny_latency = (y == 0) ? (_cX*_k/2) : _cY;
      _chan[ny_out]->SetLatency( ny_latency );
      _chan_cred[ny_out]->SetLatency( ny_latency );
    } else {
      _chan[ny_out]->SetLatency( 1 );
      _chan_cred[ny_out]->SetLatency( 1 );
    }
    _routers[node]->AddOutputChannel( _chan[ny_out], _chan_cred[ny_out] );
    _routers[node]->AddInputChannel( _chan[ny_in], _chan_cred[ny_in] );    

    if(gTrace){
      cout<<"Link "<<" "<<ny_out<<" "<<ny_in<<" "<<node<<" "<<_chan[ny_out]->GetLatency()<<endl;
    }
    
  }    

  // Check that all processors were connected to the network
  for ( int i = 0 ; i < _nodes ; i++ ) 
    assert( channel_vector[i] == true ) ;
  
  if(gTrace){
    cout<<"Setup Finished Link"<<endl;
  }
}


// ----------------------------------------------------------------------
//
//  Routing Helper Functions
//
// ----------------------------------------------------------------------
//根据给定的节点地址计算出该节点所连接的路由器的索引
int CMesh::NodeToRouter( int address ) {

  int y  = (address /  (_cX*gK))/_cY ;//  address/(_c*gk)
  int x  = (address %  (_cX*gK))/_cY ;//  
  int router = y*gK + x ;
  
  return router ;
}

int CMesh::NodeToPort( int address ) {
  
  const int maskX  = _cX - 1 ;
  const int maskY  = _cY - 1 ;

  int x = address & maskX ;
  int y = (int)(address/(2*gK)) & maskY ;

  return (gC / 2) * y + x;
}

// ----------------------------------------------------------------------
//
//  Routing Functions//路由函数
//
// ----------------------------------------------------------------------

// Concentrated Mesh: X-Y
int cmesh_xy( int cur, int dest ) {

  const int POSITIVE_X = 0 ;
  const int NEGATIVE_X = 1 ;
  const int POSITIVE_Y = 2 ;
  const int NEGATIVE_Y = 3 ;

  int cur_y  = cur / gK;
  int cur_x  = cur % gK;
  int dest_y = dest / gK;
  int dest_x = dest % gK;

  // Dimension-order Routing: x , y
  if (cur_x < dest_x) {
    // Express?
    if ((dest_x - cur_x) > 1){
      if (cur_y == 0)
    	return gC + NEGATIVE_Y ;
      if (cur_y == (gK-1))
    	return gC + POSITIVE_Y ;
    }
    return gC + POSITIVE_X ;
  }
  if (cur_x > dest_x) {
    // Express ? 
    if ((cur_x - dest_x) > 1){
      if (cur_y == 0)
    	return gC + NEGATIVE_Y ;
      if (cur_y == (gK-1))
    	return gC + POSITIVE_Y ;
    }
    return gC + NEGATIVE_X ;
  }
  if (cur_y < dest_y) {
    // Express?
    if ((dest_y - cur_y) > 1) {
      if (cur_x == 0)
    	return gC + NEGATIVE_X ;
      if (cur_x == (gK-1))
    	return gC + POSITIVE_X ;
    }
    return gC + POSITIVE_Y ;
  }
  if (cur_y > dest_y) {
    // Express ?
    if ((cur_y - dest_y) > 1 ){
      if (cur_x == 0)
    	return gC + NEGATIVE_X ;
      if (cur_x == (gK-1))
    	return gC + POSITIVE_X ;
    }
    return gC + NEGATIVE_Y ;
  }
  return 0;
}

// Concentrated Mesh: Y-X
int cmesh_yx( int cur, int dest ) {
  const int POSITIVE_X = 0 ;
  const int NEGATIVE_X = 1 ;
  const int POSITIVE_Y = 2 ;
  const int NEGATIVE_Y = 3 ;

  int cur_y  = cur / gK ;
  int cur_x  = cur % gK ;
  int dest_y = dest / gK ;
  int dest_x = dest % gK ;

  // Dimension-order Routing: y, x
  if (cur_y < dest_y) {
    // Express?
    if ((dest_y - cur_y) > 1) {
      if (cur_x == 0)
    	return gC + NEGATIVE_X ;
      if (cur_x == (gK-1))
    	return gC + POSITIVE_X ;
    }
    return gC + POSITIVE_Y ;
  }
  if (cur_y > dest_y) {
    // Express ?
    if ((cur_y - dest_y) > 1 ){
      if (cur_x == 0)
    	return gC + NEGATIVE_X ;
      if (cur_x == (gK-1))
    	return gC + POSITIVE_X ;
    }
    return gC + NEGATIVE_Y ;
  }
  if (cur_x < dest_x) {
    // Express?
    if ((dest_x - cur_x) > 1){
      if (cur_y == 0)
    	return gC + NEGATIVE_Y ;
      if (cur_y == (gK-1))
    	return gC + POSITIVE_Y ;
    }
    return gC + POSITIVE_X ;
  }
  if (cur_x > dest_x) {
    // Express ? 
    if ((cur_x - dest_x) > 1){
      if (cur_y == 0)
    	return gC + NEGATIVE_Y ;
      if (cur_y == (gK-1))
    	return gC + POSITIVE_Y ;
    }
    return gC + NEGATIVE_X ;
  }
  return 0;
}

void xy_yx_cmesh( const Router *r, const Flit *f, int in_channel, 
		  OutputSet *outputs, bool inject )
{

  // ( Traffic Class , Routing Order ) -> Virtual Channel Range
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    // Current Router
    int cur_router = r->GetID();

    // Destination Router
    int dest_router = CMesh::NodeToRouter( f->dest ) ;  

    if (dest_router == cur_router) {

      // Forward to processing element
      out_port = CMesh::NodeToPort( f->dest );      

    } else {

      // Forward to neighbouring router

      //each class must have at least 2 vcs assigned or else xy_yx will deadlock
      int const available_vcs = (vcEnd - vcBegin + 1) / 2;
      assert(available_vcs > 0);

      // randomly select dimension order at first hop
      bool x_then_y = ((in_channel < gC) ?
		       (RandomInt(1) > 0) :
		       (f->vc < (vcBegin + available_vcs)));

      if(x_then_y) {
	out_port = cmesh_xy( cur_router, dest_router );
	vcEnd -= available_vcs;
      } else {
	out_port = cmesh_yx( cur_router, dest_router );
	vcBegin += available_vcs;
      }
    }

  }

  outputs->Clear();

  outputs->AddRange( out_port , vcBegin, vcEnd );
}

// ----------------------------------------------------------------------
//
//  Concentrated Mesh: Random XY-YX w/o Express Links
//
//   <int> cur:  current router address 
///  <int> dest: destination router address 
//
// ----------------------------------------------------------------------

int cmesh_xy_no_express( int cur, int dest ) {
  
  const int POSITIVE_X = 0 ;
  const int NEGATIVE_X = 1 ;
  const int POSITIVE_Y = 2 ;
  const int NEGATIVE_Y = 3 ;

  const int cur_y  = cur  / gK ;
  const int cur_x  = cur  % gK ;
  const int dest_y = dest / gK ;
  const int dest_x = dest % gK ;


  //  Note: channel numbers bellow gC (degree of concentration) are
  //        injection and ejection links

  // Dimension-order Routing: X , Y
  if (cur_x < dest_x) {
    return gC + POSITIVE_X ;
  }
  if (cur_x > dest_x) {
    return gC + NEGATIVE_X ;
  }
  if (cur_y < dest_y) {
    return gC + POSITIVE_Y ;
  }
  if (cur_y > dest_y) {
    return gC + NEGATIVE_Y ;
  }
  return 0;
}

int cmesh_yx_no_express( int cur, int dest ) {

  const int POSITIVE_X = 0 ;
  const int NEGATIVE_X = 1 ;
  const int POSITIVE_Y = 2 ;
  const int NEGATIVE_Y = 3 ;
  
  const int cur_y  = cur / gK ;
  const int cur_x  = cur % gK ;
  const int dest_y = dest / gK ;
  const int dest_x = dest % gK ;

  //  Note: channel numbers bellow gC (degree of concentration) are
  //        injection and ejection links

  // Dimension-order Routing: X , Y
  if (cur_y < dest_y) {
    return gC + POSITIVE_Y ;
  }
  if (cur_y > dest_y) {
    return gC + NEGATIVE_Y ;
  }
  if (cur_x < dest_x) {
    return gC + POSITIVE_X ;
  }
  if (cur_x > dest_x) {
    return gC + NEGATIVE_X ;
  }
  return 0;
}

void xy_yx_no_express_cmesh( const Router *r, const Flit *f, int in_channel, 
			     OutputSet *outputs, bool inject )
{
  // ( Traffic Class , Routing Order ) -> Virtual Channel Range
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    // Current Router
    int cur_router = r->GetID();

    // Destination Router
    int dest_router = CMesh::NodeToRouter( f->dest );  

    if (dest_router == cur_router) {

      // Forward to processing element
      out_port = CMesh::NodeToPort( f->dest );

    } else {

      // Forward to neighbouring router
    
      //each class must have at least 2 vcs assigned or else xy_yx will deadlock
      int const available_vcs = (vcEnd - vcBegin + 1) / 2;
      assert(available_vcs > 0);

      // randomly select dimension order at first hop
      bool x_then_y = ((in_channel < gC) ?
		       (RandomInt(1) > 0) :
		       (f->vc < (vcBegin + available_vcs)));

      if(x_then_y) {
	out_port = cmesh_xy_no_express( cur_router, dest_router );
	vcEnd -= available_vcs;
      } else {
	out_port = cmesh_yx_no_express( cur_router, dest_router );
	vcBegin += available_vcs;
      }
    }
  }

  outputs->Clear();

  outputs->AddRange( out_port , vcBegin, vcEnd );
}
//============================================================
//
//=====
int cmesh_next( int cur, int dest ) {

  const int POSITIVE_X = 0 ;
  const int NEGATIVE_X = 1 ;
  const int POSITIVE_Y = 2 ;
  const int NEGATIVE_Y = 3 ;
  
  int cur_y  = cur / gK ;
  int cur_x  = cur % gK ;
  int dest_y = dest / gK ;
  int dest_x = dest % gK ;

  // Dimension-order Routing: x , y
  if (cur_x < dest_x) {
    // Express?
    if ((dest_x - cur_x) > gK/2-1){
      if (cur_y == 0)
	return gC + NEGATIVE_Y ;
      if (cur_y == (gK-1))
	return gC + POSITIVE_Y ;
    }
    return gC + POSITIVE_X ;
  }
  if (cur_x > dest_x) {
    // Express ? 
    if ((cur_x - dest_x) > gK/2-1){
      if (cur_y == 0)
	return gC + NEGATIVE_Y ;
      if (cur_y == (gK-1)) 
	return gC + POSITIVE_Y ;
    }
    return gC + NEGATIVE_X ;
  }
  if (cur_y < dest_y) {
    // Express?
    if ((dest_y - cur_y) > gK/2-1) {
      if (cur_x == 0)
	return gC + NEGATIVE_X ;
      if (cur_x == (gK-1))
	return gC + POSITIVE_X ;
    }
    return gC + POSITIVE_Y ;
  }
  if (cur_y > dest_y) {
    // Express ?
    if ((cur_y - dest_y) > gK/2-1){
      if (cur_x == 0)
	return gC + NEGATIVE_X ;
      if (cur_x == (gK-1))
	return gC + POSITIVE_X ;
    }
    return gC + NEGATIVE_Y ;
  }

  assert(false);
  return -1;
}

void dor_cmesh( const Router *r, const Flit *f, int in_channel, 
		OutputSet *outputs, bool inject )
{
  // ( Traffic Class , Routing Order ) -> Virtual Channel Range
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    // Current Router
    int cur_router = r->GetID();

    // Destination Router
    int dest_router = CMesh::NodeToRouter( f->dest ) ;  
  
    if (dest_router == cur_router) {

      // Forward to processing element
      out_port = CMesh::NodeToPort( f->dest ) ;

    } else {

      // Forward to neighbouring router
      out_port = cmesh_next( cur_router, dest_router );
    }
  }

  outputs->Clear();

  outputs->AddRange( out_port, vcBegin, vcEnd);
}

//============================================================
//
//=====
int cmesh_next_no_express( int cur, int dest ) {

  const int POSITIVE_X = 0 ;
  const int NEGATIVE_X = 1 ;
  const int POSITIVE_Y = 2 ;
  const int NEGATIVE_Y = 3 ;
  
  //magic constant 2, which is supose to be _cX and _cY
  int cur_y  = cur/gK ;
  int cur_x  = cur%gK ;
  int dest_y = dest/gK;
  int dest_x = dest%gK ;

  // Dimension-order Routing: x , y
  if (cur_x < dest_x) {
    return gC + POSITIVE_X ;
  }
  if (cur_x > dest_x) {
    return gC + NEGATIVE_X ;
  }
  if (cur_y < dest_y) {
    return gC + POSITIVE_Y ;
  }
  if (cur_y > dest_y) {
    return gC + NEGATIVE_Y ;
  }
  assert(false);
  return -1;
}

void dor_no_express_cmesh( const Router *r, const Flit *f, int in_channel, 
			   OutputSet *outputs, bool inject )
{
  // ( Traffic Class , Routing Order ) -> Virtual Channel Range
  int vcBegin = 0, vcEnd = gNumVCs-1;
  if ( f->type == Flit::READ_REQUEST ) {
    vcBegin = gReadReqBeginVC;
    vcEnd = gReadReqEndVC;
  } else if ( f->type == Flit::WRITE_REQUEST ) {
    vcBegin = gWriteReqBeginVC;
    vcEnd = gWriteReqEndVC;
  } else if ( f->type ==  Flit::READ_REPLY ) {
    vcBegin = gReadReplyBeginVC;
    vcEnd = gReadReplyEndVC;
  } else if ( f->type ==  Flit::WRITE_REPLY ) {
    vcBegin = gWriteReplyBeginVC;
    vcEnd = gWriteReplyEndVC;
  }
  assert(((f->vc >= vcBegin) && (f->vc <= vcEnd)) || (inject && (f->vc < 0)));

  int out_port;

  if(inject) {

    out_port = -1;

  } else {

    // Current Router
    int cur_router = r->GetID();

    // Destination Router
    int dest_router = CMesh::NodeToRouter( f->dest ) ;  
  
    if (dest_router == cur_router) {

      // Forward to processing element
      out_port = CMesh::NodeToPort( f->dest );

    } else {

      // Forward to neighbouring router
      out_port = cmesh_next_no_express( cur_router, dest_router );
    }
  }

  outputs->Clear();

  outputs->AddRange( out_port, vcBegin, vcEnd );
}
