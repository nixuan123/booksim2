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

////////////////////////////////////////////////////////////////////////
//
// CMesh: Mesh topology with concentration and express links along the
//         edge of the network
//
////////////////////////////////////////////////////////////////////////
//
// RCS Information:
//  $Author: jbalfour $
//  $Date: 2007/06/26 22:49:23 $
//  $Id$
// 
////////////////////////////////////////////////////////////////////////
#ifndef _HammingMESH_HPP_
#define _HammingMESH_HPP_
//包含了网络和函数的声明
#include "network.hpp"
#include "routefunc.hpp"
//定义了一个CMesh的网络拓扑，它继承自Network类
class HammingMesh : public Network {
public:
  //使用配置信息和网络名称来初始化CMesh实例
  HamingMesh( const Configuration &config, const string & name );
  int GetN() const;//用于返回网络的维度
  int GetK() const;//用于返回每个维度上路由器的数量

  static int NodeToRouter( int address ) ;//将节点地址转换为路由器id
  static int NodeToPort( int address ) ;//将节点地址转换为端口号

  static void RegisterRoutingFunctions() ;//注册路由函数

private:

  static int _cX ;
  static int _cY ;

  static int _memo_NodeShiftX ;
  static int _memo_NodeShiftY ;
  static int _memo_PortShiftY ;

  void _ComputeSize( const Configuration &config );//根据配置信息计算网络的大小
  void _BuildNet( const Configuration& config );//构建网络，包括初始化路由器和通道

  int _k ;
  int _n ;
  int _c ;
  int _xcount;
  int _ycount;
  int _xrouter;
  int _yrouter;
  bool _express_channels;
};

//
// Routing Functions
//
//路由函数
void xy_yx_cmesh( const Router *r, const Flit *f, int in_channel, 
		  OutputSet *outputs, bool inject ) ;

void xy_yx_no_express_cmesh( const Router *r, const Flit *f, int in_channel, 
			     OutputSet *outputs, bool inject ) ;
//实现了基于XY-YX路由策略的路由函数，前者考虑了快速通道，后者没有
void dor_cmesh( const Router *r, const Flit *f, int in_channel, 
		OutputSet *outputs, bool inject ) ;

void dor_no_express_cmesh( const Router *r, const Flit *f, int in_channel, 
			   OutputSet *outputs, bool inject ) ;

#endif
