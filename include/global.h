//
// Created by 朱熙 on 2018/6/1.
//

#ifndef PARSERSERVER_GLOBAL_H
#define PARSERSERVER_GLOBAL_H
#include <iostream>
#include <list>
#include <thread>
#include <boost/log/trivial.hpp>
#include <boost/log/sources/severity_channel_logger.hpp>
#include "endian.h"
//#include "../src/util/IntegerUtil.hpp"
#include "../memUtil.hpp"


using namespace std;

namespace logging = boost::log;
using namespace logging::trivial;
namespace src = boost::log::sources;
namespace keywords = boost::log::keywords;


typedef vector<char> buffer_type;

#define TCP_RECV_BUF_SIZE 62000             //tcp接收缓存大小
#define SHARE_MEMORY_SIZE 8192              //共享内存大小



#define MQ_RECV_PACKAGE_SIZE 1024           //mq单包数据大小
#define MQ_RECV_BUF_SIZE 2048               //mq接收缓存大小

#endif //PARSERSERVER_GLOBAL_H
