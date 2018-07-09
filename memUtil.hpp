//
// Created by 朱熙 on 2018/6/21.
//

#ifndef PARSERSERVER_MEMUTIL_H
#define PARSERSERVER_MEMUTIL_H

#include <iostream>

#define ZX_MEMCPY(des, src, len) memUtil::zxMemcpy(des, src, len);
#define ZX_DELETE(des) memUtil::zxDelete(des);


class memUtil {
public:
    static void zxMemcpy(char* &des, const char* &&src, int len) {
        if(des) {
            delete des;
            des = nullptr;
        }

        if (len > 0) {
            des = new char[len];
            memcpy(des, src, len);
        }

    }

    static void zxDelete(char* &des) {
        if (des) {
            delete[] des;
            des = nullptr;
        }
    }
};

#endif //PARSERSERVER_MEMUTIL_H
