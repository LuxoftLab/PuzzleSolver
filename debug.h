#ifndef DEBUG_H
#define DEBUG_H

#include <cstdio>
#include <iostream>


#define ARGS_NUM(...) ARGS_NUM_IMPL_((0,__VA_ARGS__,5,4,3,2,1))
#define ARGS_NUM_IMPL_(arg) ARGS_NUM_IMPL arg
#define ARGS_NUM_IMPL(_0,_1,_2,_3,_4,_5, N,...) N
#define FUNC_CALL(func, ...) FUNC_CALL_NUM(func, ARGS_NUM(__VA_ARGS__))
#define FUNC_CALL_NUM(func, args) FUNC_CALL_NUM_(func, args)
#define FUNC_CALL_NUM_(func, args) FUNC_CALL_NUM__(func, args)
#define FUNC_CALL_NUM__(func, args) func ## args

#define PRINT(...) (FUNC_CALL(PRINT, __VA_ARGS__)(__VA_ARGS__))
#define PRINT1(a) std::cout << #a << " = " << (a) << "\n"
#define PRINT2(a,b) std::cout << #a << " = " << (a) << "," << #b << " = " << (b) << "\n"
#define PRINT3(a,b,c) std::cout << #a << " = " << (a) << "," << #b << " = " << (b) << "," << #c << " = " << (c) << "\n"
#define PRINT4(a,b,c,d) std::cout << #a << " = " << (a) << "," << #b << " = " << (b) << "," << #c << " = " << (c) << "," << #d << " = " << (d) << "\n"
#define LOG() std::cout


#endif // DEBUG_H
