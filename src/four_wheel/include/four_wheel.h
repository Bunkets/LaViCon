
#ifndef FOUR_WHEEL_H
#define FOUR_WHEEL_H

#define logs_debug(...)     do{if(logs_display){printf("\033[33m[debug] "); printf(__VA_ARGS__); printf("\033[0m\n");}}while(0)
#define logs_info(...)     do{if(logs_display){printf("[four wheel] [info] "); printf(__VA_ARGS__); printf("\n");}}while(0)
#define logs_error(...)     do{if(logs_display){printf("\033[31m[four wheel] [error] "); printf(__VA_ARGS__); printf("\033[0m\n");}}while(0)

#define logs_debug_stream(args)    do{if(logs_display){std::cout << "\033[33m" << "[debug]" << " " << args << "\033[0m" << std::endl;}}while(0)
#define logs_info_stream(args)    do{if(logs_display){std::cout << "[four wheel]" << " " << "[info]" << " " << args << std::endl;}}while(0)
#define logs_error_stream(args)    do{if(logs_display){std::cout << "\033[31m" << "[four wheel]" << " " << "[error]" << " " << args << "\033[0m" << std::endl;}}while(0)

#define try_catch_head    try {
#define try_catch_end(msg)    } catch(const std::exception& e) {std::cerr << e.what() << msg << '\n'; while(1);}

extern bool logs_display;

#endif
