#ifndef _STACKTRACE_H_
#define _STACKTRACE_H_
#include <boost/iostreams/device/null.hpp>
#include <boost/iostreams/stream_buffer.hpp>
namespace planning
{
void stacktrace();
//cheesy logger class
class Logger
{
public:
    Logger();
    ~Logger();
    std::streambuf * get_stream_buf();
private:
    bool isCout;
    std::streambuf * stream_buf_;
};
}

#endif
