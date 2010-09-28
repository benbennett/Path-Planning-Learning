#include "stacktrace.hpp"
#include <execinfo.h> 
#include<stdlib.h>
#include<stdio.h>
namespace planning
{

	void stacktrace()
	{

	void *addresses[10];
	char **strings;

	int size = backtrace(addresses, 100);
	strings = backtrace_symbols(addresses, size);
	printf("Stack frames: %d\n", size);
	for(int i = 0; i < size; i++)
	{
		printf("%d: %p\n", i, addresses[i]);
		printf("%s\n", strings[i]);
	}
	free(strings);

	}
  Logger::Logger()
  {
    #ifndef DEBUG	
    		//redirect output to not print out  
        using namespace boost;
        namespace io= boost::iostreams;
    		io::stream_buffer<io::null_sink> * hold_buff;
        hold_buff = new io::stream_buffer<io::null_sink> ();
        hold_buff->open(io::null_sink());
        this->stream_buf_ = hold_buff; 
        this->isCout = false;
    #else
    		this->stream_buff = cout.rdbuf(); 
        this->isCout = true;
    #endif
    	
  }
  Logger::~Logger()
  {
    if(!isCout)
    {
        delete stream_buf_; 
    }
  }
  std::streambuf * Logger::get_stream_buf()
  {
    return stream_buf_;
  }
}
