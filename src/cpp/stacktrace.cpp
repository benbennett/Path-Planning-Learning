namespace planning
{

#include <execinfo.h> 
#include<stdlib.h>
#include<stdio.h>
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
}
