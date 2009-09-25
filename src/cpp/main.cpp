
#include "ada_star.cpp"
#include <iostream>
#include <boost/unordered_set.hpp>
#include <iterator>
int main()
{
	std::cout<<"HELLO WORLD"<< std::endl;
	int pt[] = {0,1};
	std::vector<int> foo(pt,pt+2);	
	//foo.push_back(1);
	//foo.push_back(2);	
	planning::State<int,double> myState(foo);
	boost::unordered_set<planning::State<int,double> > aSet;
	aSet.insert(myState);
	std::copy(aSet.begin(),aSet.end(),std::ostream_iterator <planning::State<int,double> >(std::cout,"::"));
	return 0;
}
