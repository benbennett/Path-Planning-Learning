
#include "ada_star.cpp"
#include <iostream>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <iterator>
#include <algorithm>
#include <queue>
#include <vector>
#include <functional>
void testStateClass();
void testAnytimeDstar();
void testAnytimeDstarSimple();
void testKeyClass();
using namespace std;
using namespace planning;
int main()
{

//	testStateClass();
//	testAnytimeDstarSimple();
	testAnytimeDstar();
//	testKeyClass();	
	return 0;
}
void testKeyClass()
{

	planning::Key<int,double>one(0,0);
	planning::Key<int,double>two(0,0);
	planning::Key<int,double>three(1,2);
	planning::Key<int,double>four(1,3);
	assert(one==two);
	assert(one>=two);
	assert(one<=two);

	assert(one < three);
	assert(three < four);

	assert(three  > one);
	assert(four  > three);

	assert(one <= three);
	assert(three <= four);

	assert(three >= one);
	assert(four >= three);
	one = four;
	assert(four==one);
	
	one = two;

	std::priority_queue< Key<int,double>, std::vector< Key<int, double> >,std::greater< Key<int, double > > > open_;

	open_.push(four);
	open_.push(three);
	open_.push(two);
	open_.push(one);
	planning::Key<int,double>hold(-1,-1);
	
	hold  = open_.top();
	open_.pop();		
	assert(hold==one);

	hold  = open_.top();
	open_.pop();		
	assert(hold==two);

	hold  = open_.top();
	open_.pop();		
	assert(hold==three);

	hold  = open_.top();
	open_.pop();		
	assert(hold==four);

}
void testStateClass()
{

	using namespace planning;
	shared_ptr < int> null_test;
	assert(null_test==NULL);
	std::cout<<"Testing State Class"<< std::endl;
	int pt[] = {10,10};
	std::vector<int> goal_pt(pt,pt+2);	
	boost::shared_ptr<State <int,double> > goal_ptr;
	goal_ptr.reset(new planning::State<int,double>(goal_pt));
	goal_ptr->setGoal(goal_ptr);	
	int s_pt[] = {0,0};
	std::vector<int> start_pt(s_pt,s_pt+2);
	shared_ptr<	State<int,double> > start_ptr;
	start_ptr.reset( new State<int,double> (start_pt));
  	
	assert(goal_ptr->isGoal()==true);
  	assert(start_ptr->g()==planning::INF);
  	assert(start_ptr->rhs()==planning::INF);

	std::cout<<"Ending testing State class"<< std::endl;

}


void testAnytimeDstarSimple()
{

	using namespace planning;
	using namespace std;
	typedef  State<int,double> aState;
	typedef  AnytimeDstar<int,double> ADStar_def;

	shared_ptr< ADStar_def >  aDstar;
	aDstar.reset( new ADStar_def());
	shared_ptr< aState > hold_ptr;
	shared_ptr< aState > other_ptr;
	hold_ptr = aDstar->createState(10,10);
	cout<< *hold_ptr <<endl; 
	aDstar->addState(hold_ptr->getPoint(),hold_ptr );
	other_ptr = aDstar->createState(10,10);
	assert(other_ptr==hold_ptr);
	other_ptr = aDstar->createState(0,0);
	
	/*unordered_map<vector<int>, shared_ptr < aState> > states= aDstar->buildState(other_ptr);
	//assert( states.size()==8);	
	//unordered_map<vector<int>, shared_ptr < aState> >::iterator pos = states.begin();
	while(pos!=states.end())
	{
		cout<<	(*(pos->second))<<endl;
		pos++;
	}
	*/

}
void testAnytimeDstar()
{

	using namespace planning;
	using namespace std;
	typedef  State<int,double> aState;
	typedef  AnytimeDstar<int,double> ADStar_def;
	typedef  shared_ptr< State<int,double> > shared_state_def;
	//ADStar_def adstar(100,100);
//	shared_state_def start = adstar.createState(1,1);
//	adstar.init(start);
 //   cout<<adstar.ComputeorImprovePath()<<endl;
}
