
#include "ada_star.hpp"
#include "key.hpp"
#include "state.hpp"
#include <vector>
#include <iostream>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <iterator>
#include <algorithm>
#include <queue>
#include <vector>
#include <functional>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE AnyDstar 
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_CASE(testKeyClass)
{
	using namespace planning;	
	planning::Key<int,double>one(0,0);
	planning::Key<int,double>two(0,0);
	planning::Key<int,double>three(1,2);
	planning::Key<int,double>four(1,3);
	BOOST_CHECK(one==two);
	BOOST_CHECK(one>=two);
	BOOST_CHECK(one<=two);

	BOOST_CHECK(one < three);
	BOOST_CHECK(three < four);

	BOOST_CHECK(three  > one);
	BOOST_CHECK(four  > three);

	BOOST_CHECK(one <= three);
	BOOST_CHECK(three <= four);

	BOOST_CHECK(three >= one);
	BOOST_CHECK(four >= three);
	one = four;
	BOOST_CHECK(four==one);
	
	one = two;

	std::priority_queue< Key<int,double>, std::vector< Key<int, double> >,std::greater< Key<int, double > > > open_;

	open_.push(four);
	open_.push(three);
	open_.push(two);
	open_.push(one);
	planning::Key<int,double>hold(-1,-1);
	
	hold  = open_.top();
	open_.pop();		
	BOOST_CHECK(hold==one);

	hold  = open_.top();
	open_.pop();		
	BOOST_CHECK(hold==two);

	hold  = open_.top();
	open_.pop();		
	BOOST_CHECK(hold==three);

	hold  = open_.top();
	open_.pop();		
	BOOST_CHECK(hold==four);

}

inline std::vector<int> createState(int x, int y)
{

	int point[] = { x,y}; 
	std::vector<int> st1(point,point+2);
	return st1;

}
BOOST_AUTO_TEST_CASE(StateTestCase)
{

	using namespace planning;
	shared_ptr < int> null_test;
	BOOST_CHECK(null_test==NULL);
	int pt[] = {10,10};
	std::vector<int> goal_pt(pt,pt+2);	
	boost::shared_ptr<State <int,double> > goal_ptr;
	goal_ptr.reset(new planning::State<int,double>(goal_pt));
	goal_ptr->setGoal(goal_ptr);	
	int s_pt[] = {0,0};
	std::vector<int> start_pt(s_pt,s_pt+2);
	shared_ptr<	State<int,double> > start_ptr;
	start_ptr.reset( new State<int,double> (start_pt));
  	
	BOOST_CHECK(goal_ptr->isGoal()==true);
  	BOOST_CHECK(start_ptr->g()==planning::INF);
  	BOOST_CHECK(start_ptr->rhs()==planning::INF);


}


BOOST_AUTO_TEST_CASE(adaAddForbiddenComplex)
{

	using namespace planning;
	using namespace std;
	typedef  State<int,double> aState;
	typedef  AnytimeDstar<int,double> ADStar_def;
	typedef  shared_ptr< State<int,double> > shared_state_def;

	typedef  shared_ptr< State<int,double> > shared_state_def;
	ADStar_def adstar;
	
	shared_state_def start = adstar.createState(1,1);
	shared_state_def goal = adstar.createState(100,100);

	adstar.init(start,goal);
    int mr =  adstar.ComputeorImprovePath();
	std::cout<<mr<<std::endl;	
	BOOST_CHECK(mr==100);
	//not on path to test outside of path 	

	for(int j=2;j<50;j++)
	{
		//add a bunch
		adstar.addForbidden(createState(j,j));	

	}

	adstar.MoveAllFromIncsToOpen();
	adstar.UpdateAllPriorities();
	adstar.ClearClosed();
	adstar.ComputeorImprovePath();
	adstar.getPath();
	for(int j=50;j<100;j++)
	{
		//add spot and call planner again
		adstar.addForbidden(createState(j,j));	
		adstar.MoveAllFromIncsToOpen();
		adstar.UpdateAllPriorities();
		adstar.ClearClosed();
		cout<<"adaAddForbiddenComplex call "<<j <<","<<j;
		adstar.ComputeorImprovePath();
	}

	cout<<"adaAddForbiddenComplex done getting through points 50 50 99,99 line"<<endl; 
	std::list< shared_state_def > final_path = adstar.getPath();

	cout<<"got path "<<endl; 
	std::list< shared_state_def >::iterator iter_path;
	iter_path= final_path.end();
	iter_path--;
	std::vector<int> hold= (*iter_path)->getPoint();
	std::cout<<hold[0]<<","<<hold[1]<<std::endl;	
	
	BOOST_CHECK(hold[0]==100);
	BOOST_CHECK(hold[1]==100);
    	
	iter_path = final_path.begin();
	//should be a diagonal so we will check it.	
	int i=2;

	while(i<100)
	{

		cout<<" Checking path"<<endl;
		hold= (*iter_path)->getPoint();
		BOOST_CHECK(!(hold[0]==i && hold[1]==i));
		i++;
		iter_path++;
	}
	//final sanity checks
	std::map< std::vector<int> , int > forbiddens = adstar.getForbidden();

	iter_path = final_path.begin();
	//should be a diagonal so we will check it.	
	while(iter_path!=final_path.end())
	{
		hold= (*iter_path)->getPoint();
		BOOST_CHECK(forbiddens.find(hold) == forbiddens.end());	
		iter_path++;
	}
}

BOOST_AUTO_TEST_CASE(adaAddForbidden)
{

	using namespace planning;
	using namespace std;
	typedef  State<int,double> aState;
	typedef  AnytimeDstar<int,double> ADStar_def;
	typedef  shared_ptr< State<int,double> > shared_state_def;

	typedef  shared_ptr< State<int,double> > shared_state_def;
	ADStar_def adstar;
	
	shared_state_def start = adstar.createState(1,1);
	shared_state_def goal = adstar.createState(100,100);

	adstar.init(start,goal);
    BOOST_CHECK(adstar.ComputeorImprovePath()==100);
	//not on path to test outside of path 	
		
	adstar.addForbidden(createState(4,5));	
	adstar.ComputeorImprovePath();
	std::list< shared_state_def > final_path = adstar.getPath();
	std::list< shared_state_def >::iterator iter_path;
	iter_path= final_path.end();
	iter_path--;
	std::vector<int> hold= (*iter_path)->getPoint();
	std::cout<<hold[0]<<","<<hold[1]<<std::endl;	
	BOOST_CHECK(hold[0]==100);
	BOOST_CHECK(hold[1]==100);
	iter_path = final_path.begin();
	//should be a diagonal so we will check it.	
	int i=2;
	while(iter_path!=final_path.end())
	{
		hold= (*iter_path)->getPoint();
		BOOST_CHECK(hold[0]==i);
		BOOST_CHECK(hold[1]==i);
		i++;
		iter_path++;
	}
}

BOOST_AUTO_TEST_CASE(adasimpletest)
{

	using namespace planning;
	using namespace std;
	typedef  State<int,double> aState;
	typedef  AnytimeDstar<int,double> ADStar_def;
	typedef  shared_ptr< State<int,double> > shared_state_def;

	typedef  shared_ptr< State<int,double> > shared_state_def;
	ADStar_def adstar;
	
	shared_state_def start = adstar.createState(1,1);
	shared_state_def goal = adstar.createState(100,100);


	adstar.init(start,goal);
    BOOST_CHECK(adstar.ComputeorImprovePath()==100);
	std::list< shared_state_def > final_path = adstar.getPath();
	std::list< shared_state_def >::iterator iter_path;
	iter_path= final_path.end();
	iter_path--;
	std::vector<int> hold= (*iter_path)->getPoint();
	std::cout<<hold[0]<<","<<hold[1]<<std::endl;	
	BOOST_CHECK(hold[0]==100);
	BOOST_CHECK(hold[1]==100);
	iter_path = final_path.begin();
	//should be a diagonal so we will check it.	
	int i=2;
	while(iter_path!=final_path.end())
	{
		hold= (*iter_path)->getPoint();
		BOOST_CHECK(hold[0]==i);
		BOOST_CHECK(hold[1]==i);
		i++;
		iter_path++;
	}
}
