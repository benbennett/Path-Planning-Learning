/*
Copyright (c) 2010, Benjamin Bennett
All rights reserved.
(BSD License)
Redistribution and use in source and binary forms, with or without modification,
permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.
* The names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
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
using namespace std;
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

inline planning::tuple<int> createState(int x, int y)
{

    int point[] = { x,y};
    planning::tuple<int> st1(point,point+2);
    return st1;

}

inline planning::tuple<long> createStateL(long x, long y)
{

    long point[] = { x,y};
    planning::tuple<long> st1(point,point+2);
    return st1;

}
BOOST_AUTO_TEST_CASE(StateTestCase)
{

    cout<<"StateTestCase"<<endl;
    using namespace planning;
    boost::shared_ptr < int> null_test;
    BOOST_CHECK(null_test==NULL);
    int pt[] = {10,10};
    planning::tuple<int> goal_pt(pt,pt+2);
    boost::shared_ptr<State <int,double> > goal_ptr;
    goal_ptr.reset(new planning::State<int,double>(goal_pt));
    goal_ptr->setGoal(goal_ptr);
    int s_pt[] = {0,0};
    planning::tuple<int> start_pt(s_pt,s_pt+2);
    boost::shared_ptr<	State<int,double> > start_ptr;
    start_ptr.reset( new State<int,double> (start_pt));

    BOOST_CHECK(goal_ptr->isGoal()==true);
    BOOST_CHECK(start_ptr->g()==planning::INF);
    BOOST_CHECK(start_ptr->rhs()==planning::INF);


}

BOOST_AUTO_TEST_CASE(Aadasimpletest)
{

    cout<<"adasimpletest"<<endl;
    using namespace planning;
    using namespace std;
    AnytimeDstar<int,double> adstar;

    boost::shared_ptr< State<int, double> > start = adstar.createState(1,1);
    boost::shared_ptr< State<int, double> > goal = adstar.createState(100,100);


    adstar.init(start,goal);
    BOOST_CHECK(adstar.ComputeorImprovePath()==100);
    cout<<"Planning successful, running new test"<<endl;
    std::list< boost::shared_ptr< State<int, double> > > final_path = adstar.getPath();
    std::list< boost::shared_ptr< State<int, double> > >::iterator iter_path;
    iter_path= final_path.end();
    iter_path--;
    planning::tuple<int> hold= (*iter_path)->getPoint();
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

BOOST_AUTO_TEST_CASE(DeadEndTest)
{

    cout<<"*********************Dead End Test *****************************"<<endl;
    cout<<"*********************Dead End Test *****************************"<<endl;
    cout<<"*********************Dead End Test *****************************"<<endl;
    using namespace planning;
    using namespace std;

    AnytimeDstar<int,double> adstar;

    shared_ptr< State<int, double> > start = adstar.createState(1,1);
    shared_ptr< State<int, double> > goal = adstar.createState(10,10);

    adstar.init(start,goal);
    int mr =  adstar.ComputeorImprovePath();

    std::cout<<mr<<std::endl;
    //not on path to test outside of path

    adstar.addForbidden(createState(4,5));
    mr = adstar.ComputeorImprovePath();
    adstar.addForbidden(createState(4,6));
    mr = adstar.ComputeorImprovePath();

    adstar.addForbidden(createState(5,6));
    mr = adstar.ComputeorImprovePath();
    adstar.addForbidden(createState(6,5));
    mr = adstar.ComputeorImprovePath();
    adstar.addForbidden(createState(6,4));
    mr = adstar.ComputeorImprovePath();

    adstar.addForbidden(createState(5,4));
    mr = adstar.ComputeorImprovePath();

    adstar.addForbidden(createState(6,6));
    mr = adstar.ComputeorImprovePath();
    BOOST_CHECK(mr>=0);
    cout<<"*********************Dead End Test *****************************"<<endl;
    cout<<"*********************Dead End Test *****************************"<<endl;
    cout<<"*********************Dead End Test *****************************"<<endl;
}

BOOST_AUTO_TEST_CASE(cycletest1)
{
    cout<<"*********************Cycle Test 1 Test *****************************"<<endl;
    cout<<"*********************Cycle Test 1 Test *****************************"<<endl;
    cout<<"*********************Cycle Test 1 Test *****************************"<<endl;
    cout<<"*********************Cycle Test 1 Test *****************************"<<endl;
    using namespace planning;
    using namespace std;

    AnytimeDstar<long,double> adstar;

    boost::shared_ptr< State<long, double> > start = adstar.createState(28606,28605);
    boost::shared_ptr< State<long, double> > goal = adstar.createState(28626,28626);

    adstar.init(start,goal);
    boost::unordered_map< planning::tuple<long>,long> forbids;

    forbids[createStateL(28615,28600)]=1;
    forbids[createStateL(28614,28599)]=1;
    forbids[createStateL(28616,28607)]=1;
    forbids[createStateL(28618,28615)]=1;
    forbids[createStateL(28614,28600)]=1;
    forbids[createStateL(28613,28599)]=1;
    forbids[createStateL(28615,28607)]=1;
    forbids[createStateL(28617,28615)]=1;
    forbids[createStateL(28613,28600)]=1;
    forbids[createStateL(28614,28607)]=1;
    forbids[createStateL(28616,28615)]=1;
    forbids[createStateL(28612,28600)]=1;
    forbids[createStateL(28613,28607)]=1;
    forbids[createStateL(28611,28600)]=1;
    forbids[createStateL(28624,28599)]=1;
    forbids[createStateL(28613,28608)]=1;
    forbids[createStateL(28624,28600)]=1;
    forbids[createStateL(28613,28609)]=1;
    forbids[createStateL(28624,28601)]=1;
    forbids[createStateL(28625,28605)]=1;
    forbids[createStateL(28613,28610)]=1;
    forbids[createStateL(28610,28599)]=1;
    forbids[createStateL(28624,28602)]=1;
    forbids[createStateL(28610,28600)]=1;
    forbids[createStateL(28623,28599)]=1;
    forbids[createStateL(28624,28603)]=1;
    forbids[createStateL(28624,28604)]=1;
    forbids[createStateL(28624,28605)]=1;
    forbids[createStateL(28609,28599)]=1;
    forbids[createStateL(28609,28600)]=1;
    forbids[createStateL(28622,28599)]=1;
    forbids[createStateL(28621,28599)]=1;
    forbids[createStateL(28620,28599)]=1;
    forbids[createStateL(28619,28599)]=1;
    forbids[createStateL(28618,28599)]=1;
    forbids[createStateL(28617,28599)]=1;
    forbids[createStateL(28619,28607)]=1;
    forbids[createStateL(28631,28605)]=1;
    forbids[createStateL(28616,28599)]=1;
    forbids[createStateL(28631,28606)]=1;
    forbids[createStateL(28618,28607)]=1;
    forbids[createStateL(28631,28607)]=1;
    forbids[createStateL(28631,28608)]=1;
    forbids[createStateL(28631,28609)]=1;
    forbids[createStateL(28615,28599)]=1;
    forbids[createStateL(28619,28615)]=1;
    adstar.setForbidden(forbids);
    int mr =  adstar.ComputeorImprovePath();

    cout<<"*********************Cycle Test 1 Test *****************************"<<endl;
    cout<<"*********************Cycle Test 1 Test *****************************"<<endl;
    cout<<"*********************Cycle Test 1 Test *****************************"<<endl;
    cout<<"*********************Cycle Test 1 Test *****************************"<<endl;
    std::cout<<mr<<std::endl;
    BOOST_CHECK(mr>=0);
}
BOOST_AUTO_TEST_CASE(getForbiddens)
{
    cout<<"check get forbiddens"<<endl;
    using namespace planning;
    using namespace std;

    AnytimeDstar<int,double> adstar;

    boost::shared_ptr< State<int, double> > start = adstar.createState(1,1);
    boost::shared_ptr< State<int, double> > goal = adstar.createState(100,100);

    adstar.init(start,goal);
    int mr =  adstar.ComputeorImprovePath();

    std::cout<<mr<<std::endl;
    BOOST_CHECK(mr==100);
    //not on path to test outside of path

    for(int j=2; j<50; j++)
    {
        //add a bunch
        adstar.addForbidden(createState(j,j));
        mr = adstar.ComputeorImprovePath();
        std::cout<<mr<<std::endl;
    }
    boost::unordered_map< planning::tuple<int>,int> forbids = adstar.getForbidden();
    boost::unordered_map< planning::tuple<int>,int>::iterator my_iter =forbids.begin();
    while(my_iter!=forbids.end())
    {
        cout<<my_iter->first<<endl;
        my_iter++;
    }
}

BOOST_AUTO_TEST_CASE(adaAddForbiddenComplex)
{
    cout<<"adaAddForbiddenComplex"<<endl;
    using namespace planning;
    using namespace std;

    AnytimeDstar<int,double> adstar;

    boost::shared_ptr< State<int, double> > start = adstar.createState(1,1);
    boost::shared_ptr< State<int, double> > goal = adstar.createState(100,100);

    adstar.init(start,goal);
    int mr =  adstar.ComputeorImprovePath();

    std::cout<<mr<<std::endl;
    BOOST_CHECK(mr==100);
    //not on path to test outside of path

    for(int j=2; j<50; j++)
    {
        //add a bunch
        adstar.addForbidden(createState(j,j));
        mr = adstar.ComputeorImprovePath();
        std::cout<<mr<<std::endl;
    }



    for(int j=50; j<100; j++)
    {
        //add spot and call planner again
        adstar.addForbidden(createState(j,j));
        cout<<"adaAddForbiddenComplex call "<<j <<","<<j<<endl;
        mr = adstar.ComputeorImprovePath();

        std::cout<<mr<<std::endl;
    }

    cout<<"adaAddForbiddenComplex done getting through points 50 50 99,99 line"<<endl;
    std::list< boost::shared_ptr< State<int, double> > > final_path = adstar.getPath();

    cout<<"got path "<<endl;
    std::list< boost::shared_ptr< State<int, double> > >::iterator iter_path;
    iter_path= final_path.end();
    iter_path--;
    planning::tuple<int> hold= (*iter_path)->getPoint();
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
    boost::unordered_map< planning::tuple<int> , int > forbiddens = adstar.getForbidden();

    iter_path = final_path.begin();
    //should be a diagonal so we will check it.
    while(iter_path!=final_path.end())
    {
        hold= (*iter_path)->getPoint();
        BOOST_CHECK(forbiddens.find(hold) == forbiddens.end());
        iter_path++;
    }
}

BOOST_AUTO_TEST_CASE(adaMoveStart)
{

    using namespace planning;
    using namespace std;

    AnytimeDstar<int,double> adstar;

    boost::shared_ptr< State<int, double> > start = adstar.createState(1,1);
    boost::shared_ptr< State<int, double> > goal = adstar.createState(100,100);

    cout<<"adaMoveStart"<<endl;
    adstar.init(start,goal);
    int mr =  adstar.ComputeorImprovePath();
    std::cout<<mr<<std::endl;
    BOOST_CHECK(mr==100);
    //not on path to test outside of path

    for(int j=20; j<50; j++)
    {
        //add a bunch
        adstar.addForbidden(createState(j,j));

    }

    adstar.ComputeorImprovePath();
    adstar.getPath();
    for(int j=2; j<15; j++)
    {
        //add spot and call planner again
        cout<<"adaMoveStart"<<j<<endl;
        adstar.moveStart(j,j);
        adstar.ComputeorImprovePath();
    }

    std::list< boost::shared_ptr< State<int, double> > > final_path = adstar.getPath();

    cout<<"got path "<<endl;
    std::list< boost::shared_ptr< State<int, double> > >::iterator iter_path;
    iter_path= final_path.end();
    iter_path--;
    planning::tuple<int> hold= (*iter_path)->getPoint();
    std::cout<<hold[0]<<","<<hold[1]<<std::endl;

    BOOST_CHECK(hold[0]==100);
    BOOST_CHECK(hold[1]==100);

}
BOOST_AUTO_TEST_CASE(adaAddForbidden)
{

    cout<<"adaAddForbidden"<<endl;
    using namespace planning;
    using namespace std;
    AnytimeDstar<int,double> adstar;

    boost::shared_ptr< State<int, double> > start = adstar.createState(1,1);
    boost::shared_ptr< State<int, double> > goal = adstar.createState(100,100);

    adstar.init(start,goal);
    BOOST_CHECK(adstar.ComputeorImprovePath()==100);
    //not on path to test outside of path

    adstar.addForbidden(createState(4,5));
    adstar.ComputeorImprovePath();
    std::list< boost::shared_ptr< State<int, double> > > final_path = adstar.getPath();
    std::list< boost::shared_ptr< State<int, double> > >::iterator iter_path;
    iter_path= final_path.end();
    iter_path--;
    planning::tuple<int> hold= (*iter_path)->getPoint();
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

