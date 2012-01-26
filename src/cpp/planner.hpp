/*
Copyright (c) 2010, Benjamin Bennett
All rights reserved.

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
/*

   Base class to hold simplistic methods to allow child classes to
   clearly define the alogrithm
   //TODO make more generic
*/

#ifndef PLANNER_HPP_
#define PLANNER_HPP_

#include "stacktrace.hpp"
#include "state.hpp"
#include "key.hpp"
#include "priority_dict.hpp"
#include <vector>
#include <iterator>
#include <list>
#include <functional>
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/functional/hash.hpp>
#include <iostream>

namespace  planning
{

template<typename Z, typename R>
class Planner
{
protected:
    boost::shared_ptr < planning::State< Z, R> > start_;
    boost::shared_ptr < planning::State< Z, R> > goal_;


    boost::unordered_map< tuple<Z>, boost::shared_ptr< planning::State<Z,R> >  > path_map_;
    boost::unordered_map< tuple<Z> , Z > forbidden_;
    boost::unordered_map< tuple<Z> , boost::shared_ptr< planning::State<Z, R > >   > states_;
    priority_dict< tuple<Z> , Key<Z,R>  > open_;
    boost::unordered_map< tuple<Z> , boost::shared_ptr< planning::State<Z, R > >  > incons_;
    boost::unordered_map< tuple<Z> , boost::shared_ptr< planning::State<Z, R > >  > closed_ ;
    R eps_;
    Logger log_stream;
    std::ostream  log;
protected:
    list< boost::shared_ptr < planning::State< Z, R> > > path_;
    bool never_built_;
public:
    Planner():
        log_stream(),
        log(log_stream.get_stream_buf()),
        eps_(3.0),
        never_built_(true)
    {
    }

    void init(boost::shared_ptr< planning::State<Z, R > > start ,boost::shared_ptr< planning::State<Z, R > > goal )
    {

        eps_=3.0;
        start_  = start;
        start_->setGoal(goal);
        goal_= goal;
        goal_->setStart(start);
        goal_->setRhs(0);
        Key<Z,R> goal_key(goal_,eps_);
        open_.push(goal_key);
        never_built_=true;
    }
    void setStart(  boost::shared_ptr< planning::State<Z, R > >  start)
    {

        start_ = start;
    }
    void setForbidden(boost::unordered_map< tuple<Z> , Z > forbidden)
    {
        forbidden_ = forbidden;
    }
    void setEps(R eps)
    {
        eps_ = eps;
    }
    boost::unordered_map< tuple<Z> , Z > getForbidden()
    {
        return forbidden_;
    }
    void moveStart(Z first, Z second)
    {
        boost::shared_ptr< planning::State<Z, R > >  hold_state = this->createState(first,second);
        this->buildState(hold_state);
        hold_state->setStart(hold_state);
        eps_=3.0;
        start_  = hold_state;
        start_->setGoal(goal_);
        open_.push(Key<Z,R>(goal_,eps_));
    }
    boost::shared_ptr< planning::State<Z, R > > getStart()
    {
        return start_;
    }
    bool hasKey(tuple<Z> point) const
    {
        return  states_.find(point)!= states_.end();
    }
    boost::shared_ptr< planning::State<Z, R > > getState( tuple<Z> point)
    {
        return  states_.at(point);
    }
    void addState( tuple<Z> point, boost::shared_ptr< planning::State<Z, R > > state)
    {

        states_[point] = state;
    }
    boost::shared_ptr< planning::State<Z, R > > createState( Z first, Z second)
    {
        Z  pt[] = {first,second};
        tuple<Z> point(pt,pt+2);
        if ( hasKey(point))
            return states_[point];

        boost::shared_ptr< planning::State<Z, R > >  temp_ptr;
        temp_ptr.reset(new planning::State<Z,R> (point));

        if (goal_==NULL)
            temp_ptr->setGoal(temp_ptr);
        else
            temp_ptr->setGoal(goal_);

        if(start_==NULL)
            temp_ptr->setStart(temp_ptr);
        else
            temp_ptr->setStart(start_);
        states_[point] = temp_ptr;
        return temp_ptr;
    }


    virtual void UpdateState( boost::shared_ptr< planning::State<Z, R > >  s ) = 0;
    /*  Add forbidden.
     *  Check if it is has been visited at all, take note , return
     *  Else  go through and remove all states that connect to the new forbidden
     *  state, remove the conntected point and updated the connected states in
     *  the priority queue.
     *
     */
    void addForbidden( tuple<Z> point)
    {

        if (forbidden_.find(point) != forbidden_.end())
            return;
        forbidden_[point] = 0;
        log<<"fb:( "<<point[0]<<","<<point[1]<<")"<<endl;
        //do not have to do anything if point has not be visited.
        if (!hasKey(point))
            return;
        //have to go through and disconnect the succ going to the
        //current point that is forbidden.
        boost::shared_ptr< planning::State<Z, R > > state = getState(point);
        typename boost::unordered_map< tuple<Z> , boost::shared_ptr< planning::State<Z,R> > > ::iterator  succ_iter;
        boost::unordered_map< tuple<Z> , boost::shared_ptr< planning::State<Z,R> > >  hold_map;

        boost::shared_ptr< planning::State<Z, R > > hold_update_state;

        hold_map =  state->getSuccessors();
        succ_iter = hold_map.begin();
        vector< boost::shared_ptr< planning::State<Z, R > > > toUpdate;
        typename vector<  boost::shared_ptr< planning::State<Z, R > >  >::iterator toUpdate_iter;
        while (succ_iter!= hold_map.end())
        {
            hold_update_state = succ_iter->second;

            if (closed_.find(succ_iter->first)!= closed_.end())
                closed_.erase(succ_iter->first);
            hold_update_state = succ_iter->second;
            if(hold_update_state->getSuccessors().size()>0)
            {
                hold_update_state->removeSuccessor(state);
                //it is a dead end
                if(hold_update_state->getSuccessors().size()<2)
                {
                    shared_ptr< State<Z,R> > delSucc = hold_update_state->getMinSuccessor();
                    delSucc->removeSuccessor(hold_update_state);
                    toUpdate.push_back(delSucc);
                    forbidden_[hold_update_state->getPoint()] = 0;

                }
                else
                    toUpdate.push_back(hold_update_state);
            }
            succ_iter++;
        }

        toUpdate_iter = toUpdate.begin();
        while (toUpdate_iter != toUpdate.end())
        {
            log<<"update state:"<<**toUpdate_iter;
            this->UpdateState(*toUpdate_iter);
            toUpdate_iter++;
        }

    }

    /* Build up state for the first time , if it has not been explored before.
     */
    void  buildState( boost::shared_ptr< planning::State<Z, R > > & newstate)
    {

        if(newstate->getSuccessors().size()==0)
        {
            for( Z x = -1; x <2; x++)
            {
                for( Z y = -1; y < 2; y++)
                {
                    if(y==0 && x==0)
                        continue;
                    Z first = newstate->getPoint()[0]+ x;
                    Z second = newstate->getPoint()[1]+y;

                    Z  pt[] = {first,second};
                    tuple<Z> point(pt,pt+2);
                    //only add succ iff not in forbidden
                    if (forbidden_.find( point) == forbidden_.end())
                        newstate->addSuccessor(createState(first,second));
                }
            }
            //means it is a dead end
            if(newstate->getSuccessors().size()<2)
                forbidden_[newstate->getPoint()]=0;
        }
    }
    void ClearClosed()
    {

        this->closed_.clear();
    }
    inline tuple<Z> get_key(Z x, Z y)
    {
        Z point[] = {x,y};
        return tuple<Z>(point,point+2);
    }
    /* Utility method to get a list of states from start to goal.
     * Not really need , but just a ease of use.
     */
public:
    list< boost::shared_ptr <  planning::State< Z, R>  >  > getPath()
    {
        realBuildPath();
        return path_;
    }
protected:
    int buildPath()
    {
        return realBuildPath();
    }
    int realBuildPath()
    {
        path_.clear();
        //typename planning::State<Z,R>  hold;
        boost::shared_ptr < planning::State< Z, R> > hold;
        boost::shared_ptr < planning::State< Z, R> > prev;
        hold = prev= start_;
        path_map_.clear();

        do
        {
            //you shouldn't need the beginning
            //because that is where you are starting.
            hold = hold->getMinSuccessor();
            if(path_map_.find(hold->getPoint())!=path_map_.end())
            {
                typename list< boost::shared_ptr < planning::State< Z, R> > >::iterator  path_iter;
                path_iter=path_.begin();
                cerr<<"Cycle detected, will attempt to replan"<<endl;

                while(path_iter!=path_.end())
                {
                    cerr<<*(*path_iter)<<":"<<(*path_iter)->g()<< "addr: "<<(*path_iter).get()<< ",";

                    path_iter++;
                }
                cerr<<*hold<<endl;
                return -1;
            }
            path_map_[hold->getPoint()] =prev;
            path_.push_back(hold);
            prev = hold;

        }
        while(hold!=goal_);
        return 1;
    }
};
}
#endif
