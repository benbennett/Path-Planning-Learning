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

   Simple implementation of
   "Anytime Dynamic A*: An Anytime, Replanning Algorithm" by
   Maxim Likhachev , Dave Ferguson , Geoff Gordon , Anthony Stentz , and Sebastian Thrun
   Warning this may be incorrect, mainly put together to learn from there paper.
   See paper at:
http://www.ri.cmu.edu/pub_files/pub4/likhachev_maxim_2005_1/likhachev_maxim_2005_1.pdf

*/

#ifndef ADA_STAR_HPP_
#define ADA_STAR_HPP_

#include "stacktrace.hpp"
#include "state.hpp"
#include "key.hpp"
#include "planner.hpp"
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
class AnytimeDstar : public Planner<Z,R>
{
protected:
    void UpdateAllPriorities()
    {

        priority_dict< tuple<Z>, Key<Z,R>  > newqueue;
        while(!this->open_.empty())
        {
            Key<Z,R> hold = this->open_.top();
            if (
                this->forbidden_.find(hold.getState()->getPoint()) == this->forbidden_.end()
                &&
                hold.getState()->getSuccessors().size()>0
            )
                newqueue.push( Key<Z,R>(hold.getState(),this->eps_));

            this->open_.pop();
        }

        while(!newqueue.empty())
        {
            Key<Z,R> hold = newqueue.top();
            this->open_.push( Key<Z,R>(hold.getState(),this->eps_));
            newqueue.pop();
        }
    }
    void MoveAllFromIncsToOpen()
    {


        typename boost::unordered_map< tuple<Z> , boost::shared_ptr< planning::State<Z,R> > > ::iterator  iter;
        iter =this->incons_.begin();
        while(iter!=this->incons_.end())
        {
            this->open_.push(Key<Z,R>(iter->second,this->eps_));
            iter++;
        }
        this->incons_.clear();

    }
    /* Update the state
     * if s not goal
     *   rhs(s) = min_succ(s)
     * if rhs(s) != g(s)
     *    if not in closed :
     *		put in open
     *	  else:
     *		 put in incons
     */
    virtual void UpdateState( boost::shared_ptr< planning::State<Z, R > >  s )
    {

        if( !(s->isGoal()))
            s->setRhs(s->getMinSuccessorValue());

        if(this->open_.contains(s->getPoint()))
            this->open_.remove(s->getPoint());

        if(s->g() != s->rhs())
        {
            if(this->closed_.find(s->getPoint())== this->closed_.end())
            {
                assert(s->getSuccessors().size()>0);
                Key<Z,R> hold(s,this->eps_);
                assert(hold.getState()->getSuccessors().size()>0);
                this->open_.push(hold);

            }
            else
                this->incons_[s->getPoint()] = s;
        }
    }
public:
    int ComputeorImprovePath()
    {
        int mr;
        if(this->never_built_)
        {
            this->log<<"Start:"<<this->start_;
            mr= ComputePath(this->start_);
            this->never_built_=false;
            int bp;
            if(mr>=0)
                bp= this->buildPath();

            if(bp>=0)
            {
                this->log<<"First time planning successful."<<endl;
                return mr;
            }

        }
        for(int i=0; i<5; i++)
        {
            MoveAllFromIncsToOpen();
            UpdateAllPriorities();
            this->ClearClosed();
            mr= ComputePath(this->start_);
            int bp;
            if(mr>=0)
                bp= this->buildPath();
            if(bp>=0)
            {
                cout<<"First time planning successful."<<endl;
                return mr;
            }
            else
                cout<<"Planning failed, trying to replan."<<endl;


        }
        cout<<"Need to replan from scratch."<<endl;
        return -1;
    }
protected:
    int ComputePath(boost::shared_ptr < planning::State< Z, R> >   start)
    {
        Key<Z,R> hold_key;
        boost::shared_ptr< planning::State<Z, R > > hold_state;
        boost::shared_ptr< planning::State<Z, R > > hold_update_state;
        //g++ wants the typename in front or it gets confused.
        typename boost::unordered_map< tuple<Z> , boost::shared_ptr< planning::State<Z,R> > > ::iterator  succ_iter;
        boost::unordered_map< tuple<Z> , boost::shared_ptr< planning::State<Z,R> > >  hold_map;
        int states = 0;
        buildState(start);
        while( !this->open_.empty() &&
                ( Key<Z,R>(this->open_.top(),this->eps_)
                  < Key<Z,R>(start,this->eps_)
                  || start->g() != start->rhs()) )
        {
            hold_key =  this->open_.top();
            hold_state = hold_key.getState();
            this->log<<"state:"<<*hold_state<<endl;
            hold_state->in_queue_=false;
            hold_state->setStart(start);
            hold_state->visited_=true;
            this->open_.pop();
            states++;
            if( hold_state->g() > hold_state->rhs())
            {
                hold_state->setG(hold_state->rhs());
                this->closed_[hold_state->getPoint()]= hold_state;
                buildState(hold_state);
                assert(hold_state->getSuccessors().size()>0);
                hold_map = hold_state->getSuccessors();
                succ_iter = hold_map.begin();
                while(succ_iter!= hold_map.end())
                {
                    hold_update_state = succ_iter->second;
                    assert(succ_iter->second!=NULL);
                    assert(succ_iter->second!=NULL);
                    buildState(hold_update_state);
                    hold_update_state->setStart(start);
                    UpdateState(hold_update_state);
                    succ_iter++;
                }
            }
            else
            {
                hold_state->setG(INF);
                assert(hold_state->getSuccessors().size()>0);

                hold_map = hold_state->getSuccessors();
                succ_iter = hold_map.begin();

                while(succ_iter!= hold_map.end())
                {
                    hold_update_state = succ_iter->second;
                    buildState(hold_update_state);
                    hold_update_state->setStart(start);
                    UpdateState(hold_update_state);
                    succ_iter++;
                }
            }

        }
        return states;
    }
};
}
#endif
