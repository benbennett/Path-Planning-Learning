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
#include <vector> 
#include <iterator>
#include <list>
#include <cmath>
#include <queue>
#include <functional>
#include <boost/unordered_set.hpp>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/functional/hash.hpp>
#include <iostream>
#include "stacktrace.hpp"
#include "state.hpp"
#include "key.hpp"
namespace  planning 
{

	using namespace std;
	template<typename Z, typename R> 
		class AnytimeDstar
		{
			public:
				boost::shared_ptr < State < Z, R> > start_; 
				boost::shared_ptr < State < Z, R> > goal_; 


				map< vector<Z>, shared_ptr< State<Z,R> > > path_map_;
				map< vector<Z> , Z > forbidden_;
				map< vector<Z> , shared_ptr< State<Z, R > > > states_;
				priority_queue< Key<Z,R>, vector< Key<Z,R>  >,greater< Key<Z,R> >  > open_;
				map< vector<Z> , shared_ptr< State<Z, R > > > incons_;
				map< vector<Z> , shared_ptr< State<Z, R > > > closed_ ;
				R eps_;
		public:
				list< shared_ptr < State < Z, R> > > path_;
				bool never_built_;
		public:
				AnytimeDstar()
				{
					eps_=3.0;
					never_built_=true;
				}

				void init(shared_ptr< State<Z, R > > start,shared_ptr< State<Z, R > > goal )
				{
					eps_=3.0;
					start_  = start;
					start_->setGoal(goal);
					goal_= goal;	
					goal_->setStart(start);
					goal_->setRhs(0);
					goal_->in_queue_=true;	
					Key<Z,R> goal_key(goal_,eps_);
					open_.push(goal_key);
					never_built_=true;
				}
				void setForbidden(map< vector<Z> , Z > forbidden)
				{
					forbidden_ = forbidden;

				}
				void setEps(R eps)
				{
					eps_ = eps;
				}   
				map< vector<Z> , Z >getForbidden()
				{
					return forbidden_;
				}   
				void setStart( const shared_ptr< State<Z, R > >  start)
				{
					start_ = start;
				}
				void moveStart(Z first, Z second)
				{
				   shared_ptr< State<Z, R > >  hold_state = this->createState(first,second);
				   this->buildState(hold_state);
				   hold_state->setStart(hold_state);
					 eps_=3.0;
					 start_  = hold_state;
					 start_->setGoal(goal_);
				}
				shared_ptr< State<Z, R > > getStart()
				{
					return start_;
				}
				bool hasKey(vector<Z> point)
				{
					return  states_.find(point)!= states_.end(); 
				}
				shared_ptr< State<Z, R > > getState( vector<Z> point)
				{
					return  states_.at(point);
				}
				void addState( vector<Z> point, shared_ptr< State<Z, R > > state)
				{
					states_[point] = state;
				}       
				shared_ptr< State<Z, R > > createState( Z first, Z second)
				{
					Z  pt[] = {first,second};
					vector<Z> point(pt,pt+2);
					if ( hasKey(point))
					{
						return states_[point];
					}
					shared_ptr< State<Z, R > >  temp_ptr;
					temp_ptr.reset(new State<Z,R> (point));
					if (goal_==NULL)
						temp_ptr->setGoal(temp_ptr);
					else
					{
						temp_ptr->setGoal(goal_);
					}
					if(start_==NULL)
						temp_ptr->setStart(temp_ptr);
					else
					{
						temp_ptr->setStart(start_);
					}
					states_[point] = temp_ptr;
					return temp_ptr;
				}
				/*  Add forbidden.
				 *  Check if it is has been visited at all, take note , return 
				 *  Else  go through and remove all states that connect to the new forbidden
				 *  state, remove the conntected point and updated the connected states in 
				 *  the priority queue. 
				 *
				 */
				void addForbidden( vector<Z> point)
				{

					if (forbidden_.find(point) != forbidden_.end())
						return;
					forbidden_[point] = 0;
					forbidden_[point] = 0; 
					cout<<"fb:( "<<point[0]<<","<<point[1]<<")"<<endl;
					if (!hasKey(point))
					{
						//do not have to do anything if point has not be visited. 
						return; 
					}
					//have to go through and disconnect the succ going to the 
					//current point that is forbidden.
					shared_ptr< State<Z, R > > state = getState(point);
					typename map< vector<Z> , shared_ptr< State<Z,R> > > ::iterator  succ_iter;				
					map< vector<Z> , shared_ptr< State<Z,R> > >  hold_map;

					shared_ptr< State<Z, R > > hold_update_state;

					hold_map =  state->getSuccessors();
					succ_iter = hold_map.begin();   
					vector< shared_ptr< State<Z, R > > > toUpdate;   
					typename vector<  shared_ptr< State<Z, R > >  >::iterator toUpdate_iter; 
					while (succ_iter!= hold_map.end())
					{
						hold_update_state = succ_iter->second;

						if (closed_.find(succ_iter->first)!= closed_.end())
						{
							closed_.erase(succ_iter->first);
						}
						hold_update_state = succ_iter->second;
						if(hold_update_state->getSuccessors().size()>0)
						{
							hold_update_state->removeSuccessor(state);
							toUpdate.push_back(hold_update_state);
						}	
						succ_iter++;
					}

					toUpdate_iter = toUpdate.begin();
					while (toUpdate_iter != toUpdate.end())
					{
						UpdateState(*toUpdate_iter);
						(*toUpdate_iter)->getMinSuccessor();   
						toUpdate_iter++;
					}

				}
				/* Build up state for the first time , if it has not been explored before. 
				 * Find out if 
				 *
				 *
				 *
				 */
				void  buildState( shared_ptr< State<Z, R > > & in)
				{
					if(in->getSuccessors().size()==0)
					{
						for( Z x = -1;x <2;x++)
						{
							for( Z y = -1; y < 2; y++)
							{
								if(y==0 && x==0)
								{
									continue;
								}
								Z first = in->getPoint()[0]+ x;
								Z second = in->getPoint()[1]+y;

								Z  pt[] = {first,second};
								vector<Z> point(pt,pt+2);
								//only add succ iff not in forbidden		
								if (forbidden_.find( point) == forbidden_.end())
								{
									shared_ptr< State<Z, R > > hold =createState(first,second);
									in->addSuccessor(hold);	
								}
							}
						}
						assert(in->getSuccessors().size()>0);
					}
				}
				/* Changes the priorities of the keys in the queue . 
				 * Should be called after graph change or when the inflation factor changed. 
				 *
				 *
				 *
				 *
				 */
		private:
				void UpdateAllPriorities()
				{

					priority_queue< Key<Z,R>, vector< Key<Z,R>  >,less< Key<Z,R> > > newqueue;
					while(!open_.empty())
					{
						Key<Z,R> hold = open_.top();
						if ( 
							  forbidden_.find(hold.getState()->getPoint()) == forbidden_.end()
								&& 
								hold.getState()->getSuccessors().size()>0
								)
						{
								newqueue.push( Key<Z,R>(hold.getState(),eps_));
						}
						open_.pop();	
					}

					while(!newqueue.empty())
					{
						Key<Z,R> hold = newqueue.top();
						open_.push( Key<Z,R>(hold.getState(),eps_));
						newqueue.pop();
					}
				}
				void MoveAllFromIncsToOpen()
				{


					typename map< vector<Z> , shared_ptr< State<Z,R> > > ::iterator  iter;	
					iter =incons_.begin();
					while(iter!=incons_.end())
					{
						open_.push(Key<Z,R>(iter->second,eps_));
						iter++;	
					}
					incons_.clear();
					
				}
				void ClearClosed()
				{

					this->closed_.clear();
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
				void UpdateState( shared_ptr< State<Z, R > >  s)
				{

					if( !(s->isGoal()))
						s->setRhs(s->getMinSuccessorValue());
					//work around , need a priority map
					if( s->in_queue_)
						s->in_queue_=false;
					if(s->g() != s->rhs())
					{
						if(closed_.find(s->getPoint())== closed_.end())
						{
						  
							s->in_queue_ = true;
							assert(s->getSuccessors().size()>0);
							Key<Z,R> hold(s,eps_);
							assert(hold.getState()->getSuccessors().size()>0);
							open_.push(hold);

						}
						else
							incons_[s->getPoint()] = s;
					}			
				}
				/* Clears out all points that should be removed from the queue.
				 * The points are removed because they are the successor of a point that
				 * was just anazalzed and its priority has already been checked.
				 * TODO priority map is what needs to be done to replace this   
				 * functionality. 
				 */
				bool filterQueue()
				{

					shared_ptr< State<Z, R > > hold_state;
					if(open_.empty())
						return false;
					while(!open_.empty()) 
					{
						Key<Z,R> hold(open_.top());
						if(hold.getState()->in_queue_==false)
							open_.pop();
						else
							break;	
					}
					if(open_.empty())
						return false;
					return true;	
				}
		public:
				int ComputeorImprovePath()
				{
						int mr;
						if(never_built_)
						{
								cout<<"Start:"<<start_;
								mr= ComputePath(start_);
								never_built_=false;
								int bp;
								if(mr>=0)
								 bp= buildPath();
								if(bp>=0)
								{
										cout<<"First time planning successful."<<endl;
										return mr;
								}

						}
						for(int i=0;i<5;i++)
						{
								MoveAllFromIncsToOpen();
								UpdateAllPriorities();
								ClearClosed();
								mr= ComputePath(start_);
								int bp;
								if(mr>=0)
								 bp= buildPath();
								if(bp>=0)
								{
										cout<<"First time planning successful."<<endl;
										return mr;
								}
								else
								{
										cout<<"Planning failed, trying to replan."<<endl;
										
								}

						}
						cout<<"Need to replan from scratch."<<endl;
						return -1;
				}
		private:
				int ComputePath(shared_ptr < State < Z, R> >   start)
				{
					Key<Z,R> hold_key;
					shared_ptr< State<Z, R > > hold_state;
					shared_ptr< State<Z, R > > hold_update_state;
					//g++ wants the typename in front or it gets confused.	
					typename map< vector<Z> , shared_ptr< State<Z,R> > > ::iterator  succ_iter;				
					map< vector<Z> , shared_ptr< State<Z,R> > >  hold_map;
					int states = 0;
					buildState(start);
					while( filterQueue() && 
							( Key<Z,R>(open_.top(),eps_)
							  < Key<Z,R>(start,eps_)
							  || start->g() != start->rhs()) )
					{
						hold_key =  open_.top();
						hold_state = hold_key.getState();	
						hold_state->in_queue_=false;
						hold_state->setStart(start);
						hold_state->visited_=true;
						open_.pop();
						states++;
						if( hold_state->g() > hold_state->rhs())
						{
							hold_state->setG(hold_state->rhs());
							closed_[hold_state->getPoint()]= hold_state;
							buildState(hold_state);
							assert(hold_state->getSuccessors().size()>0);
							UpdateState(hold_state);
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
				inline std::vector<Z> get_key(Z x, Z y)
				{
					Z point[] = {x,y};
					std::vector<Z> key(point,point+2);
					return key;
				}
				/* Utility method to get a list of states from start to goal.
				 * Not really need , but just a ease of use. 
				 */	
		public:
				list< shared_ptr <  State < Z, R>  >  > getPath()
				{
						return path_;
				}
		private:
				int buildPath()
				{
					path_.clear();
					//typename State<Z,R>  hold; 
					shared_ptr < State < Z, R> > hold;			
					shared_ptr < State < Z, R> > prev;			
					hold = prev= start_;
					path_map_.clear();

					do
					{	
						//you shouldn't need the beginning
						//because that is where you are starting.	
						hold = hold->getMinSuccessor();
						if(path_map_.find(hold->getPoint())!=path_map_.end())
						{
								typename list< boost::shared_ptr < State < Z, R> > >::iterator  path_iter;
								path_iter=path_.begin();
								cout<<"Cycle detected, will attempt to replan"<<endl;
								while(path_iter!=path_.end())
								{
										cout<<*(*path_iter)<<":"<<(*path_iter)->g()<<",";
										path_iter++;
								}
								cout<<*hold<<endl;
								throw 1;
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
