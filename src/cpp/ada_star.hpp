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
#include <cmath>
#include <queue>
#include <functional>
#include <boost/unordered_set.hpp>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/functional/hash.hpp>
#include <iostream>
#include "state.hpp"
#include "key.hpp"
namespace  planning 
{


	template<typename Z, typename R> 
		class AnytimeDstar
		{
			public:
			shared_ptr < State < Z, R> > start_; 
			shared_ptr < State < Z, R> > goal_; 
			
			typedef State<Z,R> state_def;	
			typedef shared_ptr < State<Z,R> > shared_state_def;	
			typedef std::vector<Z> vectorZ;
			typedef Key<Z,R> key_def;
			typedef std::map< std::vector<Z>, shared_state_def > state_map_def;			

		std::map< vectorZ , shared_state_def > forbidden_;
		std::map< vectorZ , shared_state_def > states_;
		std::priority_queue< key_def, std::vector< key_def  >,std::greater< key_def >  > open_;
		std::map< vectorZ , shared_state_def > incons_;
		std::map< vectorZ , shared_state_def > closed_ ;
		R eps_;
	public:
		AnytimeDstar():states_(),open_(),closed_(),incons_()
		{
			eps_=3.0;
		}

		void init(shared_state_def start,shared_state_def goal )
		{
				eps_=3.0;
				start_  = start;
				start_->setGoal(goal);
				goal_= goal;	
				goal_->setStart(start);
				goal_->setRhs(0);
				goal_->in_queue_=true;	
				key_def goal_key(goal_,eps_);
				open_.push(goal_key);
		}
		void setForbidden(std::map< vectorZ , shared_state_def > forbidden)
		{
			forbidden_ = forbidden;

		}
		void setEps(R eps)
		{
			eps_ = eps;
		}   
		std::map< vectorZ , shared_state_def > getForbidden()
		{
			return forbidden_;
		}   
		void setStart( const shared_state_def & start)
		{
			start_ = start;
		}
		shared_state_def getStart()
		{
			return start_;
		}
		bool hasKey(std::vector<Z> point)
		{
			return  states_.find(point)!= states_.end(); 
		}
		shared_state_def getState( std::vector<Z> point)
		{
			return  states_.at(point);
		}
		void addState( std::vector<Z> point, shared_state_def state)
		{
			states_[point] = state;
		}       
		shared_state_def createState( Z first, Z second)
		{
			Z  pt[] = {first,second};
			std::vector<Z> point(pt,pt+2);
			if ( hasKey(point))
			{
				return states_[point];
			}
			shared_state_def  temp_ptr;
			temp_ptr.reset(new state_def(point));
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
		
		void addForbidden( std::vector<Z> point)
		{

			if (forbidden_.find(point) != forbidden_.end())
				return;
			forbidden_.insert(point);
			if (!hasKey(point))
			{
				//do not have to do anything if point has not be visited. 
				return; 
			}
			//have to go through and disconnect the succ going to the 
			//current point that is forbidden.
			shared_state_def state = getState(point);
			typename state_map_def::iterator  succ_iter;				
			state_map_def hold_map;

			shared_state_def hold_update_state;

			hold_map =  state->getSuccessors();
			succ_iter = hold_map.begin();   
			std::vector< shared_state_def > toUpdate;   
			typename std::vector<  shared_state_def  >::iterator toUpdate_iter; 
			while (succ_iter!= hold_map.end())
			{
				hold_update_state = succ_iter->second;

				assert(succ_iter->first!=NULL);
				assert(succ_iter->second!=NULL);
				if (closed_.find(succ_iter->first)!= closed_.end())
				{
					closed_.erase(succ_iter->first);
				}
				hold_update_state = succ_iter->second;
				hold_update_state->removeSuccessor(state);
				toUpdate.push_back(hold_update_state);
				succ_iter++;
			}

			toUpdate_iter = toUpdate.begin();
			while (toUpdate_iter != toUpdate.end())
			{
				UpdateState(*toUpdate_iter);
				toUpdate_iter->getMinSuccessor();   
				toUpdate_iter++;
			}

		}
			void  buildState( shared_state_def & in)
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
						std::vector<Z> point(pt,pt+2);
						//only add succ iff not in forbidden		
						if (forbidden_.find( point) == forbidden_.end())
						{
							shared_state_def hold =createState(first,second);
							in->addSuccessor(hold);	
						}
					}
				}
					assert(in->getSuccessors().size()>0);
				}
			}
			void UpdateAllPriorities()
			{

				std::priority_queue< key_def, std::vector< key_def  >,std::less< key_def > > newqueue;
				while(!open_.empty())
				{
					key_def hold = open_.top();
					newqueue.push( key_def(hold->getState(),eps_));
					open_.pop();	
				}
				open_= newqueue;
			}
			void UpdateState( shared_state_def & s)
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
						key_def hold(s,eps_);
						assert(hold.getState()->getSuccessors().size()>0);
						open_.push(hold);
					    	
					}
					else
						incons_[s->getPoint()] = s;
				}			
			}
			bool filterQueue()
			{

				shared_state_def hold_state;
				if(open_.empty())
					return false;
				while(!open_.empty()) 
				{
					key_def hold(open_.top());
					if(hold.getState()->in_queue_==false)
						open_.pop();
					else
						break;	
				}
				if(open_.empty())
					return false;
				return true;	
			}
			int ComputeorImprovePath()
			{
				key_def hold_key;
				shared_state_def hold_state;
				shared_state_def hold_update_state;
				//g++ wants the typename in front or it gets confused.	
				typename state_map_def::iterator  succ_iter;				
				state_map_def hold_map;
				int states = 0;
				while( filterQueue() && 
							( key_def(open_.top(),eps_)
							  < key_def(start_,eps_)
							|| start_->g() != start_->g()) )
				{
					hold_key =  open_.top();
					hold_state = hold_key.getState();	
					hold_state->in_queue_=false;
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
							UpdateState(hold_update_state);
							succ_iter++;
						}
					}
					else
					{
						hold_state->setG(INF);
						assert(hold_state->getSuccessors().size()>0);
						succ_iter = hold_state->getSuccessors().begin();
						while(succ_iter!= hold_state->getSuccessors().end())
						{
							hold_update_state = succ_iter->second;
							buildState(hold_update_state);
							assert(hold_update_state->getSuccessors().size()>0);
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
