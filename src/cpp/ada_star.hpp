/*
   Simple implementation of
   "Anytime Dynamic A*: An Anytime, Replanning Algorithm" by
   Maxim Likhachev , Dave Ferguson , Geoff Gordon , Anthony Stentz , and Sebastian Thrun
   Warning this may be incorrect, mainly put together to learn from there paper.
   See paper at: 
http://www.ri.cmu.edu/pub_files/pub4/likhachev_maxim_2005_1/likhachev_maxim_2005_1.pdf
*/

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
namespace  planning 
{
	const double INF=10000000.0;
	using namespace boost;

	template <typename Z , typename R>
		class State
		{
			public: 
				typedef std::vector<Z> tuple;
				bool in_queue_;
			public:
				tuple point_;
				R rhs_value_;
				R gofs_;
				//Have to use a pointer , otherwise it is incomplete type. 
				//IE doesn't know how to istatinate a instance 
				//Little confusing , dynamic 
				//
				std::map< std::vector<Z> , shared_ptr< State<Z,R> > > successors_;
				shared_ptr<  State<Z,R> >min_successor_;
				R min_successor_value_;
				shared_ptr< State<Z,R> >goal_;
				void clear()
				{
					Z pt[] = {0,0};
					std::vector<Z> start_pt(pt,pt+2);
					rhs_value_=INF;
					gofs_ = INF; 

				}
			public:
				State():successors_()
				{
					in_queue_= false;
					clear();
				}
				State(tuple pos):successors_()
				{

					in_queue_= false;
					clear();
					this->point_= pos;

				}
				State(tuple pos, boost::shared_ptr< State <Z,R> >  goal):successors_()
				{
					in_queue_= false;
					clear();
					point_ = pos;
					goal_  = goal;
				}
				void setGoal(boost::shared_ptr< State <Z,R> >  goal)
				{
					goal_ = goal;
				}
				friend std::size_t  hash_value(State<Z,R> const & in)
				{
					std::size_t seed=0;
					boost::hash_combine(seed,in.point_[0]);
					boost::hash_combine(seed,in.point_[1]);
					return 0;

				}
			
					
				std::map< std::vector<Z> , shared_ptr< State<Z,R> > >  getSuccessors() const
				{
					return successors_;
				}
				State <Z,R> getMinSuccessor()
				{
					R min = -1;
					typename std::map< std::vector<Z> , shared_ptr< State<Z,R> > >::iterator pos;
					State<Z,R> hold;
					assert(successors_.size()>0);
					pos	= successors_.begin();
					while(pos!=successors_.end())
					{
						hold = *(pos->second);
						if( min<0 || min> csprimeGsprime(hold))
						{
							min = csprimeGsprime(hold);
							min_successor_ = pos->second;
						}	
						pos++;
					}
					min_successor_value_ = min;
						
					return *min_successor_;
				}

				R  getMinSuccessorValue()
				{
					getMinSuccessor();
					return min_successor_value_;	
				}
				void removeSuccessor(State const & s)
				{

				}
				void addSuccessor(shared_ptr< State<Z,R> > in)
				{
					assert(in!=NULL);
					successors_[in->getPoint()] = in;
				}

				R rhs()
				{
					return rhs_value_;
				}

				void setRhs(R in)
				{
					rhs_value_ = in; 	
				}

				R csprimeGsprime(State<Z,R> & sprime ) 
				{
					return R( cost(sprime) + sprime.g() ) ;
				}

				R cost(State<Z,R> const & sprime) 
				{
					return R (1) ;
				}

				R g()
				{
					return gofs_;
				}

				void setG(R g )
				{
					gofs_ = g;
				}

				R  h()
				{
					R mr=0;
					for(int i=0;i<2;i++)
					{
						mr = std::abs(goal_->point_[i] - point_[i]);
					}	
					return mr;
				}
				std::vector<Z> getPoint()
				{
					return point_;
				}
				bool isGoal()
				{
					return goal_->point_ == point_;
				}

				bool operator==(State<Z,R> const & rhs) const
				{
					return rhs.point_==this->point_;
				}
				friend std::ostream& operator << (std::ostream& os, const State<Z,R>& in)
				{
					os<<"(";
					std::copy(in.point_.begin(),in.point_.end(),std::ostream_iterator <R>(os,","));	
					os<<")";
				}
		};

	template<typename Z, typename R> 
		class Key 
		{
			public:
				Key( shared_ptr < State < Z , R >  > const & s,R  eps)
				{
					state_ = s;
					if( s->g()> s->rhs())
					{
						k1 = s->rhs()+eps*s->h();
						k2 = s->rhs();
					}
					else
					{
						k1 = s->g()+ s->h();
						k2 = s->g();
					}			
				}

				Key( Key<Z,R>  const & other,R  eps)
				{
					state_ = other.state_;

					if( state_ ->g()> state_ ->rhs())
					{
						k1 = state_ ->rhs()+eps*state_ ->h();
						k2 = state_ ->rhs();
					}
					else
					{
						k1 = state_ ->g()+ state_ ->h();
						k2 = state_ ->g();
					}			
				}
				Key()
				{
					k1 = INF;
					k2 = INF;
				}
				Key(R k1_in, R k2_in)
				{
					k1=k1_in; 
					k2=k2_in;
				}
			public:
				R k1;
				R k2;
				shared_ptr < State < Z , R >  > state_;
			public:
				void setState( shared_ptr < State < Z , R >  > in)
				{
					state_ = in;
				}
				shared_ptr < State < Z , R >  >  getState()
				{
					return state_;
				}
				bool operator==(const Key &rhs) const
				{
					if(k1==rhs.k1 && k2==rhs.k2)
						return true;
					return false;
				}
				bool operator>=(const Key &rhs) const
				{
					if(*this>rhs || *this==rhs )
						return true;
					return false;
				}
				bool operator<=(const Key &rhs) const
				{
					if (*this<rhs || *this==rhs)
						return true;
					return false;
				}
				bool operator!=(const Key &rhs) const
				{
					if(*this == rhs)
						return false;
					return true;
				}
				bool operator<(const Key &rhs) const
				{
					if(k1< rhs.k1)
						return true;
					if(k1==rhs.k1 && k2<rhs.k2)
						return true;	
					return false;
				}
				bool operator>(const Key &rhs) const
				{
					if( *this== rhs)
						return false;
					return !(*this< rhs);	
				}
				Key &  operator= (const Key & rhs)
				{
					k1 = rhs.k1;
					k2 = rhs.k2;
					state_ = rhs.state_;
				}
				friend std::ostream& operator << (std::ostream& os, const Key<Z,R>& in)
				{
					os<<in.k1 <<","<<in.k2;	
				}
		};


	template<typename Z, typename R> 
		class AnytimeDstar
		{
			typedef State<Z,R> state_def;	
			typedef shared_ptr < State<Z,R> > shared_state_def;	
			typedef std::vector<Z> vectorZ;
			typedef Key<Z,R> key_def;
			public:
			shared_ptr < State < Z, R> > start_; 
			shared_ptr < State < Z, R> > goal_; 
			
			
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
			AnytimeDstar(Z first, Z second):states_(),open_(),closed_(),incons_()

			{
				shared_state_def hold = createState(first,second);
				goal_ = hold;	
			}
			void init(shared_state_def start)
			{
				start_  = start;
				goal_->setRhs(0);
				goal_->in_queue_=true;	
				key_def goal_key(goal_,eps_);
				open_.push(goal_key);
			}	
			void setStart( shared_state_def start)
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
				if( hasKey(point))
				{
					return states_[point];
				}	
				shared_state_def  temp_ptr;
				temp_ptr.reset(new state_def(point));
				if(goal_==NULL)
					temp_ptr->setGoal(temp_ptr);
				else
					temp_ptr->setGoal(goal_);
				return temp_ptr;
			}

			void  buildState( shared_state_def in)
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
							shared_state_def hold =createState(first,second);
							in->addSuccessor(hold);	
						}
					}
					assert(in->getSuccessors().size()>0);
				}
			}
			void UpdateAllPriorities()
			{

				std::priority_queue< key_def, std::vector< key_def  >,std::greater< key_def > > newqueue;
				while(!open_.empty())
				{
					key_def hold = open_.top();
					newqueue.push( key_def(hold->getState(),eps_));
					open_.pop();	
				}
				open_= newqueue;
			}
			void UpdateState( shared_state_def s)
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
						//TODO FIX me open_.push( key_def(s,eps_));
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
					key_def hold = open_.top();
					if(hold.getState()->in_queue_==false)
						open_.pop();
					else
						break;	
				}
				if(open_.empty())
					return false;
				return true;	
			}
			void ComputeorImprovePath()
			{
				key_def hold_key;
				shared_state_def hold_state;
				shared_state_def hold_update_state;
				typename std::map< std::vector<Z>, shared_state_def >::iterator  succ_iter;				
				typename std::map< std::vector<Z>, shared_state_def > hold_map;	
				while( filterQueue() && 
							( key_def(open_.top(),eps_)
							  < key_def(start_,eps_)
							|| start_->g() != start_->g()) )
				{
					hold_key =  open_.top();
					hold_state = hold_key.getState();	
					hold_state->in_queue_=false;	
					open_.pop();
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

							hold_update_state = succ_iter->second;
							buildState(hold_update_state);
					/*		assert(hold_update_state->getSuccessors().size()>0);
					*/
							UpdateState(hold_update_state);
							succ_iter++;
						}
					}
					else
					{
						hold_state->setG(INF);
						UpdateState(hold_state);
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

			}

		};
}
