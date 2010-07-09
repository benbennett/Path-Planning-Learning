
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
#ifndef STATE_HPP_
#define STATE_HPP_
#include <vector> 
#include <iterator>
#include <cmath>
#include <queue>
#include <functional>
#include <boost/unordered_set.hpp>
#include <sstream>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/functional/hash.hpp>
#include <iostream>
namespace  planning 
{

	const double INF=10000000.0;
	using namespace boost;
	using namespace std;
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
				shared_ptr< State<Z,R> >start_;
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
				State(boost::shared_ptr< State <Z,R> >  goal,tuple pos){

					in_queue_= false;
					clear();
					goal_  = goal;
				}
				State(tuple pos, boost::shared_ptr< State <Z,R> >  start,boost::shared_ptr< State <Z,R> >  goal):successors_()
				{
					in_queue_= false;
					clear();
					start_ = start;
					point_ = pos;
					goal_  = goal;
				}
				void setStart(boost::shared_ptr< State <Z,R> > & start)
				{
					start_ = start;
				}

				void setGoal(boost::shared_ptr< State <Z,R> > & goal)
				{
					goal_ = goal;
				}
				friend std::size_t  hash_value(State<Z,R> const & in)
				{
					std::size_t seed=0;
					boost::hash_combine(seed,in.point_[0]);
					boost::hash_combine(seed,in.point_[1]);
					return seed;

				}
			
					
				std::map< std::vector<Z> , shared_ptr< State<Z,R> > >  getSuccessors() const
				{
					return successors_;
				}
				boost::shared_ptr<	State <Z,R> > getMinSuccessor()
				{
					R min = -1;
					typename std::map< std::vector<Z> , shared_ptr< State<Z,R> > >::iterator pos;
					State<Z,R> hold;
					if(!(successors_.size()>0))
					{
						std::stringstream ss(std::stringstream::out); 
						ss<<"("<<point_[0]<<","<<point_[1]<<")"<< "call to getSuccessors no successors "<<std::endl;
						std::cerr<<ss.str();
						throw 20; 
					}
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
						
					return min_successor_;
				}

				R  getMinSuccessorValue()
				{
					getMinSuccessor();
					return min_successor_value_;	
				}
				void removeSuccessor(shared_ptr< State<Z,R> > in)
				{
			assert(in!=NULL);
			successors_.erase(in->getPoint());
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

				R csprimeGsprime(State<Z,R>  & sprime  ) 
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
					R mr=-1;
					mr=-1;
					R max2 =-1;
					for(int i=0;i<2;i++)
					{
						mr = std::abs(start_->point_[i] - point_[i]);
						if(mr>max2)
							max2=mr;
					}
					
					return max2;
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
					return os;
				}
		};

}
#endif
