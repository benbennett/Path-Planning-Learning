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
#include <boost/unordered_set.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/functional/hash.hpp>
namespace  planning 
{
	const double INF=10000000.0;
	using namespace boost;
	template <typename Z , typename R>
		class State
		{
			public: 
				typedef std::vector<Z> tuple;
				
			private:
				tuple point;
				tuple goal;
				R rhs_value;
				R gofs;
				//Have to use a pointer , otherwise it is incomplete type. 
				//IE doesn't know how to istatinate a instance 
				//Little confusing , dynamic 
				//
				unordered_set< State<Z,R> >successors_;
				shared_ptr<  State<Z,R> >min_successor_;
				shared_ptr< State<Z,R> >goal_;
			public:
				State()
				{
					
				}
				State(tuple pos)
				{
					this->point= pos;
				}
				/*   
				 *
				 */
				void init(tuple pos, boost::shared_ptr<tuple>  goal)
				{

				}
				friend std::size_t  hash_value(State<Z,R> const & in)
				{
					std::size_t seed=0;
					boost::hash_combine(seed,in.point[0]);
					boost::hash_combine(seed,in.point[1]);
					return 0;

				}

				State getMinSuccessor()
				{
					State<Z,R> mr;	
					return mr;
				}

				void removeSuccessor(State const & s)
				{

				}

				void rhs()
				{

				}

				void setRhs(R)
				{

				}

				void csprimeGsprime(State const & sprime )
				{


				}

				R cost(State & sprime)
				{
					R mr;
					return mr;
				}

				R g()
				{
					R mr;
					return mr;
				}

				void setG(R g )
				{

				}

				R  h()
				{
					R mr;
					return mr;
				}

				bool isGoal()
				{
					return true;
				}
				//TODO over load ostream operator and == operator

				bool operator==(State<Z,R> const & rhs) const
				{
					return rhs.point==this->point;
				}

				friend std::ostream& operator << (std::ostream& os, const State<Z,R>& in)
				{
					os<<"(";
					std::copy(in.point.begin(),in.point.end(),std::ostream_iterator <R>(os,","));	
					os<<")";
				}
		};


	template<typename Z, typename R> 
		class StateTransSpace 
		{
			public:
				StateTransSpace()
				{

				}
			private:

			public:

				std::vector< State<Z,R> > StateTranFunc(State<Z,R> at, std::vector< Z [2] >)
				{

				}


		};

	template<typename Z, typename R> 
		class Key 
		{
			public:
				Key();
				Key(R k1, R k2);
			private:
				R k1;
				R k2;
			public:

				bool operator==(const Key &rhs)
				{
					if(k1==rhs->k1 && k2==rhs->k2)
						return true;
					return false;
				}
				bool operator>=(const Key &rhs)
				{
					if(this>rhs || this==rhs)
						true;
					return false;
				}
				bool operator<=(const Key &rhs)
				{
					if (this < rhs || this=rhs)
						return true;
					return false;
				}
				bool operator!=(const Key &rhs)
				{
					if(this==rhs)
						return false;
					return true;
				}
				bool operator>(const Key &rhs)
				{	
					return !(this<rhs);
				}
				bool operator<(const Key &rhs)
				{
					if(k1< rhs->k1)
						return true;
					if(k1==rhs->k1 && k1<rhs->k2)
						return true;	
					return false;
				}
		};
}
