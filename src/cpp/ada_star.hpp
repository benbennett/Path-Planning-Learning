/*
   Simple implementation of
   "Anytime Dynamic A*: An Anytime, Replanning Algorithm" by
   Maxim Likhachev , Dave Ferguson , Geoff Gordon , Anthony Stentz , and Sebastian Thrun

   Warning this may be incorrect, mainly put together to learn from there paper.
   See paper at: 
http://www.ri.cmu.edu/pub_files/pub4/likhachev_maxim_2005_1/likhachev_maxim_2005_1.pdf
*/

#include <vector> 
#include <boost/shared_ptr.hpp>
namespace  planning 
{
	const double INF=10000000.0;

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

			public:
				State()
				{

				}
				State(tuple in)
				{
					this->point= in;
				}
				/*   
				 *
				 */
				void init(tuple pos, boost::shared_ptr<tuple>  goal)
				{

				}

				/*
				 * Going to be used in the hash function for map methods 
				 */
				long hash()
				{
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

				bool operator==(const State  & rhs);
		};
	template<typename Z, typename R> 
		std::size_t  hash_value(State<Z,R> const & in);


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
