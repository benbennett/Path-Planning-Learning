/*
   Simple implementation of
   "Anytime Dynamic A*: An Anytime, Replanning Algorithm" by
   Maxim Likhachev , Dave Ferguson , Geoff Gordon , Anthony Stentz , and Sebastian Thrun

   Warning this may be incorrect, mainly put together to learn from there paper.
   See paper at: 
http://www.ri.cmu.edu/pub_files/pub4/likhachev_maxim_2005_1/likhachev_maxim_2005_1.pdf
*/

#include <vector> 
namespace  planning 
{
	template <typename Z , typename R>
		class State
		{
			public: 
				typedef std::vector<Z> tuple;	
			private:
				tuple point;
				/*tuple goal;
				  R rhs_value;
				  R gofs;
				  */
			public:
				State();
				/*   
				 *
				 */
				void init(tuple pos, const tuple & goal);

				/*
				 * Going to be used in the hash function for map methods 
				 */
				long hash();

				State getMinSuccessor();

				void removeSuccessor(State const & s); 

				void rhs();

				void setRhs(R);

				void csprimeGsprime(State const & sprime );

				R cost(State & sprime);

				R g();

				void setG(R g );

				R  h();

				bool isGoal();
				//TODO over load ostream operator and == operator

		};

	template<typename Z, typename R> 
		class StateTransSpace 
		{
			public:
				StateTransSpace();
			private:

			public:

				std::vector< State<Z,R> > StateTranFunc(State<Z,R> at, std::vector< Z [2] >);


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
				//simple overload of operators 
				bool operator==(const Key &rhs);
				bool operator>=(const Key &rhs);
				bool operator<=(const Key &rhs);
				bool operator!=(const Key &rhs);
				bool operator>(const Key &rhs);
				bool operator<(const Key &rhs);
		};
}
