namespace Bivariate
{
	template <typename T>
	void plot(const vector<T>& x, const vector<T>& y);

	template <typename T>
	string buildRangeString(const vector<T>& x);

	template <typename T>
	void plot_bivariate_independent(T u1, T s1, T u2, T s2);

	template <typename T>
	void plot_bivariate(T u1, T s1, T u2, T s2, T rho);
}

namespace Bivariate {


	template <typename T>
	T sample_normal(T u, T s)
	{
		typedef boost::mt19937 RNGType;
		RNGType gen;
		boost::normal_distribution<T> nd(u,s);
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > randNormal(gen, nd);
		return randNormal();
	}


	template <typename T>
	matrix<T> sample_normal(T u, T s, int numObservations)
	{
		return matrix<T>(2,2,3);
	}




	template <typename T>
	void plot_bivariate(T u1, T s1, T u2, T s2, T rho)
	{

		if(rho<-1 || rho>1)
			throw std::runtime_error("Correlation cannot be smaller than -1 or larger than +1");


		int numStds = 3; //How many std deviations to plot
		int numPoints = 2000;


		T cov = rho * s1 * s2;


		//Cholesky decompisition
		matrix<T> A(2,2);
		A(0,0) = s1;
		A(0,1) = 0;
		A(1,0) = cov / s1;
		A(1,1) = sqrt(s1*s1*s2*s2 - cov) / s1;



		typedef boost::mt19937 RNGType;
		RNGType gen;
		boost::normal_distribution<T> nd(0.0, 1.0);
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > randNormal(gen, nd);

		vector<T> x1, x2;
		matrix<T> z(2,1), v(2,1), u(2,1);

		u(0,0) = u1;
		u(1,0) = u2;


		for(int i = 0; i < numPoints; i++)
		{
		
			z(0,0) = randNormal();
			z(1,0) = randNormal();

			boost::numeric::ublas::axpy_prod(A,z,v,true);
			
			v+=u;
			
			x1.push_back(v(0,0));
			x2.push_back(v(1,0));

		}

		plot(x1,x2);
	}



	template <typename T>
	void plot_bivariate_independent(T u1, T s1, T u2, T s2)
	{

		int numStds = 3; //How many std deviations to plot
		int numPoints = 2000;

		typedef boost::mt19937 RNGType;
		RNGType gen;
		boost::normal_distribution<T> nd1(u1, s1);
		boost::normal_distribution<T> nd2(u2, s2);

		boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > randNormal1(gen, nd1);
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > randNormal2(gen, nd2);

		vector<T> x1, x2;
		for(int i = 0; i < numPoints; i++)
		{
			x1.push_back(randNormal1());
			x2.push_back(randNormal2());
		}

		plot(x1,x2);
	}



	template <typename T>
	void plot(const vector<T>& x, const vector<T>& y)
	{

		if(x.size()!=y.size())
			throw std::runtime_error("When plotting, x and y vectors must have same length.");


		if(x.size()==0)
			throw std::runtime_error("When plotting, must have at least 1 value to plot");


		Gnuplot gp;


		// Don't forget to put "\n" at the end of each line!
		gp << "set xrange " << buildRangeString(x) << std::endl;
		gp << "set yrange " << buildRangeString(y) << std::endl;
		// '-' means read from stdin.  The send1d() function sends data to gnuplot's stdin.
		gp << "plot '-'" << std::endl;
		gp.send1d(boost::make_tuple(x,y));
	}

	template <typename T>
	string buildRangeString(const vector<T>& x)
	{

		T min = *std::min_element(x.begin(),x.end());
		T max = *std::max_element(x.begin(),x.end());

		T range = max - min;
		T offset = 0.05 * range;
		min-=offset;
		max+=offset;

		return "[" + boost::lexical_cast<std::string>(min) + ":" + boost::lexical_cast<std::string>(max) + "]";
	}

}

