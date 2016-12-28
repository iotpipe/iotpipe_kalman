#pragma once

#include <string>
#include <vector>

#include "boost/lexical_cast.hpp"
#include "/home/akamor/gnuplot-iostream/gnuplot-iostream.h"

namespace Plotter
{

	extern bool first;
    extern Gnuplot gp;


    using std::string;

    string GenerateStyleString(string options);
    
    string GetSizeStyle(string options);
    
	string GetColorStyle(string options);
	
    string GetPlotTypeStyle(string options);

    string ColorMapping(char c);
    

	template <typename T>
	void CheckPlotArguments(std::vector<T> x, std::vector<T> y, string options);
	
	template <typename T>	
	void SendGNUPlotCommand(std::vector<T> x, std::vector<T> y, string options, char sep=',');
	
	template <typename T>
	void plot(std::vector<T> x, std::vector<T> y, string options);
	
	template<typename T, typename... Args>
	void plot(std::vector<T> x, std::vector<T> y, string options, Args ...args);
	
    template <typename T>
	string buildRangeString(const std::vector<T>& x);
}

template <typename T>
void Plotter::CheckPlotArguments(std::vector<T> x, std::vector<T> y, string options)
{
    if(x.size()!=y.size())
    {
        throw std::runtime_error("When plotting, x and y must have same length.");
    }

    if(x.size()==0)
    {
        throw std::runtime_error("When plotting, must have at least 1 value to plot");
    }		
}

template <typename T>	
void Plotter::SendGNUPlotCommand(std::vector<T> x, std::vector<T> y, string options, char sep /* =',' */)
{
    if(first==true)
    {
        first = false;
        gp << "plot " << gp.binFile1d(boost::make_tuple(x,y),"record") << GenerateStyleString(options) << sep;
        std::cout << "plot " << gp.binFile1d(boost::make_tuple(x,y),"record") << GenerateStyleString(options) << sep;
    }
    else
    {
        gp << gp.binFile1d(boost::make_tuple(x,y),"record") << GenerateStyleString(options) << sep;
        std::cout << gp.binFile1d(boost::make_tuple(x,y),"record") << GenerateStyleString(options) << sep;
    }		
}


template <typename T>
void Plotter::plot(std::vector<T> x, std::vector<T> y, string options)
{
    Plotter::CheckPlotArguments(x,y,options);
    Plotter::SendGNUPlotCommand(x,y,options,'\n');
    first = true;
}

template<typename T, typename... Args>
void Plotter::plot(std::vector<T> x, std::vector<T> y, string options, Args ...args)
{
    Plotter::CheckPlotArguments(x,y,options);

    Plotter::SendGNUPlotCommand(x,y,options);

    plot(args...);
}

template <typename T>
std::string Plotter::buildRangeString(const std::vector<T>& x)
{

    T min = *std::min_element(x.begin(),x.end());
    T max = *std::max_element(x.begin(),x.end());

    T range = max - min;
    T offset = 0.05 * range;
    min-=offset;
    max+=offset;

    return "[" + boost::lexical_cast<std::string>(min) + ":" + boost::lexical_cast<std::string>(max) + "]";
}