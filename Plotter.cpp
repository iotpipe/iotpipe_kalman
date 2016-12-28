#include "Plotter.h"
#include "boost/lexical_cast.hpp"


namespace Plotter
{
    bool first = true;
    Gnuplot gp;
}


using std::string;

string Plotter::GenerateStyleString(string options)
{
    string style;

    string color = GetColorStyle(options);
    if(color!=string(""))
    {
        options.erase(0,1);
        style+=color;
        style+=" ";
    }

    string plotType = GetPlotTypeStyle(options);
    if(plotType!=string(""))
    {
        options.erase(0,1);
        style+=plotType;
        style+=" ";
    }


    //This one goes last
    string size = GetSizeStyle(options);
    if(size!=string(""))
    {
        style+=size;
    }

    return style;
}

string Plotter::GetPlotTypeStyle(string options)
{
    if(options.length()==0)
    {
        return string("");
    }

    char t = (char)options[0];

    switch(t)
    {
        case '-':
            return string("w l");
            break;
        case '+':
            return string("with points");
            break;
        default:
            return string("");
            break;
    }
}

string Plotter::GetSizeStyle(string options)
{
    if(options.length()==0)
        return string("");


    try
    {
        int ps = boost::lexical_cast<int>( options );
        return string("lw ") + options;
    } 
    catch( boost::bad_lexical_cast const& ) 
    {
        return string("");
    }

                
}

string Plotter::GetColorStyle(string options)
{
    if(options.length()==0)
        return string("");

    string color = ColorMapping((char)options[0]);
    if(color.length()==0)
        return string("");
    else
        return string("lc rgb \"") + color + string("\""); 
}

string Plotter::ColorMapping(char c)
{
    switch(c)
    {
        case 'y':
            return string("yellow");
            break;
        case 'm':
            return string("magenta");
            break;
        case 'c':
            return string("cyan");
            break;
        case 'r':
            return string("red");
            break;
        case 'g':
            return string("green");
            break;
        case 'b':
            return string("blue");
            break;
        case 'w':
            return string("white");
            break;
        case 'k':
            return string("black");
            break;
        default:
            return string("");
            break;
    }        
}