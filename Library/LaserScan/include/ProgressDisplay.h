
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/



/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : March 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Different template classes for showing the progress of a lengthy operation.
*
*--------------------------------------------------------------------*/

#ifndef __PROGRESS_DISPLAY__H__
#define __PROGRESS_DISPLAY__H__
#include<string>
#include<iostream>
#include<iomanip>
#include <cstdio>
#include <cstdarg>
using std::string;
using std::cerr;
using std::flush;
using std::endl;
template <class T>
class ProgressDisplay
{
public:	
	ProgressDisplay(T _interval=100,T _total=0,string msg="Progress")
	{
		Initialize(_interval,_total,msg);
	}
	
	void Initialize(T _interval=100,T _total=0,string msg="Progress")
	{
		interval = _interval;
		total = _total;
		lastDisplay = 0;
		defaultMessage = msg;
		tickCounter = 0;
	}
	
	ProgressDisplay(T _total,string msg = "Progress ",T _times=100)
	{
		Initialize(_total,msg ,_times);
	}
	
	void Initialize(T _total,string msg = "Progress ",T _times=100)
	{
		total = _total;
		interval = total/_times;
		lastDisplay = 0;
		defaultMessage = msg;
		tickCounter = 0;
	}
	
	virtual void Step(const char* msg,int count)
	{
		if(lastDisplay<0)
		{
			return;
		}
		else if(!lastDisplay || (count - lastDisplay)>=interval || count == total)
		{
			if(!msg)
				msg = defaultMessage.c_str();
			else
				defaultMessage = msg;
				
			cerr<<msg<<"  "<<count<<"";
			if(total>0)
			{
				cerr<<"/"<<total<<"     ";
				cerr<<std::setw(6)<<std::setprecision(3)<<(double)count/(double)total*100.00<<"%";
			}
			cerr<<"    \r";
			lastDisplay = count;
		}
	}
	
	void ShowMessage(const char* msg)
	{
		//Change last display to force display.
		lastDisplay = 0;
		//Now call step with the stored value.
		this->Step(msg,lastStep);	
	}
	
	//Functor.
	void operator()(const char* msg = NULL, int ticks=1)const
	{
		(*((ProgressDisplay<T>*)this))(msg,ticks);
	}
	
	//Functor.
	void operator()(const char* msg,int ticks=1)
	{
		Step(msg,tickCounter+=ticks);
	}
	
	//Reset the object
	virtual void Reset(const char* newMsg=NULL)
	{
		if(newMsg)
			defaultMessage = newMsg;
		lastDisplay = 0;
		tickCounter = 0;
		lastDisplay = 0;
		Step(NULL,0);
	}
		
	//Reset the object
	void Reset(const char* newMsg=NULL)const
	{
		(*((ProgressDisplay<T>*)this)).Reset(newMsg);
	}
	
	virtual void End(const char* msg=NULL)
	{
		if(!msg)
			msg = defaultMessage.c_str();
			
		Step(msg,total);
			
		if(lastDisplay>=0)
		{
			cerr<<"\n"<<msg<<"  Done!!!\n";
			//Reset();
		}
	}
	
	///Access function.
	void SetTotal(int newTotal)
	{
		total = newTotal;	
	}
	
	///Access function.
	void SetInterval(int newInterval)
	{
		interval = newInterval;
	}
	
	//Destructor.
	//If end is not called call it.
	~ProgressDisplay()
	{
		//cerr<<"~ProgressDisplay for "<<this<<"  called\n"<<std::flush;
	}
	
	std::ostream& Print(std::ostream& os=std::cerr)
	{
		os<<"ProgressDisplay: "<<this<<endl;
		os<<"interval: "<<interval<<endl;
		os<<"total: "<<total<<endl;
		os<<"lastDisplay: "<<lastDisplay<<endl;
		os<<"tickCounter: "<< tickCounter<<endl;
		os<<"*** Progress Display ends ***"<<endl;
		return os;
	}
	
	
protected:
	T interval;
	T lastDisplay;
	T total;
	T tickCounter;
	T lastStep;
	string defaultMessage;

};	

///All classes should just use this rather than creating the ProgressDisplay objects themselves.
class ProgressDisplayManager
{
public:
	static ProgressDisplay<int>* GetProgressObject()
	{
		if(!globalProgress)
		{
			globalProgress = new ProgressDisplay<int>();
		}
		return globalProgress;
	}
	
	static void SetProgressObject(ProgressDisplay<int>* newObject)
	{
		delete globalProgress;
		globalProgress = newObject;
	}
private:
	static ProgressDisplay<int>* globalProgress;	
};

#endif// __PROGRESS_DISPLAY__H__
