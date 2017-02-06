/* -----------------------------------------------------------------
 |Initial Creation: Biao Xiong
 |Data: Sep 21, 2NULL15
 |
 |
------------------------------------------------------------------- */
#ifndef __SINGLETON_H_
#define __SINGLETON_H_

#include <mutex> 

// assumes _DATA_TYPE_ has a default constructor
template<typename _DATA_TYPE_>
class singleton
{
public:
	static _DATA_TYPE_ * request();
	static void release();

private:
	singleton();
	singleton(singleton<_DATA_TYPE_> const & s);
	~singleton();
	singleton<_DATA_TYPE_> & operator =(singleton<_DATA_TYPE_> const & s);
	static _DATA_TYPE_ * pointer;
	static std::mutex mtx;
	// ...
};

template<typename _DATA_TYPE_>
_DATA_TYPE_ * singleton<_DATA_TYPE_>::pointer = NULL;

template<typename _DATA_TYPE_>
std::mutex singleton<_DATA_TYPE_>::mtx;

template<typename _DATA_TYPE_>
_DATA_TYPE_ * singleton<_DATA_TYPE_>::request()
{
	if(singleton<_DATA_TYPE_>::pointer == NULL)
	{
		singleton<_DATA_TYPE_>::mtx.lock();

		if(singleton<_DATA_TYPE_>::pointer == NULL)
		{
			singleton<_DATA_TYPE_>::pointer = new _DATA_TYPE_;
		}

		singleton<_DATA_TYPE_>::mtx.unlock();
	}

	return singleton<_DATA_TYPE_>::pointer;
}

template<typename _DATA_TYPE_>
void singleton<_DATA_TYPE_>::release()
{
	if(singleton<_DATA_TYPE_>::pointer != NULL)
	{
		singleton<_DATA_TYPE_>::mtx.lock();

		if(singleton<_DATA_TYPE_>::pointer != NULL)
		{
			delete singleton<_DATA_TYPE_>::pointer;

			singleton<_DATA_TYPE_>::pointer = NULL;
		}

		singleton<_DATA_TYPE_>::mtx.unlock();
	}
}

template<typename _DATA_TYPE_>
singleton<_DATA_TYPE_>::singleton()
{
	// ...
}

// ...

//int main()
//{
//	int * s;

//	s = singleton<int>::request();

	// ...

//	singleton<int>::release();

//	return NULL;
//}

#endif
