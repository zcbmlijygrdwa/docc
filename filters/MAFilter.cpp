/*============================
  This class is an implementation of a moving average filter. This is a single-input and single-output filter.

  The class take the raw data as the input, and returns the filtered data as the output.

Input:
dataIn: raw data input

Ouput:
filteredData: filtered data

Parameters:
None

Usage:

Filter maf;
maf.init();
maf.setBufferSize(15);

......
realTimeLoop
{
....
filteredData = maf.put(rawData);
...
}

maf.reset(); //reset the filter


Created by Zhenyu Yang on 2017/10/25

==============================================*/
#include "MAFilter.h"
#include <iostream>
void MAFilter::init()
{
	bufferedSize = 0;
	totalSize = 10;
	ifBufferFull = false;
	sum = 0;
}

float MAFilter::put(float dataIn)
{

	if(bufferedSize<totalSize)
	{
		buffer[bufferedSize] = dataIn;
		bufferedSize++;
	}
	else
	{
		ifBufferFull = true;
		bufferedSize = 0;
		buffer[bufferedSize] = dataIn;
		bufferedSize++;
	}

	getSum();

	if(!ifBufferFull)
	{
		filteredData =  sum/bufferedSize;

	}
	else
	{

		filteredData =  sum/totalSize;
	}

	return filteredData;

}


void MAFilter::setBufferSize(int n)
{
	buffer = new float[n];
	totalSize = n;
}

void MAFilter::getSum()
{
	sum = 0;

	if(!ifBufferFull)
	{
		for(int i = 0 ;i<bufferedSize;i++)
		{
			sum+=buffer[i];
		}

	}

	else
	{

		for(int i = 0 ;i<totalSize;i++)
		{
			sum+=buffer[i];
		}
	}
}

void MAFilter::reset()
{
	ifBufferFull = false;
	bufferedSize = 0;

	for(int i = 0 ;i<totalSize;i++)
	{
		buffer[i] = 0;
	}

}
