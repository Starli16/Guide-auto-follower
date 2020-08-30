#pragma once
#include <cmath>
using namespace std;
template<typename T>
struct AverageFilter
{
    int FilterLength;
    int num;
    T Filter[20];
    T upperbound,lowerbound,maxchange;
    AverageFilter(int,T,T,T MaxChange=9999999.0);
    bool AddElement(T Element);
    bool Check(T Element);
    T GetValue();
};

template<typename T>
AverageFilter<T>::AverageFilter(int length,T LowerBound,T UpperBound,T MaxChange){
    FilterLength=length;
    num=0;
    upperbound=UpperBound;
    lowerbound=LowerBound;
    maxchange=MaxChange;
    for(int i=0;i<length;i++) Filter[i]=0;
}

template<typename T>
bool AverageFilter<T>::AddElement(T Element){
    if(!Check(Element)) return false;
    if(num<FilterLength){
        Filter[num]=Element;
        num++;
    }
    else{
        for(int i=0;i<FilterLength-1;i++) Filter[i]=Filter[i+1];
        Filter[FilterLength-1]=Element;
    }
    return true;
}
template<typename T>
bool AverageFilter<T>::Check(T Element){
    if(Element>upperbound || Element<lowerbound) {
        return false;
    }
    if(num>0){
        if(abs(GetValue()-Element) > maxchange)  return false;
    }
    return true;
}
template<typename T>
T AverageFilter<T>::GetValue(){
    T sum=0;
    for(int i=0;i<num;i++) sum=sum+Filter[i];
    if(num>0)  return sum/num;
    else return 0;
}
