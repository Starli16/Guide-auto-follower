template<typename T>
struct AverageFilter
{
    int FilterLength;
    int num;
    T Filter[20];
    AverageFilter(int);
    void AddElement(T Elenment);
    T GetValue();
};

template<typename T>
AverageFilter<T>::AverageFilter(int length){
    FilterLength=length;
    num=0;
    for(int i=0;i<length;i++) Filter[i]=0;
}

template<typename T>
void AverageFilter<T>::AddElement(T Element){
    if(num<FilterLength){
        Filter[num]=Element;
        num++;
    }
    else{
        for(int i=0;i<FilterLength-1;i++) Filter[i]=Filter[i+1];
        Filter[FilterLength-1]=Element;
    }
    
}
template<typename T>
T AverageFilter<T>::GetValue(){
    T sum=0;
    for(int i=0;i<num;i++) sum=sum+Filter[i];
    return sum/num;
}
