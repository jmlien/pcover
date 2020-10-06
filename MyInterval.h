#pragma once

#include <utility>

namespace GMUCS425
{

class MyInterval
{
public:

  MyInterval(){s=e=DBL_MAX;}
  MyInterval(double _s, double _e):s(_s), e(_e){}
  MyInterval(const MyInterval& o):s(o.s), e(o.e){}
  MyInterval(const std::pair<double,double>& p):s(p.first), e(p.second){}

  bool overlap(const MyInterval& o) const {
    if(e<o.s) return false;
    if(s>o.e) return false;
    return true;
  }

  MyInterval intersect(const MyInterval& o) const {
    MyInterval ans;
    if(overlap(o)){
      ans.s=max(s,o.s);
      ans.e=min(e,o.e);
    }
    return ans;
  }

  //undefined when the intervals do not overlap
  MyInterval merge(const MyInterval& o) const {
    MyInterval ans;
    if(overlap(o)){
      ans.s=min(s,o.s);
      ans.e=max(e,o.e);
    }
    return ans;
  }

  MyInterval operator+(double v){
  	return MyInterval(s+v,e+v);
  }

  bool valid() const {
  	if(e<s) return false;
  	if(s==e && s==DBL_MAX) return false;
  	return true;
  }

  double length() const { return e-s; }

  void set(double _s, double _e){s=_s; e=_e;}
  void set(const std::pair<double,double>& p){s=p.first; e=p.second;}

  double s,e; //start and end of an interval, assuming s<=e
};


//max dist from one end of i1 to the other end of i2
inline double span(const MyInterval& i1, const MyInterval& i2){
	double s=min(i1.s,i2.s);
	double e=max(i1.e,i2.e);
	return e-s;
}


//if i1&i2 do not overlap, the gap between them is returned
inline double gap(const MyInterval& i1, const MyInterval& i2){
	if(i1.overlap(i2)) return 0;
	if(i1.s<i2.s) return i2.s-i1.e;
	return i1.s-i2.e;
}

}//end namespace GMUCS425
