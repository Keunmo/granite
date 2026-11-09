// SPDX-License-Identifier: MIT
// Brown-Conrady camera for Masul's Granite integration, 2026.
// Parameter order matches OpenCV: fx fy cx cy k1 k2 p1 p2 k3.
#pragma once
#include <granite/utils/sophus_utils.hpp>
#include <limits>
namespace granite {
template<typename Scalar=double> class BrownConradyCamera {
 public:
  static constexpr int N=9;
  using Vec2=Eigen::Matrix<Scalar,2,1>;
  using Vec4=Eigen::Matrix<Scalar,4,1>;
  using VecN=Eigen::Matrix<Scalar,N,1>;
  using Mat22=Eigen::Matrix<Scalar,2,2>;
  using Mat24=Eigen::Matrix<Scalar,2,4>;
  using Mat2N=Eigen::Matrix<Scalar,2,N>;
  using Mat42=Eigen::Matrix<Scalar,4,2>;
  using Mat4N=Eigen::Matrix<Scalar,4,N>;
  BrownConradyCamera(){param.setZero();}
  explicit BrownConradyCamera(const VecN& p):param(p){}
  template<class S> BrownConradyCamera<S> cast() const {return BrownConradyCamera<S>(param.template cast<S>());}
  static const std::string getName(){return "brown_conrady";}
  const VecN& getParam() const{return param;}
  void operator+=(const VecN& inc){param+=inc;}
  void setFromInit(const Vec4& p){param.setZero();param.template head<4>()=p;}

  // Distorted normalized coordinates and exact local/parameter derivatives.
  void distort(const Vec2& v,Vec2& d,Mat22* J=nullptr,Mat2N* P=nullptr) const {
    const Scalar x=v[0],y=v[1],r2=x*x+y*y,r4=r2*r2,r6=r4*r2;
    const Scalar k1=param[4],k2=param[5],p1=param[6],p2=param[7],k3=param[8];
    const Scalar a=1+k1*r2+k2*r4+k3*r6,b=k1+2*k2*r2+3*k3*r4;
    d<<x*a+2*p1*x*y+p2*(r2+2*x*x),y*a+p1*(r2+2*y*y)+2*p2*x*y;
    if(J){(*J)<<a+2*x*x*b+2*p1*y+6*p2*x,2*x*y*b+2*p1*x+2*p2*y,
                    2*x*y*b+2*p1*x+2*p2*y,a+2*y*y*b+6*p1*y+2*p2*x;}
    if(P){P->setZero();P->col(4)<<x*r2,y*r2;P->col(5)<<x*r4,y*r4;
      P->col(6)<<2*x*y,r2+2*y*y;P->col(7)<<r2+2*x*x,2*x*y;P->col(8)<<x*r6,y*r6;}
  }
  bool project(const Vec4& p,Vec2& uv,Mat24* point=nullptr,Mat2N* intr=nullptr) const {
    if(!p.allFinite()||!param.allFinite()||param[0]<=0||param[1]<=0||p[2]<=Sophus::Constants<Scalar>::epsilonSqrt())return false;
    Vec2 xy=p.template head<2>()/p[2],d;Mat22 J;Mat2N P;distort(xy,d,&J,intr?&P:nullptr);
    uv<<param[0]*d[0]+param[2],param[1]*d[1]+param[3];if(!uv.allFinite())return false;
    if(point){Mat24 D;D.setZero();D(0,0)=1/p[2];D(1,1)=1/p[2];D(0,2)=-xy[0]/p[2];D(1,2)=-xy[1]/p[2];
      *point=param.template head<2>().asDiagonal()*J*D;}
    if(intr){*intr=param.template head<2>().asDiagonal()*P;(*intr)(0,0)=d[0];(*intr)(1,1)=d[1];(*intr)(0,2)=1;(*intr)(1,3)=1;}
    return true;
  }
  bool unproject(const Vec2& uv,Vec4& ray,Mat42* pixel=nullptr,Mat4N* intr=nullptr) const {
    if(!uv.allFinite()||!param.allFinite()||param[0]<=0||param[1]<=0)return false;
    Vec2 target;target<<(uv[0]-param[2])/param[0],(uv[1]-param[3])/param[1];
    Vec2 xy=target,d;Mat22 J;const Scalar tol=std::numeric_limits<Scalar>::epsilon()*Scalar(100);
    bool converged=false;
    for(int i=0;i<40;++i){
      distort(xy,d,&J);Vec2 error=d-target;
      if(!d.allFinite()||J.determinant()<=tol)return false;
      if(error.template lpNorm<Eigen::Infinity>()<=tol){converged=true;break;}
      Vec2 delta=J.inverse()*error;Scalar step=1;bool accepted=false;
      for(int k=0;k<16;++k){Vec2 candidate=xy-step*delta,cd;distort(candidate,cd);
        if(cd.allFinite()&&(cd-target).squaredNorm()<error.squaredNorm()){xy=candidate;accepted=true;break;}step*=Scalar(.5);}
      if(!accepted)return false;
    }
    if(!converged)return false;
    const Scalar n=std::sqrt(1+xy.squaredNorm());ray<<xy[0]/n,xy[1]/n,1/n,0;
    if(pixel||intr){
      Eigen::Matrix<Scalar,3,1> v;v<<xy,1;
      Eigen::Matrix<Scalar,3,3> norm=Eigen::Matrix<Scalar,3,3>::Identity()/n-v*v.transpose()/(n*n*n);
      Mat42 L;L.setZero();L.template topRows<3>()=norm.template leftCols<2>()*J.inverse();
      if(pixel)*pixel=L*param.template head<2>().cwiseInverse().asDiagonal();
      if(intr){Mat2N P;distort(xy,d,nullptr,&P);P=-P;
        P(0,0)=-target[0]/param[0];P(1,1)=-target[1]/param[1];P(0,2)=-1/param[0];P(1,3)=-1/param[1];*intr=L*P;}
    }
    return true;
  }
  static Eigen::aligned_vector<BrownConradyCamera> getTestProjections(){
    VecN p;p<<300,310,320,240,.08,-.02,.001,-.002,.003;return {BrownConradyCamera(p)};}
  static Eigen::aligned_vector<Eigen::Vector2i> getTestResolutions(){return {Eigen::Vector2i(640,480)};}
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:VecN param;
};
}
