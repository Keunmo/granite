#include <granite/camera/brown_conrady_camera.hpp>
#include <granite/camera/pinhole_camera.hpp>
#include <granite/serialization/headers_serialization.h>
#include <cereal/archives/json.hpp>
#include <sstream>
#include <iostream>
#include <stdexcept>
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){
 using C=granite::BrownConradyCamera<double>;C::VecN p;p<<300,310,320,240,.08,-.02,.001,-.002,.003;C cam(p);
 for(double x:{-.8,0.,.7})for(double y:{-.6,0.,.5}){
  C::Vec4 point;point<<x,y,1.3,1;C::Vec2 uv;C::Mat24 J;C::Mat2N P;
  require(cam.project(point,uv,&J,&P),"project failed");
  C::Vec4 ray;C::Mat42 U;C::Mat4N V;require(cam.unproject(uv,ray,&U,&V),"unproject failed");
  require((ray.head<3>()-point.head<3>().normalized()).norm()<1e-10,"round trip");
  const double e=1e-6;
  for(int k=0;k<4;++k){auto a=point,b=point;a[k]+=e;b[k]-=e;C::Vec2 u,v;cam.project(a,u);cam.project(b,v);require(((u-v)/(2*e)-J.col(k)).norm()<1e-5,"point derivative");}
  for(int k=0;k<9;++k){auto a=p,b=p;a[k]+=e;b[k]-=e;C::Vec2 u,v;C(a).project(point,u);C(b).project(point,v);require(((u-v)/(2*e)-P.col(k)).norm()<1e-5,"intrinsic projection derivative");C::Vec4 ra,rb;require(C(a).unproject(uv,ra)&&C(b).unproject(uv,rb),"perturbed unprojection");require(((ra-rb)/(2*e)-V.col(k)).norm()<1e-6,"intrinsic inverse derivative");}
  for(int k=0;k<2;++k){auto a=uv,b=uv;a[k]+=e;b[k]-=e;C::Vec4 ra,rb;cam.unproject(a,ra);cam.unproject(b,rb);require(((ra-rb)/(2*e)-U.col(k)).norm()<1e-6,"pixel inverse derivative");}
  auto fc=cam.cast<float>();Eigen::Vector4f fr;require(fc.unproject(uv.cast<float>(),fr),"float inverse");require((fr.cast<double>()-ray).norm()<1e-4,"float precision");
 }
 C::Vec4 behind;behind<<0,0,-1,1;C::Vec2 uv;require(!cam.project(behind,uv),"negative depth accepted");
 auto zero=p;zero.tail<5>().setZero();C z(zero);granite::PinholeCamera<double> pin(p.head<4>());C::Vec4 point;point<<.4,.3,1,1;C::Vec2 a,b;z.project(point,a);pin.project(point,b);require((a-b).norm()<1e-12,"zero distortion pinhole mismatch");
 // Numerical values from cv2.projectPoints for these coefficients (OpenCV order).
 cam.project(point,a);require(std::abs(a.x()-441.985625)<1e-6 && std::abs(a.y()-334.732609375)<1e-6,"OpenCV reference mismatch");
 granite::GenericCamera<double> generic;generic.variant=cam;std::stringstream stream;
 {cereal::JSONOutputArchive ar(stream);ar(generic);}granite::GenericCamera<double> loaded;
 {cereal::JSONInputArchive ar(stream);ar(loaded);}require(loaded.getName()=="brown_conrady","serialization model");require((loaded.getParam()-p).norm()<1e-12,"serialization parameters");
 std::cout<<"Brown camera: project/unproject, all analytic Jacobians, float, pinhole limit, OpenCV reference and serialization PASS\n";
}
