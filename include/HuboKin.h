#ifndef _HUBOKIN_H_
#define _HUBOKIN_H_

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <complex>

#define HUBOKIN_USE_KCONSTANTS

namespace HK {

  typedef Eigen::Matrix< float, 6, 1 > Vector6f;
  typedef Eigen::Vector3f Vector3f;
  typedef Eigen::Isometry3f Isometry3f;
  typedef Eigen::Matrix< float, 6, 2 > Matrix62f;
  typedef std::vector<int> IntArray;

  class HuboKin {
  public:

    enum {
      SIDE_RIGHT = 0,
      SIDE_LEFT = 1
    };

    struct KinConstants {

	  float arm_l1, arm_l2, arm_l3, arm_l4;
	  float leg_l1, leg_l2, leg_l3, leg_l4, leg_l5, leg_l6;

	  Matrix62f arm_limits;
	  Matrix62f leg_limits;
    
	  Vector6f  arm_offset;
	  Vector6f  leg_offset;

      IntArray arm_mirror;
      IntArray leg_mirror;

      KinConstants();


	  Matrix62f getArmLimits(int side) const;
	  Matrix62f getLegLimits(int side) const;
	  Vector6f  getArmOffset(int side) const;
	  Vector6f  getLegOffset(int side) const;

    };

    KinConstants kc;

	static Matrix62f mirrorLimits(const Matrix62f& orig, const IntArray& mirror);
	static Vector6f  mirrorAngles(const Vector6f& orig, const IntArray& mirror);

	static void DH2HG(Isometry3f &B, float t, float f, float r, float d);

	void armFK(Isometry3f &B, const Vector6f &q, int side) const;

	void armFK(Isometry3f &B, const Vector6f &q, int side,
			   const Isometry3f &endEffector) const;

	void armIK(Vector6f &q, const Isometry3f& B,
			   const Vector6f& qPrev, int side) const;

	void armIK(Vector6f &q, const Isometry3f& B,
			   const Vector6f& qPrev, int side,
			   const Isometry3f &endEffector) const;

	void legFK(Isometry3f &B, const Vector6f &q, int side) const;

	void legIK(Vector6f &q, const Isometry3f& B,
			   const Vector6f& qPrev, int side) const;


  };



}

#endif


