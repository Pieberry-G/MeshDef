#pragma once

#include <Eigen\Eigen>

#include <set>
#include <array>

class Quadric
{
protected: // data
	double a2, ab, ac, ad;
	double     b2, bc, bd;
	double         c2, cd;
	double             d2;
	double area;

public: // data access
	double getArea(void) const { return area; }

protected: // init function
	void init(double a, double b, double c, double d, double p_area, double e)
	{
		a2 = a*a; ab = a*b; ac = a*c; ad = a*d;
		b2 = b*b; bc = b*c; bd = b*d;
		c2 = c*c; cd = c*d;
		d2 = d*d;
		area = p_area;
	}

	void init(double a, double b, double c, double d, double p_area)
	{
		a2 = a*a; ab = a*b; ac = a*c; ad = a*d;
		b2 = b*b; bc = b*c; bd = b*d;
		c2 = c*c; cd = c*d;
		d2 = d*d;
		area = p_area;
	}

public: // constructor
	Quadric(void) {}

	Quadric(double a, double b, double c, double d, double p_area, double e)
	{
		init(a, b, c, d, p_area, e);
	}

	Quadric(double a, double b, double c, double d, double p_area)
	{
		init(a, b, c, d, p_area);
	}

	void init(const Eigen::Vector3d& abc, double d, double p_area, double e)
	{
		init(abc[0], abc[1], abc[2], d, p_area, e);
	}

	Quadric(const Eigen::Vector3d& abc, double d, double p_area, double e)
	{
		init(abc[0], abc[1], abc[2], d, p_area, e);
	}

	Quadric(const Eigen::Vector3d& abc, double d, double p_area)
	{
		init(abc[0], abc[1], abc[2], d, p_area);
	}

	Quadric(const Quadric& other)
	{
		a2 = other.a2; ab = other.ab; ac = other.ac; ad = other.ad;
		b2 = other.b2; bc = other.bc; bd = other.bd;
		c2 = other.c2; cd = other.cd;
		d2 = other.d2;
		area = other.area;
	}

public: // operators
	Quadric& operator+=(const Quadric& other)
	{
		a2 += other.a2; ab += other.ab; ac += other.ac; ad += other.ad;
		b2 += other.b2; bc += other.bc; bd += other.bd;
		c2 += other.c2; cd += other.cd;
		d2 += other.d2;
		area += other.area;

		return *this;
	}

	Quadric& operator*=(double l)
	{
		a2 *= l; ab *= l; ac *= l; ad *= l;
		b2 *= l; bc *= l; bd *= l;
		c2 *= l; cd *= l;
		d2 *= l;

		return *this;
	}

	double solve(Eigen::Vector3d& bestPos) const
	{
		Eigen::Matrix3d m;
		m << a2, ab, ac,
			ab, b2, bc,
			ac, bc, c2;

		// calculate the inverse matrix and determinate at the same time efficiently
		Eigen::Matrix3d adj;
		adj << m.row(1).cross(m.row(2)),
			m.row(2).cross(m.row(0)),
			m.row(0).cross(m.row(1));
		double det = adj.row(0).dot(m.row(0));
		if (abs(det) > 1e-8)
		{
			bestPos = (adj.transpose() * Eigen::Vector3d(ad, bd, cd)) / (-det);
			return value(bestPos);
		}
		else
		{
			return -1.0;
		}
	}

	double value(double x, double y, double z) const
	{
		return x*x*a2 + 2*x*y*ab + 2*x*z*ac + 2*x*ad +
			y*y*b2 + 2*y*z*bc + 2*y*bd + 
			z*z*c2 + 2*z*cd +
			d2;
	}

	double value(const Eigen::Vector3d& bestPos) const
	{
		return value(bestPos[0], bestPos[1], bestPos[2]);
	}
};