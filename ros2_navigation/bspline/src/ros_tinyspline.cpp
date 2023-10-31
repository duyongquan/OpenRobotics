#include "bspline/ros_tinyspline.hpp"

#include <stdexcept>
#include <cstdio>

namespace bspline 
{

/******************************************************************************
*                                                                             *
* DeBoorNet                                                                   *
*                                                                             *
******************************************************************************/
DeBoorNet::DeBoorNet()
{
	ts_deboornet_default(&net);
}

DeBoorNet::DeBoorNet(const DeBoorNet &other)
{
	tsError err = ts_deboornet_copy(&other.net, &net);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
}

DeBoorNet::~DeBoorNet()
{
	ts_deboornet_free(&net);
}

DeBoorNet & DeBoorNet::operator=(const DeBoorNet &other)
{
	if (&other != this) {
		tsError err = ts_deboornet_copy(&other.net, &net);
		if (err < 0)
			throw std::runtime_error(ts_enum_str(err));
	}
	return *this;
}

real DeBoorNet::knot() const
{
	return ts_deboornet_knot(&net);
}

size_t DeBoorNet::index() const
{
	return ts_deboornet_index(&net);
}

size_t DeBoorNet::multiplicity() const
{
	return ts_deboornet_multiplicity(&net);
}

size_t DeBoorNet::numInsertions() const
{
	return ts_deboornet_num_insertions(&net);
}

size_t DeBoorNet::dimension() const
{
	return ts_deboornet_dimension(&net);
}

std::vector<real> DeBoorNet::points() const
{
	tsReal *points;
	tsError err = ts_deboornet_points(&net, &points);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	size_t num_points = ts_deboornet_num_points(&net);
	real *begin = points;
	real *end = begin + num_points * dimension();
	std::vector<real> vec =
		std::vector<real>(begin, end);
	delete points;
	return vec;
}

std::vector<real> DeBoorNet::result() const
{
	tsReal *result;
	tsError err = ts_deboornet_result(&net, &result);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	size_t num_result = ts_deboornet_num_result(&net);
	real *begin = result;
	real *end = begin + num_result * dimension();
	std::vector<real> vec =
		std::vector<real>(begin, end);
	delete result;
	return vec;
}

tsDeBoorNet * DeBoorNet::data()
{
	return &net;
}



/******************************************************************************
*                                                                             *
* BSpline                                                                     *
*                                                                             *
******************************************************************************/
BSpline::BSpline()
{
	ts_bspline_default(&spline);
}

BSpline::BSpline(const BSpline &other)
{
	tsError err = ts_bspline_copy(&other.spline, &spline);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
}

BSpline::BSpline(size_t nCtrlp, size_t dim, size_t deg,
	BSpline::type type)
{
	tsError err = ts_bspline_new(nCtrlp, dim, deg, type, &spline);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
}

BSpline::~BSpline()
{
	ts_bspline_free(&spline);
}

BSpline & BSpline::operator=(
	const BSpline &other)
{
	if (&other != this) {
		tsError err = ts_bspline_copy(&other.spline, &spline);
		if (err < 0)
			throw std::runtime_error(ts_enum_str(err));
	}
	return *this;
}

DeBoorNet BSpline::operator()(real u) const
{
	return eval(u);
}

size_t BSpline::degree() const
{
	return ts_bspline_degree(&spline);
}

size_t BSpline::order() const
{
	return ts_bspline_order(&spline);
}

size_t BSpline::dimension() const
{
	return ts_bspline_dimension(&spline);
}

std::vector<real> BSpline::controlPoints() const
{
	tsReal *ctrlp;
	tsError err = ts_bspline_control_points(&spline, &ctrlp);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	size_t num_ctrlp = ts_bspline_num_control_points(&spline);
	real *begin  = ctrlp;
	real *end = begin + num_ctrlp * dimension();
	std::vector<real> vec =
		std::vector<real>(begin, end);
	delete ctrlp;
	return vec;
}

std::vector<real> BSpline::knots() const
{
	tsReal *knots;
	tsError err = ts_bspline_knots(&spline, &knots);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	size_t num_knots = ts_bspline_num_knots(&spline);
	real *begin = knots;
	real *end = begin + num_knots;
	std::vector<real> vec =
		std::vector<real>(begin, end);
	delete knots;
	return vec;
}

tsBSpline * BSpline::data()
{
	return &spline;
}

DeBoorNet BSpline::eval(real u) const
{
	DeBoorNet deBoorNet;
	tsError err = ts_bspline_eval(&spline, u, deBoorNet.data());
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	return deBoorNet;
}

void BSpline::setControlPoints(
	const std::vector<real> &ctrlp)
{
	size_t expected = ts_bspline_len_control_points(&spline);
	size_t actual = ctrlp.size();
	if (expected != actual) {
		char expected_str[32];
		char actual_str[32];
		sprintf(expected_str, "%zu", expected);
		sprintf(actual_str, "%zu", actual);
		throw std::runtime_error(
			"Expected size: " + std::string(expected_str) +
			", Actual size: " + std::string(actual_str));
	}
	tsError err = ts_bspline_set_control_points(&spline, ctrlp.data());
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
}

void BSpline::setKnots(const std::vector<real> &knots)
{
	size_t expected = ts_bspline_num_knots(&spline);
	size_t actual = knots.size();
	if (expected != actual) {
		char expected_str[32];
		char actual_str[32];
		sprintf(expected_str, "%zu", expected);
		sprintf(actual_str, "%zu", actual);
		throw std::runtime_error(
			"Expected size: " + std::string(expected_str) +
			", Actual size: " + std::string(actual_str));
	}
	tsError err = ts_bspline_set_knots(&spline, knots.data());
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
}

BSpline BSpline::fillKnots(tsBSplineType type,
	real min, real max) const
{
	BSpline bs;
	tsError err = ts_bspline_fill_knots(
		&spline, type, min, max, &bs.spline);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	return bs;
}

BSpline BSpline::insertKnot(real u,
	size_t n) const
{
	BSpline bs;
	size_t k;
	tsError err = ts_bspline_insert_knot(&spline, u, n, &bs.spline, &k);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	return bs;
}

BSpline BSpline::resize(int n, int back) const
{
	BSpline bs;
	tsError err = ts_bspline_resize(&spline, n, back, &bs.spline);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	return bs;
}

BSpline BSpline::split(real u) const
{
	BSpline bs;
	size_t k;
	tsError err = ts_bspline_split(&spline, u, &bs.spline, &k);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	return bs;
}

BSpline BSpline::buckle(real b) const
{
	BSpline bs;
	tsError err = ts_bspline_buckle(&spline, b, &bs.spline);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	return bs;
}

BSpline BSpline::toBeziers() const
{
	BSpline bs;
	tsError err = ts_bspline_to_beziers(&spline, &bs.spline);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	return bs;
}

BSpline BSpline::derive() const
{
	BSpline bs;
	tsError err = ts_bspline_derive(&spline, &bs.spline);
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	return bs;
}



/******************************************************************************
*                                                                             *
* Utils                                                                       *
*                                                                             *
******************************************************************************/
BSpline Utils::interpolateCubic(
	const std::vector<real> *points, size_t dim)
{
	if (dim == 0)
		throw std::runtime_error(ts_enum_str(TS_DIM_ZERO));
	if (points->size() % dim != 0)
		throw std::runtime_error("#points % dim == 0 failed");
	BSpline bspline;
	tsError err = ts_bspline_interpolate_cubic(
		points->data(), points->size()/dim, dim, bspline.data());
	if (err < 0)
		throw std::runtime_error(ts_enum_str(err));
	return bspline;
}

bool Utils::fequals(real x, real y)
{
	return ts_fequals(x, y) == 1;
}

std::string Utils::enum_str(tsError err)
{
	return std::string(ts_enum_str(err));
}

tsError Utils::str_enum(std::string str)
{
	return ts_str_enum(str.c_str());
}

}  // namespace bspline
