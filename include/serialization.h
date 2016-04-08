#pragma once
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct microser
{
	microser(std::ostream & ons) : ons_(ons)
	{}

	std::ostream & ons_;
};

inline microser & operator << (microser & x, const uint32_t & y)
{
	x.ons_.write((const char*)&y,4);
	return x;
}

inline microser & operator << (microser & x, const float & y)
{
	x.ons_.write((const char*)&y,4);
	return x;
}

inline microser & operator << (microser & x, const uint8_t &y)
{
	x.ons_.write((const char*)&y,1);
	return x;	
}
 
inline microser & operator << (microser & x, const double & y)
{
	x.ons_.write((const char*)&y,8);
	return x;	
}

inline microser & operator << (microser & x, const uint64_t & y)
{
	x.ons_.write((const char*)&y,8);
	return x;
}

inline microser & operator << (microser & x, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & y)
{
	x.ons_.write((const char*)&(y->points[0]), sizeof(pcl::PointXYZRGB) * y->size());
	return x;
}



BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost {
namespace serialization {
 
    /** Serialization support for cv::Mat */
    template <class Archive>
    void save(Archive & ar, const ::cv::Mat & m, const unsigned int version)
    {
		size_t elem_size = m.elemSize();
		size_t elem_type = m.type();

		ar & m.cols;
		ar & m.rows;
		ar & elem_size;
		ar & elem_type;

		const size_t data_size = m.cols * m.rows * elem_size;
		ar & boost::serialization::make_array(m.ptr(), data_size);
    }
 
    /** Serialization support for cv::Mat */
    template <class Archive>
    void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
    {
		int cols, rows;
		size_t elem_size, elem_type;

		ar & cols;
		ar & rows;
		ar & elem_size;
		ar & elem_type;

		m.create(rows, cols, elem_type);

		size_t data_size = m.cols * m.rows * elem_size;
		ar & boost::serialization::make_array(m.ptr(), data_size);
    }
}
}

